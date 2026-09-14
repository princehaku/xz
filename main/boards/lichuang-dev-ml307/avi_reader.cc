#include "avi_reader.h"

#include <climits>

namespace {
constexpr uint32_t FourCc(char a, char b, char c, char d) {
    return static_cast<uint8_t>(a) | (static_cast<uint32_t>(b) << 8) |
           (static_cast<uint32_t>(c) << 16) | (static_cast<uint32_t>(d) << 24);
}

uint32_t Le32(const uint8_t* data) {
    return data[0] | (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
}

uint16_t Le16(const uint8_t* data) { return data[0] | (static_cast<uint16_t>(data[1]) << 8); }

uint16_t Be16(const uint8_t* data) { return (static_cast<uint16_t>(data[0]) << 8) | data[1]; }

constexpr uint32_t kList = FourCc('L', 'I', 'S', 'T');
constexpr uint32_t kMjpg = FourCc('M', 'J', 'P', 'G');
constexpr size_t kMaxJpegBytes = 256 * 1024;
constexpr size_t kMaxPcmBytes = 256 * 1024;
// Bound SD reads while callers cannot check cancellation or yield. Open shares
// one budget across root/header/stream chunks; each NextFrame starts a new one.
constexpr uint32_t kMaxChunksPerCall = 4096;
}  // namespace

bool AviReader::Read(uint64_t offset, void* data, size_t size) {
    if (offset > file_size_ || size > file_size_ - offset || offset > LONG_MAX) {
        return false;
    }
    return fseek(file_, static_cast<long>(offset), SEEK_SET) == 0 &&
           fread(data, 1, size, file_) == size;
}

bool AviReader::ReadChunk(uint64_t position, uint64_t end, Chunk& chunk) {
    if (remaining_chunks_ == 0) {
        return false;
    }
    --remaining_chunks_;
    uint8_t header[8];
    if (position > end || end - position < sizeof(header) ||
        !Read(position, header, sizeof(header))) {
        return false;
    }
    chunk.id = Le32(header);
    chunk.size = Le32(header + 4);
    chunk.data = position + sizeof(header);
    chunk.end = chunk.data + chunk.size;
    chunk.next = chunk.end + (chunk.size & 1);
    return chunk.next <= end;
}

bool AviReader::Open(FILE* file) {
    *this = AviReader();
    remaining_chunks_ = kMaxChunksPerCall;
    file_ = file;
    if (!file_ || fseek(file_, 0, SEEK_END) != 0) {
        return false;
    }
    const long length = ftell(file_);
    if (length < 12) {
        return false;
    }
    file_size_ = static_cast<uint64_t>(length);
    uint8_t riff[12];
    if (!Read(0, riff, sizeof(riff)) || Le32(riff) != FourCc('R', 'I', 'F', 'F') ||
        Le32(riff + 8) != FourCc('A', 'V', 'I', ' ') ||
        static_cast<uint64_t>(Le32(riff + 4)) + 8 != file_size_) {
        return false;
    }
    bool headers_found = false;
    bool movie_found = false;
    for (uint64_t position = 12; position < file_size_;) {
        Chunk chunk;
        if (!ReadChunk(position, file_size_, chunk)) {
            return false;
        }
        if (chunk.id == kList) {
            uint8_t kind[4];
            if (chunk.size < sizeof(kind) || !Read(chunk.data, kind, sizeof(kind))) {
                return false;
            }
            if (Le32(kind) == FourCc('h', 'd', 'r', 'l')) {
                if (headers_found || !ParseHeaders(chunk)) {
                    return false;
                }
                headers_found = true;
            } else if (Le32(kind) == FourCc('m', 'o', 'v', 'i')) {
                if (movie_found) {
                    return false;
                }
                movie_found = true;
                position_ = chunk.data + 4;
                lists_[0] = {chunk.end, chunk.next};
                depth_ = 1;
            }
        }
        position = chunk.next;
    }
    failed_ = !(headers_found && movie_found);
    return !failed_;
}

bool AviReader::ParseHeaders(const Chunk& header) {
    uint8_t avih[56]{};
    bool main_found = false;
    bool selected = false;
    uint32_t streams = 0;
    for (uint64_t position = header.data + 4; position < header.end;) {
        Chunk chunk;
        if (!ReadChunk(position, header.end, chunk)) {
            return false;
        }
        if (chunk.id == FourCc('a', 'v', 'i', 'h')) {
            if (main_found || chunk.size < sizeof(avih) || !Read(chunk.data, avih, sizeof(avih))) {
                return false;
            }
            main_found = true;
        } else if (chunk.id == kList) {
            uint8_t kind[4];
            if (chunk.size < sizeof(kind) || !Read(chunk.data, kind, sizeof(kind))) {
                return false;
            }
            if (Le32(kind) == FourCc('s', 't', 'r', 'l')) {
                if (streams >= 100 || !ParseStream(chunk, streams, selected)) {
                    return false;
                }
                ++streams;
            }
        }
        position = chunk.next;
    }
    const uint32_t interval = Le32(avih);
    return main_found && selected && interval >= 33333 && interval <= 1000000 &&
           Le32(avih + 24) == streams && Le32(avih + 32) == info_.width &&
           Le32(avih + 36) == info_.height;
}

bool AviReader::ParseStream(const Chunk& stream, uint32_t index, bool& selected) {
    uint8_t strh[56]{};
    uint8_t strf[40]{};
    uint32_t format_size = 0;
    bool header_found = false;
    bool format_found = false;
    for (uint64_t position = stream.data + 4; position < stream.end;) {
        Chunk chunk;
        if (!ReadChunk(position, stream.end, chunk)) {
            return false;
        }
        if (chunk.id == FourCc('s', 't', 'r', 'h')) {
            if (header_found || chunk.size < sizeof(strh) ||
                !Read(chunk.data, strh, sizeof(strh))) {
                return false;
            }
            header_found = true;
        } else if (chunk.id == FourCc('s', 't', 'r', 'f')) {
            if (format_found) {
                return false;
            }
            format_found = true;
            format_size = chunk.size;
            const size_t read_size = chunk.size < sizeof(strf) ? chunk.size : sizeof(strf);
            if (read_size != 0 && !Read(chunk.data, strf, read_size)) {
                return false;
            }
        }
        position = chunk.next;
    }
    if (!header_found) {
        return false;
    }
    if (Le32(strh) == FourCc('a', 'u', 'd', 's')) {
        // Select the first audio stream, including when its codec is unsupported.
        // Video-only traversal remains available for such files.
        if (info_.has_audio) {
            return true;
        }
        info_.has_audio = true;
        audio_stream_ = index;
        if (!format_found || format_size < 16) {
            return true;
        }
        info_.audio_channels = Le16(strf + 2);
        info_.audio_sample_rate = Le32(strf + 4);
        info_.audio_bits_per_sample = Le16(strf + 14);
        info_.audio_sample_count = Le32(strh + 32);
        const uint32_t alignment = info_.audio_channels * 2U;
        const uint32_t scale = Le32(strh + 20);
        const uint32_t rate = Le32(strh + 24);
        info_.audio_supported =
            Le16(strf) == 1 && (info_.audio_channels == 1 || info_.audio_channels == 2) &&
            info_.audio_bits_per_sample == 16 && info_.audio_sample_rate >= 8000 &&
            info_.audio_sample_rate <= 48000 &&
            (info_.audio_sample_rate % 4000 == 0 || info_.audio_sample_rate % 11025 == 0) &&
            Le16(strf + 12) == alignment && Le32(strf + 8) == info_.audio_sample_rate * alignment &&
            (format_size == 16 || (format_size >= 18 && Le16(strf + 16) == 0)) && scale != 0 &&
            rate == static_cast<uint64_t>(info_.audio_sample_rate) * scale &&
            Le32(strh + 28) == 0 && Le32(strh + 44) == alignment && info_.audio_sample_count != 0 &&
            info_.audio_sample_count <= file_size_ / alignment;
        return true;
    }
    if (selected || Le32(strh) != FourCc('v', 'i', 'd', 's') || Le32(strh + 4) != kMjpg) {
        return true;
    }
    const uint32_t width = Le32(strf + 4);
    const uint32_t height = Le32(strf + 8);
    const uint32_t scale = Le32(strh + 20);
    const uint32_t rate = Le32(strh + 24);
    const uint32_t frames = Le32(strh + 32);
    if (!format_found || format_size < sizeof(strf) || Le32(strf) < sizeof(strf) ||
        Le32(strf) > format_size || Le32(strf + 16) != kMjpg || strf[12] != 1 || strf[13] != 0 ||
        width == 0 || width > 320 || height == 0 || height > 240 || scale == 0 || rate < scale ||
        rate > static_cast<uint64_t>(scale) * 30 || frames == 0 || frames > file_size_ / 8) {
        return false;
    }
    info_.width = width;
    info_.height = height;
    info_.frame_interval_us = static_cast<uint32_t>((1000000ULL * scale + rate / 2) / rate);
    info_.frame_count = frames;
    video_stream_ = index;
    selected = true;
    return true;
}

bool AviReader::ValidateJpeg(const std::vector<uint8_t>& jpeg) const {
    if (jpeg.size() < 4 || jpeg[0] != 0xff || jpeg[1] != 0xd8) {
        return false;
    }
    bool frame_found = false;
    uint8_t components = 0;
    size_t position = 2;
    while (position < jpeg.size()) {
        if (jpeg[position++] != 0xff) {
            return false;
        }
        while (position < jpeg.size() && jpeg[position] == 0xff) {
            ++position;
        }
        if (position == jpeg.size()) {
            return false;
        }
        const uint8_t marker = jpeg[position++];
        if (jpeg.size() - position < 2) {
            return false;
        }
        const size_t length = Be16(jpeg.data() + position);
        if (length < 2 || length > jpeg.size() - position) {
            return false;
        }
        const uint8_t* segment = jpeg.data() + position;
        if (marker == 0xc0) {
            if (frame_found || length < 8 || segment[2] != 8 || Be16(segment + 3) != info_.height ||
                Be16(segment + 5) != info_.width) {
                return false;
            }
            components = segment[7];
            if ((components != 1 && components != 3) || length != 8 + 3U * components) {
                return false;
            }
            frame_found = true;
        } else if (marker == 0xda) {
            if (!frame_found || length != 6 + 2U * components || segment[2] != components ||
                segment[length - 3] != 0 || segment[length - 2] != 63 || segment[length - 1] != 0) {
                return false;
            }
            position += length;
            // The supported baseline MJPEG frames have one interleaved scan.
            // Check entropy escapes and a final EOI before exposing the bytes.
            while (position < jpeg.size()) {
                if (jpeg[position++] != 0xff) {
                    continue;
                }
                while (position < jpeg.size() && jpeg[position] == 0xff) {
                    ++position;
                }
                if (position == jpeg.size()) {
                    return false;
                }
                const uint8_t entropy_marker = jpeg[position++];
                if (entropy_marker == 0xd9) {
                    return position == jpeg.size();
                }
                if (entropy_marker != 0 && (entropy_marker < 0xd0 || entropy_marker > 0xd7)) {
                    return false;
                }
            }
            return false;
        } else if (marker != 0xc4 && marker != 0xdb && marker != 0xdd && marker != 0xfe &&
                   (marker < 0xe0 || marker > 0xef)) {
            // Reject progressive, arithmetic, lossless and hierarchical JPEG.
            return false;
        }
        position += length;
    }
    return false;
}

AviReader::Result AviReader::NextFrame(std::vector<uint8_t>& jpeg) {
    return NextPacket(jpeg, false);
}

AviReader::Result AviReader::NextAudio(std::vector<uint8_t>& pcm) { return NextPacket(pcm, true); }

AviReader::Result AviReader::NextPacket(std::vector<uint8_t>& data, bool audio) {
    data.clear();
    if (failed_) {
        return Result::kError;
    }
    const uint8_t mode = audio ? 2 : 1;
    if ((stream_mode_ != 0 && stream_mode_ != mode) ||
        (audio && info_.has_audio && !info_.audio_supported)) {
        failed_ = true;
        return Result::kError;
    }
    stream_mode_ = mode;
    if (audio && !info_.has_audio) {
        return Result::kEnd;
    }
    remaining_chunks_ = kMaxChunksPerCall;
    while (depth_ != 0) {
        const List list = lists_[depth_ - 1];
        if (position_ == list.end) {
            position_ = list.resume;
            --depth_;
            continue;
        }
        Chunk chunk;
        if (!ReadChunk(position_, list.end, chunk)) {
            failed_ = true;
            return Result::kError;
        }
        position_ = chunk.next;
        if (chunk.id == kList) {
            uint8_t kind[4];
            if (chunk.size < sizeof(kind) || !Read(chunk.data, kind, sizeof(kind))) {
                failed_ = true;
                return Result::kError;
            }
            if (Le32(kind) == FourCc('r', 'e', 'c', ' ')) {
                if (depth_ == lists_.size()) {
                    failed_ = true;
                    return Result::kError;
                }
                lists_[depth_++] = {chunk.end, chunk.next};
                position_ = chunk.data + 4;
            }
            continue;
        }
        const uint32_t video_dc =
            FourCc('0' + video_stream_ / 10, '0' + video_stream_ % 10, 'd', 'c');
        const uint32_t video_db =
            FourCc('0' + video_stream_ / 10, '0' + video_stream_ % 10, 'd', 'b');
        const uint32_t audio_wb =
            FourCc('0' + audio_stream_ / 10, '0' + audio_stream_ % 10, 'w', 'b');
        if (audio ? chunk.id != audio_wb : (chunk.id != video_dc && chunk.id != video_db)) {
            continue;
        }
        if (audio) {
            const uint32_t alignment = info_.audio_channels * 2U;
            if (chunk.size == 0 || chunk.size > kMaxPcmBytes || chunk.size % alignment != 0 ||
                chunk.size / alignment > info_.audio_sample_count - audio_samples_read_) {
                failed_ = true;
                return Result::kError;
            }
            data.resize(chunk.size);
            if (!Read(chunk.data, data.data(), data.size())) {
                data.clear();
                failed_ = true;
                return Result::kError;
            }
            audio_samples_read_ += chunk.size / alignment;
            return Result::kAudio;
        }
        if (chunk.size == 0 || chunk.size > kMaxJpegBytes || frames_read_ >= info_.frame_count) {
            failed_ = true;
            return Result::kError;
        }
        data.resize(chunk.size);
        if (!Read(chunk.data, data.data(), data.size()) || !ValidateJpeg(data)) {
            data.clear();
            failed_ = true;
            return Result::kError;
        }
        ++frames_read_;
        return Result::kFrame;
    }
    if (audio ? audio_samples_read_ != info_.audio_sample_count
              : frames_read_ != info_.frame_count) {
        failed_ = true;
        return Result::kError;
    }
    return Result::kEnd;
}
