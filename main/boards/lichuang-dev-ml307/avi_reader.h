#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

// Bounded, sequential single-RIFF MJPEG reader. The caller owns the FILE and
// keeps it open for the reader's lifetime. Audio and AVI indexes are skipped.
class AviReader {
public:
    struct Info {
        uint32_t width = 0;
        uint32_t height = 0;
        uint32_t frame_interval_us = 0;
        uint32_t frame_count = 0;
        bool has_audio = false;
    };
    enum class Result { kFrame, kEnd, kError };

    bool Open(FILE* file);
    const Info& info() const { return info_; }
    Result NextFrame(std::vector<uint8_t>& jpeg);

private:
    struct Chunk {
        uint32_t id = 0;
        uint32_t size = 0;
        uint64_t data = 0;
        uint64_t end = 0;
        uint64_t next = 0;
    };
    struct List {
        uint64_t end = 0;
        uint64_t resume = 0;
    };
    FILE* file_ = nullptr;
    Info info_;
    uint64_t file_size_ = 0;
    uint64_t position_ = 0;
    std::array<List, 8> lists_{};
    size_t depth_ = 0;
    uint32_t video_stream_ = 0;
    uint32_t frames_read_ = 0;
    uint32_t remaining_chunks_ = 0;
    bool failed_ = true;

    bool Read(uint64_t offset, void* data, size_t size);
    bool ReadChunk(uint64_t position, uint64_t end, Chunk& chunk);
    bool ParseHeaders(const Chunk& header);
    bool ParseStream(const Chunk& stream, uint32_t index, bool& selected);
    bool ValidateJpeg(const std::vector<uint8_t>& jpeg) const;
};
