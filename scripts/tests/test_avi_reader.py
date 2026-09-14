#!/usr/bin/env python3
"""Compile the production AVI reader on a host; no ESP-IDF or hardware access.

Run with Python 3 and g++ on Linux/WSL. ASan and UBSan cover malformed RIFF,
stream selection, bounded JPEG/PCM payloads, padding, nesting and stream counts.
The prepared Big Buck Bunny sample is also checked when present (--sample can
select its location). Synthetic JPEG fixtures test framing, not JPEG decoding.
"""

import argparse
import os
from pathlib import Path
import struct
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
BOARD = ROOT / "main/boards/lichuang-dev-ml307"

HARNESS = r'''
#include "avi_reader.h"
#include <cassert>
#include <fstream>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    assert(argc == 2);
    const std::string root = argv[1];
    std::ifstream manifest(root + "/manifest.txt");
    assert(manifest);
    std::string filename;
    int expected, audio;
    unsigned checked = 0;
    AviReader reader;
    std::vector<uint8_t> jpeg{1, 2, 3};
    assert(!reader.Open(nullptr));
    assert(reader.NextFrame(jpeg) == AviReader::Result::kError && jpeg.empty());
    assert(reader.NextAudio(jpeg) == AviReader::Result::kError && jpeg.empty());
    while (manifest >> filename >> expected >> audio) {
        FILE* file = fopen((root + "/" + filename).c_str(), "rb");
        assert(file);
        const bool opened = reader.Open(file);
        if (!opened) {
            if (expected >= 0) std::cerr << "Open failed: " << filename << '\n';
            assert(expected < 0);
            assert(reader.NextFrame(jpeg) == AviReader::Result::kError && jpeg.empty());
            fclose(file);
            ++checked;
            continue;
        }
        if (expected >= 0) {
            assert(reader.info().width == 320 && reader.info().height == 240);
            assert(reader.info().frame_interval_us == 100000);
            assert(reader.info().frame_count == static_cast<unsigned>(expected));
            assert(reader.info().has_audio == static_cast<bool>(audio));
        }
        unsigned frames = 0;
        AviReader::Result result;
        do {
            result = reader.NextFrame(jpeg);
            if (result == AviReader::Result::kFrame) {
                assert(!jpeg.empty() && jpeg.size() <= 256 * 1024);
                assert(++frames < 10000);
            } else {
                assert(jpeg.empty());
            }
        } while (result == AviReader::Result::kFrame);
        if (expected < 0) {
            if (result != AviReader::Result::kError) std::cerr << "Accepted: " << filename << '\n';
            assert(result == AviReader::Result::kError);
        } else {
            if (result != AviReader::Result::kEnd || frames != static_cast<unsigned>(expected))
                std::cerr << "Frames failed: " << filename << " count=" << frames << '\n';
            assert(result == AviReader::Result::kEnd && frames == static_cast<unsigned>(expected));
        }
        assert(reader.NextFrame(jpeg) == result && jpeg.empty());
        // Open never owns/closes the file, including after a parser failure.
        assert(fseek(file, 0, SEEK_SET) == 0);
        assert(fclose(file) == 0);
        ++checked;
    }
    std::cout << "PASS: " << checked << " AVI fixtures, stream selection, limits, padding, "
              << "JPEG safety, malformed input and reader reuse\n";
    std::ifstream audio_manifest(root + "/audio_manifest.txt");
    assert(audio_manifest);
    unsigned rate, channels, supported, has_audio, expected_hash;
    checked = 0;
    while (audio_manifest >> filename >> expected >> supported >> rate >> channels >>
           has_audio >> expected_hash) {
        FILE* file = fopen((root + "/" + filename).c_str(), "rb");
        assert(file && reader.Open(file));
        assert(reader.info().has_audio == static_cast<bool>(has_audio));
        assert(reader.info().audio_supported == static_cast<bool>(supported));
        if (supported) {
            assert(reader.info().audio_sample_rate == rate);
            assert(reader.info().audio_channels == channels);
            assert(reader.info().audio_bits_per_sample == 16);
            if (expected >= 0)
                assert(reader.info().audio_sample_count == static_cast<unsigned>(expected));
        }
        unsigned samples = 0;
        uint32_t hash = 2166136261U;
        AviReader::Result result;
        do {
            result = reader.NextAudio(jpeg);
            if (result == AviReader::Result::kAudio) {
                assert(supported && !jpeg.empty() && jpeg.size() <= 256 * 1024);
                assert(jpeg.size() % (channels * 2) == 0);
                samples += jpeg.size() / (channels * 2);
                assert(samples <= reader.info().audio_sample_count);
                for (uint8_t byte : jpeg) hash = (hash ^ byte) * 16777619U;
            } else {
                assert(jpeg.empty());
            }
        } while (result == AviReader::Result::kAudio);
        if ((expected < 0 && result != AviReader::Result::kError) ||
            (expected >= 0 && (result != AviReader::Result::kEnd ||
                              samples != static_cast<unsigned>(expected) || hash != expected_hash))) {
            std::cerr << "Audio failed: " << filename << " samples=" << samples << '\n';
            assert(false);
        }
        assert(reader.NextAudio(jpeg) == result && jpeg.empty());
        assert(fseek(file, 0, SEEK_SET) == 0 && fclose(file) == 0);
        ++checked;
    }
    // Independent handles can consume the two streams at different speeds.
    FILE* video_file = fopen((root + "/pcm_mono.avi").c_str(), "rb");
    FILE* audio_file = fopen((root + "/pcm_mono.avi").c_str(), "rb");
    AviReader audio_reader;
    assert(video_file && audio_file && reader.Open(video_file) && audio_reader.Open(audio_file));
    assert(reader.NextFrame(jpeg) == AviReader::Result::kFrame);
    assert(audio_reader.NextAudio(jpeg) == AviReader::Result::kAudio);
    assert(reader.NextFrame(jpeg) == AviReader::Result::kEnd);
    assert(audio_reader.NextAudio(jpeg) == AviReader::Result::kAudio);
    assert(audio_reader.NextAudio(jpeg) == AviReader::Result::kEnd);
    // Mixing cursor modes cannot accidentally succeed with partial stream data.
    assert(audio_reader.NextFrame(jpeg) == AviReader::Result::kError && jpeg.empty());
    assert(fclose(video_file) == 0 && fclose(audio_file) == 0);
    std::cout << "PASS: " << checked << " PCM fixtures, exact samples/bytes, independent cursors, "
              << "unsupported metadata, truncation and allocation bounds\n";
}
'''


def le32(value):
    return struct.pack("<I", value)


def chunk(kind, data, pad=True):
    return kind + le32(len(data)) + data + (b"\0" if pad and len(data) & 1 else b"")


def riff(data):
    return chunk(b"RIFF", b"AVI " + data)


def listing(kind, data):
    return chunk(b"LIST", kind + data)


def jpeg(width=320, height=240, marker=0xC0, precision=8):
    sof = bytes([precision]) + struct.pack(">HHB", height, width, 3)
    sof += bytes([1, 0x22, 0, 2, 0x11, 1, 3, 0x11, 1])
    sos = bytes([3, 1, 0, 2, 0x11, 3, 0x11, 0, 63, 0])
    return (b"\xff\xd8\xff" + bytes([marker]) + struct.pack(">H", len(sof) + 2) + sof +
            b"\xff\xda" + struct.pack(">H", len(sos) + 2) + sos + b"\x12\xff\x00\x34\xff\xd9")


def stream(kind=b"vids", codec=b"MJPG", width=320, height=240, scale=1, rate=10,
           count=1, format_codec=None, prefix=b"", suffix=b""):
    header = bytearray(56)
    header[:8] = kind + codec
    struct.pack_into("<IIII", header, 20, scale, rate, 0, count)
    if kind == b"vids":
        fmt = struct.pack("<IiiHH4sIiiII", 40, width, height, 1, 24,
                          codec if format_codec is None else format_codec, 0, 0, 0, 0, 0)
    else:
        fmt = struct.pack("<HHIIHH", 1, 1, 16000, 32000, 2, 16)
    return listing(b"strl", prefix + chunk(b"strh", header) + chunk(b"strf", fmt) + suffix)


def pcm_stream(channels=1, sample_rate=16000, count=4, format_tag=1, bits=16,
               alignment=None, byte_rate=None, scale=1, rate=None, start=0,
               sample_size=None, format_tail=b"", truncate_format=None):
    alignment = channels * 2 if alignment is None else alignment
    byte_rate = sample_rate * alignment if byte_rate is None else byte_rate
    rate = sample_rate * scale if rate is None else rate
    sample_size = alignment if sample_size is None else sample_size
    header = bytearray(56)
    header[:4] = b"auds"
    struct.pack_into("<IIII", header, 20, scale, rate, start, count)
    struct.pack_into("<I", header, 44, sample_size)
    fmt = struct.pack("<HHIIHH", format_tag, channels, sample_rate, byte_rate,
                      alignment, bits) + format_tail
    if truncate_format is not None:
        fmt = fmt[:truncate_format]
    return listing(b"strl", chunk(b"strh", header) + chunk(b"strf", fmt))


def fnv1a(data):
    value = 2166136261
    for byte in data:
        value = ((value ^ byte) * 16777619) & 0xFFFFFFFF
    return value


def movie_audio(data):
    """Independently collect stream 01 PCM from the prepared sample for exact comparison."""
    def walk(start, end):
        result = b""
        while start < end:
            kind, size = struct.unpack_from("<4sI", data, start)
            body = start + 8
            assert body + size <= end
            if kind == b"LIST" and data[body:body + 4] in (b"movi", b"rec "):
                result += walk(body + 4, body + size)
            elif kind == b"01wb":
                result += data[body:body + size]
            start = body + size + (size & 1)
        return result
    return walk(12, len(data))


def headers(streams=None, width=320, height=240, interval=100000, count=1, extra=b""):
    streams = [stream(count=count)] if streams is None else streams
    avih = bytearray(56)
    struct.pack_into("<I", avih, 0, interval)
    struct.pack_into("<I", avih, 16, count)
    struct.pack_into("<III", avih, 24, len(streams), 0, width)
    struct.pack_into("<I", avih, 36, height)
    return listing(b"hdrl", extra + chunk(b"avih", avih) + b"".join(streams))


def avi(frames=None, header=None, extra=b""):
    frames = chunk(b"00dc", jpeg()) if frames is None else frames
    header = headers() if header is None else header
    return riff(header + listing(b"movi", frames) + extra)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sample", type=Path, default=ROOT /
                        "tmp/video-test/big_buck_bunny_320x240_10fps_mjpeg_pcm.avi")
    args = parser.parse_args()
    cases = []
    audio_cases = []

    def add(name, data, expected=-1, audio=False):
        cases.append((name, data, expected, int(audio)))

    def add_pcm(name, data, samples=-1, supported=True, rate=16000, channels=1,
                payload=b"", video_frames=1, has_audio=True):
        add(name, data, video_frames, has_audio)
        audio_cases.append((name, samples, int(supported), rate, channels,
                            int(has_audio), fnv1a(payload)))

    pcm = struct.pack("<hhhh", -32768, -1234, 1234, 32767)
    pcm_header = headers([stream(), pcm_stream()])
    pcm_chunks = chunk(b"01wb", pcm[:4]) + chunk(b"00dc", jpeg()) + chunk(b"01wb", pcm[4:])
    add_pcm("pcm_mono.avi", avi(pcm_chunks, pcm_header), 4, payload=pcm)
    add_pcm("pcm_waveformatex.avi", avi(pcm_chunks, headers([
        stream(), pcm_stream(format_tail=b"\0\0")])), 4, payload=pcm)
    stereo = struct.pack("<hhhhhhhh", -32768, 32767, -1, 1, 0, 1000, 2000, -2000)
    for rate in (8000, 11025, 22050, 44100, 48000):
        add_pcm(f"pcm_stereo_{rate}.avi", avi(chunk(b"00dc", jpeg()) + chunk(b"01wb", stereo),
            headers([stream(), pcm_stream(channels=2, sample_rate=rate, scale=4)])),
            4, channels=2, rate=rate, payload=stereo)
    add_pcm("pcm_audio_first.avi", avi(chunk(b"00wb", pcm) + chunk(b"01dc", jpeg()),
        headers([pcm_stream(), stream()])), 4, payload=pcm)
    add_pcm("pcm_first_of_two.avi", avi(pcm_chunks + chunk(b"02wb", b"ignored"),
        headers([stream(), pcm_stream(), pcm_stream(format_tag=3)])), 4, payload=pcm)
    add_pcm("pcm_stream_ten.avi", avi(chunk(b"00dc", jpeg()) + chunk(b"10wb", pcm),
        headers([stream()] * 10 + [pcm_stream()])), 4, payload=pcm)
    add_pcm("pcm_nested.avi", avi(listing(b"rec ", pcm_chunks), pcm_header), 4, payload=pcm)
    add_pcm("pcm_skip_bad_jpeg.avi", avi(chunk(b"00dc", b"invalid") + chunk(b"01wb", pcm),
        pcm_header), 4, payload=pcm, video_frames=-1)
    add_pcm("pcm_no_audio.avi", avi(), 0, supported=False, has_audio=False)
    maximum_pcm = bytes(range(256)) * 1024
    add_pcm("pcm_maximum.avi", avi(chunk(b"00dc", jpeg()) + chunk(b"01wb", maximum_pcm),
        headers([stream(), pcm_stream(count=len(maximum_pcm) // 2)])),
        len(maximum_pcm) // 2, payload=maximum_pcm)
    for label, options in [
        ("format_float", {"format_tag": 3}),
        ("format_adpcm", {"format_tag": 2}),
        ("eight_bit", {"bits": 8}),
        ("24_bit", {"bits": 24}),
        ("zero_channels", {"channels": 0}),
        ("surround", {"channels": 6}),
        ("low_rate", {"sample_rate": 7999}),
        ("high_rate", {"sample_rate": 48001}),
        ("unsupported_rate_9000", {"sample_rate": 9000}),
        ("unsupported_rate_12001", {"sample_rate": 12001}),
        ("bad_alignment", {"alignment": 3}),
        ("bad_byte_rate", {"byte_rate": 1}),
        ("bad_time_base", {"rate": 1234}),
        ("zero_scale", {"scale": 0}),
        ("overflow_scale", {"scale": 0xFFFFFFFF, "rate": 16000}),
        ("nonzero_start", {"start": 1}),
        ("bad_sample_size", {"sample_size": 0}),
        ("zero_count", {"count": 0}),
        ("huge_count", {"count": 0xFFFFFFFF}),
        ("short_extension", {"format_tail": b"\0"}),
        ("nonempty_extension", {"format_tail": b"\x01\0\0"}),
    ]:
        add_pcm(f"pcm_unsupported_{label}.avi", avi(pcm_chunks,
            headers([stream(), pcm_stream(**options)])), supported=False)
    for length in range(16):
        add_pcm(f"pcm_format_cut_{length}.avi", avi(pcm_chunks,
            headers([stream(), pcm_stream(truncate_format=length)])), supported=False)
    for label, payload in [
        ("unaligned", chunk(b"01wb", pcm[:-1])),
        ("empty", chunk(b"01wb", b"")),
        ("missing", b""),
        ("short_count", chunk(b"01wb", pcm[:-2])),
        ("extra_count", chunk(b"01wb", pcm + b"\0\0")),
        ("wrong_stream", chunk(b"02wb", pcm)),
        ("wrong_suffix", chunk(b"01dc", pcm)),
    ]:
        add_pcm(f"pcm_invalid_{label}.avi", avi(chunk(b"00dc", jpeg()) + payload, pcm_header))
    oversize_pcm = maximum_pcm + b"\0\0"
    add_pcm("pcm_oversize.avi", avi(chunk(b"00dc", jpeg()) + chunk(b"01wb", oversize_pcm),
        headers([stream(), pcm_stream(count=len(oversize_pcm) // 2)])))
    add_pcm("pcm_chunk_boundary.avi", avi(chunk(b"00dc", jpeg()) +
        listing(b"rec ", b"01wb" + le32(100)) + chunk(b"JUNK", b"x" * 100), pcm_header),
        video_frames=-1)
    add_pcm("pcm_budget.avi", avi(chunk(b"00dc", jpeg()) + chunk(b"JUNK", b"") * 4096 +
        chunk(b"01wb", pcm), pcm_header), video_frames=-1)
    add_pcm("pcm_budget_resets.avi", avi(chunk(b"00dc", jpeg()) +
        (chunk(b"JUNK", b"") * 4090 + chunk(b"01wb", pcm[:4])) * 2, pcm_header),
        4, payload=pcm[:4] * 2, video_frames=-1)

    add("minimal.avi", avi(), 1)
    add("no_index.avi", avi(extra=chunk(b"JUNK", b"odd")), 1)
    odd_frame = jpeg()[:-2] + b"\x42\xff\xd9"
    add("odd_frame.avi", avi(chunk(b"00dc", odd_frame)), 1)
    mixed = (chunk(b"JUNK", b"odd") + chunk(b"00wb", b"audio") +
             listing(b"rec ", chunk(b"00dc", jpeg()) + chunk(b"JUNK", b"z")))
    add("odd_nested.avi", avi(mixed, headers(extra=chunk(b"JUNK", b"x"))), 1)
    audio_first = headers([stream(kind=b"auds", codec=b"\0" * 4), stream()])
    add("audio_first.avi", avi(chunk(b"00dc", b"not a jpeg") + chunk(b"00wb", b"pcm") +
                              chunk(b"01dc", jpeg()), audio_first), 1, True)
    add("large_audio_skip.avi", avi(chunk(b"00wb", b"a" * (300 * 1024)) +
                                   chunk(b"01dc", jpeg()), audio_first), 1, True)
    ten_streams = headers([stream(kind=b"auds", codec=b"\0" * 4)] * 10 + [stream()])
    add("stream_ten.avi", avi(chunk(b"00dc", b"wrong") + chunk(b"10dc", jpeg()), ten_streams), 1, True)
    add("db_jpeg.avi", avi(chunk(b"00db", jpeg())), 1)
    add("two_frames.avi", avi(chunk(b"00dc", jpeg()) * 2, headers(count=2)), 2)
    many_junk = chunk(b"JUNK", b"") * 4096
    add("root_junk_limit.avi", avi(extra=many_junk))
    add("header_junk_limit.avi", avi(header=headers(extra=many_junk)))
    add("stream_junk_limit.avi", avi(header=headers([stream(prefix=many_junk)])))
    add("movie_junk_limit.avi", avi(many_junk + chunk(b"00dc", jpeg())))
    # A near-limit valid file verifies that budgets reset for each frame call.
    normal_junk = chunk(b"JUNK", b"") * 4090
    add("junk_budget_resets.avi", avi((normal_junk + chunk(b"00dc", jpeg())) * 2,
                                     headers(count=2)), 2)
    max_frame = jpeg()[:-2] + b"\x42" * (256 * 1024 - len(jpeg())) + b"\xff\xd9"
    add("maximum_frame.avi", avi(chunk(b"00dc", max_frame)), 1)
    nested = chunk(b"00dc", jpeg())
    for depth in range(1, 10):
        nested = listing(b"rec ", nested)
        add(f"nest_{depth}.avi", avi(nested), 1 if depth <= 7 else -1)
    for label, header in [
        ("wide", headers([stream(width=321)], width=321)),
        ("tall", headers([stream(height=241)], height=241)),
        ("zero_width", headers([stream(width=0)], width=0)),
        ("negative_width", headers([stream(width=-1)])),
        ("negative_height", headers([stream(height=-240)])),
        ("wrong_codec", headers([stream(codec=b"H264")])),
        ("wrong_format_codec", headers([stream(format_codec=b"XVID")])),
        ("dimensions_disagree", headers(width=319)),
        ("zero_scale", headers([stream(scale=0)])),
        ("zero_rate", headers([stream(rate=0)])),
        ("too_fast", headers([stream(rate=31)])),
        ("too_slow", headers([stream(scale=2, rate=1)])),
        ("overflow_rate", headers([stream(scale=0xFFFFFFFF, rate=1)])),
        ("zero_count", headers(count=0)),
        ("huge_count", headers(count=0xFFFFFFFF)),
        ("fast_avih", headers(interval=1)),
        ("no_video", headers([stream(kind=b"auds", codec=b"\0" * 4)])),
        ("101_streams", headers([stream()] * 101)),
        ("duplicate_strh", headers([stream(suffix=chunk(b"strh", b"\0" * 56))])),
        ("short_strl_chunk", headers([stream(suffix=b"1234567")])),
    ]:
        add(label + ".avi", avi(header=header))
    for label, frame in [
        ("progressive", jpeg(marker=0xC2)),
        ("lossless", jpeg(marker=0xC3)),
        ("precision12", jpeg(precision=12)),
        ("jpeg_huge_width", jpeg(width=65535)),
        ("jpeg_huge_height", jpeg(height=65535)),
        ("jpeg_zero_height", jpeg(height=0)),
        ("jpeg_mismatch", jpeg(width=319)),
        ("no_soi", jpeg()[2:]),
        ("no_eoi", jpeg()[:-2]),
        ("trailing_jpeg_bytes", jpeg() + b"bad"),
        ("empty_frame", b""),
        ("giant_frame", b"x" * (256 * 1024 + 1)),
        ("zero_segment", b"\xff\xd8\xff\xe0\x00\x00" + jpeg()[2:]),
        ("truncated_segment", b"\xff\xd8\xff\xe0\xff\xff" + jpeg()[2:]),
        ("soi_eoi_only", b"\xff\xd8\xff\xd9"),
    ]:
        add(label + ".avi", avi(chunk(b"00dc", frame)))
    for length in range(len(jpeg())):
        add(f"jpeg_cut_{length}.avi", avi(chunk(b"00dc", jpeg()[:length])))
    add("missing_frame.avi", avi(frames=b""))
    add("extra_frame.avi", avi(chunk(b"00dc", jpeg()) * 2))
    add("wrong_stream_only.avi", avi(chunk(b"01dc", jpeg())))
    add("wrong_suffix_only.avi", avi(chunk(b"00wb", jpeg())))
    add("missing_padding.avi", avi(chunk(b"JUNK", b"x", pad=False) + chunk(b"00dc", jpeg())))
    add("chunk_overflow.avi", avi(b"00dc" + le32(0xFFFFFFFF)))
    add("audio_chunk_overflow.avi", avi(b"00wb" + le32(0xFFFFFFFF)))
    add("list_too_short.avi", avi(chunk(b"LIST", b"rec")))
    add("list_boundary.avi", avi(listing(b"rec ", b"00dc" + le32(100)) + chunk(b"JUNK", b"x" * 100)))
    add("partial_header.avi", avi(b"00dc123"))
    add("duplicate_hdrl.avi", avi(extra=headers()))
    add("duplicate_movi.avi", avi(extra=listing(b"movi", chunk(b"00dc", jpeg()))))
    add("wrong_riff_type.avi", avi().replace(b"AVI ", b"AVIX", 1))
    add("riff_overflow.avi", b"RIFF" + le32(0xFFFFFFFF) + avi()[8:])
    add("riff_short.avi", b"RIFF" + le32(4) + avi()[8:])
    add("trailing_riff.avi", avi() + avi())
    valid = avi()
    for length in range(len(valid)):
        add(f"file_cut_{length}.avi", valid[:length])
    # Keep the outer RIFF length valid while truncating each nested byte boundary.
    for length in range(12, len(valid)):
        add(f"riff_cut_{length}.avi", valid[:4] + le32(length - 8) + valid[8:length])
    if args.sample.exists():
        sample = args.sample.read_bytes()
        sample_pcm = movie_audio(sample)
        assert len(sample_pcm) == 192000 * 2
        add_pcm("big_buck_bunny.avi", sample, 192000, payload=sample_pcm, video_frames=120)
    else:
        print(f"SKIP: prepared 120-frame sample is absent: {args.sample}", flush=True)
    with tempfile.TemporaryDirectory(prefix="avi-reader-test-") as name:
        directory = Path(name)
        (directory / "harness.cc").write_text(HARNESS, encoding="utf-8")
        manifest = []
        for filename, data, expected, audio in cases:
            (directory / filename).write_bytes(data)
            manifest.append(f"{filename} {expected} {audio}\n")
        (directory / "manifest.txt").write_text("".join(manifest), encoding="utf-8")
        (directory / "audio_manifest.txt").write_text(
            "".join(" ".join(map(str, row)) + "\n" for row in audio_cases), encoding="utf-8")
        binary = directory / "test"
        subprocess.run([os.environ.get("CXX", "g++"), "-std=c++17", "-Wall", "-Wextra", "-Werror",
                        "-g", "-fsanitize=address,undefined", "-I", str(BOARD),
                        str(directory / "harness.cc"), str(BOARD / "avi_reader.cc"),
                        "-o", str(binary)], check=True)
        subprocess.run([str(binary), str(directory)], check=True, timeout=30)
        if args.sample.exists():
            print("PASS: real sample has 120 validated 320x240 frames and 192000 exact mono PCM samples")


if __name__ == "__main__":
    main()
