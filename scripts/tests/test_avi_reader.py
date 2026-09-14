#!/usr/bin/env python3
"""Compile the production AVI reader on a host; no ESP-IDF or hardware access.

Run with Python 3 and g++ on Linux/WSL. ASan and UBSan cover malformed RIFF,
stream selection, bounded JPEG headers, padding, nesting and frame counts.
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

    def add(name, data, expected=-1, audio=False):
        cases.append((name, data, expected, int(audio)))

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
        add("big_buck_bunny.avi", args.sample.read_bytes(), 120, True)
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
        binary = directory / "test"
        subprocess.run([os.environ.get("CXX", "g++"), "-std=c++17", "-Wall", "-Wextra", "-Werror",
                        "-g", "-fsanitize=address,undefined", "-I", str(BOARD),
                        str(directory / "harness.cc"), str(BOARD / "avi_reader.cc"),
                        "-o", str(binary)], check=True)
        subprocess.run([str(binary), str(directory)], check=True, timeout=30)
        if args.sample.exists():
            print("PASS: real MJPEG + PCM sample has 120 validated 320x240 frames at 10 fps")


if __name__ == "__main__":
    main()
