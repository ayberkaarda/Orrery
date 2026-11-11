#pragma once

#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include "orrery/core/result.h"
#include "orrery/render/image.h"

// Dependency-free PNG support for snapshots and golden images.
//
// The writer produces 8-bit RGBA, non-interlaced PNG files. Each row gets the
// filter (None, Sub, Up, Average or Paeth) with the smallest sum of absolute
// filtered values; the scanlines are compressed with deflate: LZ77 over a
// 32 KiB window with hash chains and one step of lazy matching, in blocks of
// up to 32768 tokens, each written as a stored, fixed-Huffman or dynamic-
// Huffman block (length-limited canonical codes), whichever is smallest.
// Output is deterministic and decodable by any standard decoder (tested
// against Python's zlib).
//
// The reader accepts 8-bit RGBA, non-interlaced PNG files with any row
// filter and all three deflate block types. It verifies the signature, every
// chunk CRC-32, the zlib header (no preset dictionary, window at most
// 32 KiB), every Huffman code (over-subscribed and incomplete codes are
// rejected, except the single one-bit code deflate allows), every length,
// distance and repeat against the data written so far, and the Adler-32
// checksum. Decompression stops with an error as soon as the output would
// exceed the (4 * width + 1) * height scanline bytes of the header, whose
// size is capped at 2^28 pixels, so a small file cannot make the reader
// allocate or inflate more than the image it announces.
namespace orrery::render {

[[nodiscard]] std::uint32_t crc32(std::span<const std::uint8_t> bytes) noexcept;
[[nodiscard]] std::uint32_t adler32(std::span<const std::uint8_t> bytes) noexcept;

// Encodes a valid image (see ImageRgba8::valid); an invalid image or one
// whose size does not fit PNG limits returns an error.
[[nodiscard]] core::Result<std::vector<std::uint8_t>, std::string>
encode_png(const ImageRgba8& image);
[[nodiscard]] core::Result<ImageRgba8, std::string> decode_png(std::span<const std::uint8_t> bytes);

[[nodiscard]] core::Result<std::monostate, std::string> write_png(const std::filesystem::path& path,
                                                                  const ImageRgba8& image);
[[nodiscard]] core::Result<ImageRgba8, std::string> read_png(const std::filesystem::path& path);

} // namespace orrery::render
