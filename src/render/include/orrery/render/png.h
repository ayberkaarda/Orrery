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
// The writer produces 8-bit RGBA, non-interlaced PNG files whose zlib stream
// uses only stored (uncompressed) deflate blocks, with filter type 0 on every
// row. Files are about 4 * width * height bytes; that is the cost of not
// depending on a compression library.
//
// The reader accepts 8-bit RGBA, non-interlaced PNG files with stored deflate
// blocks and any of the five row filters, i.e. every file the writer produces
// and the same subset from other tools. It verifies the signature, every
// chunk CRC-32, the zlib header and the Adler-32 checksum, and rejects
// compressed deflate blocks with an explicit error.
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
