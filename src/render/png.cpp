#include "orrery/render/png.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string_view>
#include <system_error>

namespace orrery::render {

namespace {

constexpr std::array<std::uint8_t, 8> kSignature{0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A};
constexpr std::size_t kMaxStoredBlock = 65535;
constexpr std::size_t kMaxIdatChunk = 1U << 20U;
// Largest accepted image: 2^28 pixels (1 GiB of RGBA8). PNG itself allows
// 2^31 - 1 per dimension; this bound keeps sizes far from overflow.
constexpr std::uint64_t kMaxPixels = 1ULL << 28U;
constexpr std::uint32_t kMaxDimension = 0x7FFFFFFFU;

constexpr std::array<std::uint32_t, 256> make_crc_table() {
    std::array<std::uint32_t, 256> table{};
    for (std::uint32_t n = 0; n < 256; ++n) {
        std::uint32_t c = n;
        for (int k = 0; k < 8; ++k) {
            c = (c & 1U) != 0 ? 0xEDB88320U ^ (c >> 1U) : c >> 1U;
        }
        table[n] = c;
    }
    return table;
}

constexpr std::array<std::uint32_t, 256> kCrcTable = make_crc_table();

std::uint32_t crc32_update(std::uint32_t crc, std::span<const std::uint8_t> bytes) {
    for (const std::uint8_t byte : bytes) {
        crc = kCrcTable[(crc ^ byte) & 0xFFU] ^ (crc >> 8U);
    }
    return crc;
}

void put_u32_be(std::vector<std::uint8_t>& out, std::uint32_t value) {
    out.push_back(static_cast<std::uint8_t>(value >> 24U));
    out.push_back(static_cast<std::uint8_t>(value >> 16U));
    out.push_back(static_cast<std::uint8_t>(value >> 8U));
    out.push_back(static_cast<std::uint8_t>(value));
}

void put_u16_le(std::vector<std::uint8_t>& out, std::uint16_t value) {
    out.push_back(static_cast<std::uint8_t>(value));
    out.push_back(static_cast<std::uint8_t>(value >> 8U));
}

std::uint32_t get_u32_be(std::span<const std::uint8_t> bytes, std::size_t at) {
    return (static_cast<std::uint32_t>(bytes[at]) << 24U) |
           (static_cast<std::uint32_t>(bytes[at + 1]) << 16U) |
           (static_cast<std::uint32_t>(bytes[at + 2]) << 8U) |
           static_cast<std::uint32_t>(bytes[at + 3]);
}

std::uint16_t get_u16_le(std::span<const std::uint8_t> bytes, std::size_t at) {
    return static_cast<std::uint16_t>(bytes[at] | (bytes[at + 1] << 8U));
}

void put_chunk(std::vector<std::uint8_t>& out, std::string_view type,
               std::span<const std::uint8_t> data) {
    put_u32_be(out, static_cast<std::uint32_t>(data.size()));
    const std::size_t type_at = out.size();
    out.insert(out.end(), type.begin(), type.end());
    out.insert(out.end(), data.begin(), data.end());
    const std::span<const std::uint8_t> crc_range(out.data() + type_at, 4 + data.size());
    put_u32_be(out, crc32(crc_range));
}

std::uint8_t paeth(std::uint8_t a, std::uint8_t b, std::uint8_t c) {
    const int p = int{a} + int{b} - int{c};
    const int pa = std::abs(p - int{a});
    const int pb = std::abs(p - int{b});
    const int pc = std::abs(p - int{c});
    if (pa <= pb && pa <= pc) {
        return a;
    }
    return pb <= pc ? b : c;
}

using Failure = core::Failure<std::string>;

Failure fail(std::string message) {
    return core::failure(std::string("PNG: ") + std::move(message));
}

} // namespace

std::uint32_t crc32(std::span<const std::uint8_t> bytes) noexcept {
    return crc32_update(0xFFFFFFFFU, bytes) ^ 0xFFFFFFFFU;
}

std::uint32_t adler32(std::span<const std::uint8_t> bytes) noexcept {
    constexpr std::uint32_t kModulus = 65521;
    std::uint32_t a = 1;
    std::uint32_t b = 0;
    for (const std::uint8_t byte : bytes) {
        a = (a + byte) % kModulus;
        b = (b + a) % kModulus;
    }
    return (b << 16U) | a;
}

core::Result<std::vector<std::uint8_t>, std::string> encode_png(const ImageRgba8& image) {
    if (!image.valid()) {
        return fail("image is empty or its pixel buffer does not match width * height * 4");
    }
    if (image.width > kMaxDimension || image.height > kMaxDimension ||
        std::uint64_t{image.width} * image.height > kMaxPixels) {
        return fail("image is too large to encode");
    }

    // Raw scanlines: filter byte 0 (None) followed by the row.
    const std::size_t row_bytes = std::size_t{image.width} * 4U;
    std::vector<std::uint8_t> raw;
    raw.reserve((row_bytes + 1) * image.height);
    for (std::uint32_t y = 0; y < image.height; ++y) {
        raw.push_back(0);
        const auto row = image.pixels.begin() + static_cast<std::ptrdiff_t>(image.offset(0, y));
        raw.insert(raw.end(), row, row + static_cast<std::ptrdiff_t>(row_bytes));
    }

    // zlib stream: CMF 0x78 (deflate, 32 KiB window), FLG 0x01 (no
    // dictionary, check bits so that (CMF * 256 + FLG) % 31 == 0), stored
    // blocks, Adler-32 of the raw data.
    std::vector<std::uint8_t> zlib;
    zlib.reserve(raw.size() + raw.size() / kMaxStoredBlock * 5 + 16);
    zlib.push_back(0x78);
    zlib.push_back(0x01);
    std::size_t at = 0;
    do {
        const std::size_t length = std::min(kMaxStoredBlock, raw.size() - at);
        const bool final_block = at + length == raw.size();
        zlib.push_back(final_block ? 0x01 : 0x00); // BFINAL, BTYPE = 00 (stored)
        put_u16_le(zlib, static_cast<std::uint16_t>(length));
        put_u16_le(zlib, static_cast<std::uint16_t>(~length & 0xFFFFU));
        zlib.insert(zlib.end(), raw.begin() + static_cast<std::ptrdiff_t>(at),
                    raw.begin() + static_cast<std::ptrdiff_t>(at + length));
        at += length;
    } while (at < raw.size());
    put_u32_be(zlib, adler32(raw));

    std::vector<std::uint8_t> png(kSignature.begin(), kSignature.end());
    std::vector<std::uint8_t> header;
    put_u32_be(header, image.width);
    put_u32_be(header, image.height);
    header.insert(header.end(), {8, 6, 0, 0, 0}); // depth 8, RGBA, deflate, filter 0, no interlace
    put_chunk(png, "IHDR", header);
    for (std::size_t offset = 0; offset < zlib.size(); offset += kMaxIdatChunk) {
        const std::size_t length = std::min(kMaxIdatChunk, zlib.size() - offset);
        put_chunk(png, "IDAT", std::span<const std::uint8_t>(zlib.data() + offset, length));
    }
    put_chunk(png, "IEND", {});
    return png;
}

core::Result<ImageRgba8, std::string> decode_png(std::span<const std::uint8_t> bytes) {
    if (bytes.size() < kSignature.size() ||
        !std::equal(kSignature.begin(), kSignature.end(), bytes.begin())) {
        return fail("missing PNG signature");
    }

    ImageRgba8 image;
    bool have_header = false;
    bool have_end = false;
    std::vector<std::uint8_t> zlib;
    std::size_t at = kSignature.size();
    while (at < bytes.size()) {
        if (bytes.size() - at < 12) {
            return fail("truncated chunk at byte " + std::to_string(at));
        }
        const std::uint32_t length = get_u32_be(bytes, at);
        if (length > 0x7FFFFFFFU || bytes.size() - at - 12 < length) {
            return fail("chunk length " + std::to_string(length) + " at byte " +
                        std::to_string(at) + " exceeds the file");
        }
        const std::string type(bytes.begin() + static_cast<std::ptrdiff_t>(at + 4),
                               bytes.begin() + static_cast<std::ptrdiff_t>(at + 8));
        const std::span<const std::uint8_t> data = bytes.subspan(at + 8, length);
        const std::uint32_t stored_crc = get_u32_be(bytes, at + 8 + length);
        if (crc32(bytes.subspan(at + 4, std::size_t{length} + 4)) != stored_crc) {
            return fail("CRC mismatch in chunk '" + type + "'");
        }
        at += std::size_t{length} + 12;

        if (!have_header && type != "IHDR") {
            return fail("first chunk is '" + type + "', expected 'IHDR'");
        }
        if (type == "IHDR") {
            if (have_header || length != 13) {
                return fail("malformed or repeated IHDR");
            }
            have_header = true;
            image.width = get_u32_be(data, 0);
            image.height = get_u32_be(data, 4);
            if (image.width == 0 || image.height == 0 || image.width > kMaxDimension ||
                image.height > kMaxDimension ||
                std::uint64_t{image.width} * image.height > kMaxPixels) {
                return fail("unsupported image size " + std::to_string(image.width) + "x" +
                            std::to_string(image.height));
            }
            if (data[8] != 8 || data[9] != 6 || data[10] != 0 || data[11] != 0 || data[12] != 0) {
                return fail("only 8-bit RGBA, non-interlaced images are supported");
            }
        } else if (type == "IDAT") {
            zlib.insert(zlib.end(), data.begin(), data.end());
        } else if (type == "IEND") {
            have_end = true;
            break;
        } else if (type == "PLTE") {
            continue; // suggested palette for a truecolour image; not needed
        } else if ((type[0] & 0x20) == 0) {
            return fail("unknown critical chunk '" + type + "'");
        }
    }
    if (!have_end) {
        return fail("missing IEND chunk");
    }

    // zlib container with stored deflate blocks only.
    if (zlib.size() < 6) {
        return fail("zlib stream too short");
    }
    if ((zlib[0] & 0x0FU) != 8 || (zlib[0] * 256U + zlib[1]) % 31U != 0 || (zlib[1] & 0x20U) != 0) {
        return fail("invalid zlib header");
    }
    const std::size_t row_bytes = std::size_t{image.width} * 4U;
    const std::size_t expected = (row_bytes + 1) * image.height;
    std::vector<std::uint8_t> raw;
    raw.reserve(expected);
    std::size_t pos = 2;
    bool final_block = false;
    while (!final_block) {
        if (zlib.size() - pos < 5) {
            return fail("truncated deflate block");
        }
        const std::uint8_t block_header = zlib[pos];
        final_block = (block_header & 0x01U) != 0;
        if (((block_header >> 1U) & 0x03U) != 0) {
            return fail("compressed deflate blocks are not supported (this reader handles the "
                        "stored-block files written by encode_png)");
        }
        const std::uint16_t length = get_u16_le(zlib, pos + 1);
        const std::uint16_t complement = get_u16_le(zlib, pos + 3);
        if (static_cast<std::uint16_t>(~complement) != length) {
            return fail("stored block length check failed");
        }
        pos += 5;
        if (zlib.size() - pos < length || raw.size() + length > expected) {
            return fail("stored block exceeds the stream or the image size");
        }
        raw.insert(raw.end(), zlib.begin() + static_cast<std::ptrdiff_t>(pos),
                   zlib.begin() + static_cast<std::ptrdiff_t>(pos + length));
        pos += length;
    }
    if (zlib.size() - pos != 4) {
        return fail("unexpected data after the final deflate block");
    }
    if (adler32(raw) != get_u32_be(zlib, pos)) {
        return fail("Adler-32 mismatch");
    }
    if (raw.size() != expected) {
        return fail("decompressed size " + std::to_string(raw.size()) + " != expected " +
                    std::to_string(expected));
    }

    // Undo the row filters (bytes per pixel = 4).
    image.pixels.resize(image.byte_size());
    for (std::uint32_t y = 0; y < image.height; ++y) {
        const std::uint8_t filter = raw[y * (row_bytes + 1)];
        const std::uint8_t* in = raw.data() + y * (row_bytes + 1) + 1;
        std::uint8_t* out = image.pixels.data() + image.offset(0, y);
        const std::uint8_t* above = y > 0 ? out - row_bytes : nullptr;
        for (std::size_t i = 0; i < row_bytes; ++i) {
            const std::uint8_t a = i >= 4 ? out[i - 4] : 0;
            const std::uint8_t b = above != nullptr ? above[i] : 0;
            const std::uint8_t c = above != nullptr && i >= 4 ? above[i - 4] : 0;
            std::uint8_t predictor = 0;
            switch (filter) {
                case 0:
                    break;
                case 1:
                    predictor = a;
                    break;
                case 2:
                    predictor = b;
                    break;
                case 3:
                    predictor = static_cast<std::uint8_t>((int{a} + int{b}) / 2);
                    break;
                case 4:
                    predictor = paeth(a, b, c);
                    break;
                default:
                    return fail("invalid row filter " + std::to_string(filter) + " in row " +
                                std::to_string(y));
            }
            out[i] = static_cast<std::uint8_t>(in[i] + predictor);
        }
    }
    return image;
}

core::Result<std::monostate, std::string> write_png(const std::filesystem::path& path,
                                                    const ImageRgba8& image) {
    auto encoded = encode_png(image);
    if (!encoded) {
        return core::failure(encoded.error());
    }
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file) {
        return fail("cannot open '" + path.string() + "' for writing");
    }
    const std::vector<std::uint8_t>& bytes = encoded.value();
    const std::string buffer(bytes.begin(), bytes.end());
    file.write(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    if (!file) {
        return fail("write to '" + path.string() + "' failed");
    }
    return std::monostate{};
}

core::Result<ImageRgba8, std::string> read_png(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return fail("cannot open '" + path.string() + "'");
    }
    std::error_code size_error;
    const auto file_size = std::filesystem::file_size(path, size_error);
    if (size_error) {
        return fail("cannot read '" + path.string() + "'");
    }
    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(file_size));
    file.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    bytes.resize(static_cast<std::size_t>(file.gcount()));
    auto decoded = decode_png(bytes);
    if (!decoded) {
        return core::failure("'" + path.string() + "': " + decoded.error());
    }
    return decoded;
}

} // namespace orrery::render
