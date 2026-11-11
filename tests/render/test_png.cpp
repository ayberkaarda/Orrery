#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "orrery/render/png.h"

using namespace orrery::render;

namespace {

// Independent reference implementations (bitwise CRC-32, textbook Adler-32)
// so the test does not reuse the code under test.
std::uint32_t reference_crc32(std::span<const std::uint8_t> bytes) {
    std::uint32_t crc = 0xFFFFFFFFU;
    for (const std::uint8_t byte : bytes) {
        crc ^= byte;
        for (int bit = 0; bit < 8; ++bit) {
            const std::uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1U) ^ (0xEDB88320U & mask);
        }
    }
    return ~crc;
}

std::uint32_t reference_adler32(std::span<const std::uint8_t> bytes) {
    std::uint64_t a = 1;
    std::uint64_t b = 0;
    for (const std::uint8_t byte : bytes) {
        a = (a + byte) % 65521U;
        b = (b + a) % 65521U;
    }
    return static_cast<std::uint32_t>((b << 16U) | a);
}

std::uint32_t be32(const std::vector<std::uint8_t>& bytes, std::size_t at) {
    return (std::uint32_t{bytes[at]} << 24U) | (std::uint32_t{bytes[at + 1]} << 16U) |
           (std::uint32_t{bytes[at + 2]} << 8U) | std::uint32_t{bytes[at + 3]};
}

struct Chunk {
    std::string type;
    std::vector<std::uint8_t> data;
};

ImageRgba8 gradient(std::uint32_t width, std::uint32_t height) {
    ImageRgba8 image = ImageRgba8::filled(width, height, 0, 0, 0);
    for (std::uint32_t y = 0; y < height; ++y) {
        for (std::uint32_t x = 0; x < width; ++x) {
            const std::size_t o = image.offset(x, y);
            image.pixels[o] = static_cast<std::uint8_t>(x * 7U + y);
            image.pixels[o + 1] = static_cast<std::uint8_t>(y * 13U);
            image.pixels[o + 2] = static_cast<std::uint8_t>((x ^ y) * 5U);
            image.pixels[o + 3] = static_cast<std::uint8_t>(255U - x);
        }
    }
    return image;
}

// PNG file with the given IHDR size and zlib stream as one IDAT chunk
// (8-bit RGBA), with checksums from the reference CRC-32.
std::vector<std::uint8_t> png_with_zlib(std::uint32_t width, std::uint32_t height,
                                        const std::vector<std::uint8_t>& zlib) {
    std::vector<std::uint8_t> png{0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    const auto put32 = [&png](std::uint32_t value) {
        for (int shift = 24; shift >= 0; shift -= 8) {
            png.push_back(static_cast<std::uint8_t>(value >> static_cast<unsigned>(shift)));
        }
    };
    const auto chunk = [&](const std::string& type, const std::vector<std::uint8_t>& data) {
        put32(static_cast<std::uint32_t>(data.size()));
        std::vector<std::uint8_t> body(type.begin(), type.end());
        body.insert(body.end(), data.begin(), data.end());
        png.insert(png.end(), body.begin(), body.end());
        put32(reference_crc32(body));
    };
    std::vector<std::uint8_t> ihdr;
    for (const std::uint32_t value : {width, height}) {
        for (int shift = 24; shift >= 0; shift -= 8) {
            ihdr.push_back(static_cast<std::uint8_t>(value >> static_cast<unsigned>(shift)));
        }
    }
    ihdr.insert(ihdr.end(), {8, 6, 0, 0, 0});
    chunk("IHDR", ihdr);
    chunk("IDAT", zlib);
    chunk("IEND", {});
    return png;
}

// Hand-written deflate bits (LSB first), for streams no encoder would emit.
struct BitStream {
    std::vector<std::uint8_t> bytes{0x78, 0x01};
    std::uint32_t pending = 0;
    int count = 0;

    void put(std::uint32_t value, int bits) {
        for (int i = 0; i < bits; ++i) {
            pending |= ((value >> static_cast<unsigned>(i)) & 1U) << static_cast<unsigned>(count);
            if (++count == 8) {
                bytes.push_back(static_cast<std::uint8_t>(pending));
                pending = 0;
                count = 0;
            }
        }
    }
    // Huffman codes are sent most significant bit first.
    void put_code(std::uint32_t code, int bits) {
        for (int i = bits - 1; i >= 0; --i) {
            put((code >> static_cast<unsigned>(i)) & 1U, 1);
        }
    }
    void put_fixed_literal(std::uint32_t literal) { put_code(0x30U + literal, 8); }
    void put_fixed_end() { put_code(0, 7); }
    void align() {
        if (count > 0) {
            put(0, 8 - count);
        }
    }
    void put_adler(std::uint32_t adler) {
        align();
        for (int shift = 24; shift >= 0; shift -= 8) {
            bytes.push_back(static_cast<std::uint8_t>(adler >> static_cast<unsigned>(shift)));
        }
    }
};

// Scanlines with filter byte 0 for a pixel pattern f(x, y, channel).
template <typename Pattern>
std::vector<std::uint8_t> raw_scanlines(std::uint32_t width, std::uint32_t height, Pattern f) {
    std::vector<std::uint8_t> raw;
    for (std::uint32_t y = 0; y < height; ++y) {
        raw.push_back(0);
        for (std::uint32_t x = 0; x < width; ++x) {
            for (std::uint32_t c = 0; c < 4; ++c) {
                raw.push_back(f(x, y, c));
            }
        }
    }
    return raw;
}

std::uint8_t fixed_pattern(std::uint32_t x, std::uint32_t y, std::uint32_t c) {
    return static_cast<std::uint8_t>(((x * x + 3U * y + 17U * c) * 13U) % 251U);
}

std::uint8_t dynamic_pattern(std::uint32_t x, std::uint32_t y, std::uint32_t c) {
    if (c == 3) {
        return 255;
    }
    const std::uint64_t v =
        ((std::uint64_t{x} * 2654435761U + std::uint64_t{y} * 40503U + std::uint64_t{c} * 977U) >>
         7U) &
        7U;
    return static_cast<std::uint8_t>(v * 30U);
}

// Pixels of a pattern (the decoded form of raw_scanlines).
template <typename Pattern>
std::vector<std::uint8_t> pattern_pixels(std::uint32_t width, std::uint32_t height, Pattern f) {
    std::vector<std::uint8_t> pixels;
    for (std::uint32_t y = 0; y < height; ++y) {
        for (std::uint32_t x = 0; x < width; ++x) {
            for (std::uint32_t c = 0; c < 4; ++c) {
                pixels.push_back(f(x, y, c));
            }
        }
    }
    return pixels;
}

// zlib streams of raw_scanlines(6, 4, fixed_pattern) and
// raw_scanlines(16, 16, dynamic_pattern), produced by Python's zlib module
// (an independent encoder): compressobj(9, DEFLATED, 15, 9, Z_FIXED), which
// writes fixed-Huffman blocks, and compress(data, 9), which chose a dynamic
// Huffman block.
const std::vector<std::uint8_t> kFixedZlib{
    0x78, 0x01, 0x63, 0x60, 0xB8, 0xBB, 0x7F, 0x21, 0xEF, 0xAB, 0x33, 0xEB, 0x4C, 0xC4, 0x3E,
    0x5F, 0x2D, 0x0D, 0xB7, 0x94, 0xBE, 0xB0, 0x69, 0x4A, 0x99, 0x97, 0x0E, 0xDF, 0x6B, 0x06,
    0x75, 0xCE, 0x67, 0x27, 0x40, 0x82, 0xD1, 0xB6, 0xF2, 0x8C, 0x73, 0xEA, 0x12, 0x9C, 0xBE,
    0xDF, 0xDC, 0x3D, 0xB7, 0x30, 0xD8, 0x54, 0x9C, 0xC1, 0xCF, 0x40, 0xE8, 0x3D, 0x48, 0xB0,
    0x29, 0xC5, 0x4D, 0xE3, 0xF0, 0xD2, 0xF6, 0x4C, 0x65, 0xD6, 0x47, 0x47, 0x66, 0x54, 0xC5,
    0xD8, 0x31, 0x80, 0xB4, 0x83, 0x04, 0x57, 0x76, 0xE7, 0xFA, 0x03, 0x4D, 0x9C, 0x00, 0x32,
    0x66, 0xFF, 0xC2, 0xE6, 0x54, 0x00, 0x7D, 0xCB, 0x2A, 0xE4};
const std::vector<std::uint8_t> kDynamicZlib{
    0x78, 0xDA, 0x95, 0x92, 0xD1, 0x0D, 0xC4, 0x30, 0x08, 0x43, 0x3D, 0x49, 0x26, 0xC9, 0x24, 0x99,
    0x84, 0x49, 0x98, 0x24, 0x93, 0xB0, 0x10, 0x8D, 0xA5, 0xBB, 0xA6, 0x55, 0xA1, 0xA2, 0xDF, 0xCF,
    0xB2, 0x91, 0x31, 0x60, 0xE6, 0x63, 0x74, 0x9F, 0x73, 0x7A, 0x6F, 0xCD, 0x55, 0xC5, 0x01, 0xB8,
    0x8C, 0xE1, 0x66, 0xD3, 0x47, 0x27, 0x53, 0x6F, 0x64, 0x42, 0x46, 0xFD, 0x62, 0xD4, 0x2F, 0x06,
    0x11, 0x0A, 0x2D, 0x14, 0xFE, 0x4D, 0x28, 0x9C, 0x4A, 0x86, 0x87, 0x09, 0xEA, 0x69, 0xED, 0x34,
    0x11, 0xB2, 0xDF, 0xE5, 0x78, 0x4B, 0xDB, 0xC2, 0x6D, 0xA2, 0x64, 0x0C, 0x5D, 0x97, 0x93, 0x21,
    0x3A, 0x2B, 0x4B, 0xBB, 0xF6, 0x74, 0x9A, 0x44, 0xC2, 0x28, 0x6D, 0xF7, 0x74, 0x2F, 0x1B, 0xD1,
    0x59, 0x59, 0x5A, 0x58, 0xF6, 0x7B, 0x9A, 0x86, 0xAF, 0xED, 0x17, 0x13, 0xE4, 0x1B, 0xA8, 0xBD,
    0x16, 0xA5, 0xB4, 0x64, 0x03, 0xBC, 0x1C, 0xD5, 0xC5, 0x65, 0x65, 0xA3, 0xBA, 0xB8, 0xAC, 0x6C,
    0x7C, 0xDA, 0x40, 0x50, 0x36, 0xAA, 0x8B, 0xCB, 0xCA, 0x46, 0x75, 0x71, 0x8F, 0xAF, 0x64, 0x43,
    0xCA, 0x16, 0x97, 0x95, 0x7D, 0x00, 0xAA, 0x82, 0x3A, 0x5B};

} // namespace

TEST_CASE("Checksums match published check values", "[render][png]") {
    const std::string text = "123456789";
    const std::span<const std::uint8_t> bytes(reinterpret_cast<const std::uint8_t*>(text.data()),
                                              text.size());
    CHECK(crc32(bytes) == 0xCBF43926U);
    CHECK(adler32(bytes) == 0x091E01DEU);
    CHECK(crc32({}) == 0U);
    CHECK(adler32({}) == 1U);
}

TEST_CASE("Written PNG parses back: signature, IHDR, CRCs, zlib and pixels", "[render][png]") {
    // 200 x 90 RGBA: 72090 raw bytes of filtered scanlines.
    const ImageRgba8 image = gradient(200, 90);
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "orrery_test_png_roundtrip.png";
    REQUIRE(write_png(path, image).has_value());

    std::ifstream file(path, std::ios::binary);
    REQUIRE(file);
    std::vector<std::uint8_t> bytes(std::filesystem::file_size(path));
    file.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    bytes.resize(static_cast<std::size_t>(file.gcount()));
    file.close();
    std::filesystem::remove(path);

    const std::array<std::uint8_t, 8> signature{0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    REQUIRE(bytes.size() > signature.size());
    CHECK(std::equal(signature.begin(), signature.end(), bytes.begin()));

    std::vector<Chunk> chunks;
    std::size_t at = signature.size();
    while (at < bytes.size()) {
        REQUIRE(bytes.size() - at >= 12);
        const std::uint32_t length = be32(bytes, at);
        REQUIRE(bytes.size() - at - 12 >= length);
        const std::span<const std::uint8_t> crc_range(bytes.data() + at + 4, length + 4U);
        CHECK(be32(bytes, at + 8 + length) == reference_crc32(crc_range));
        chunks.push_back({std::string(bytes.begin() + static_cast<std::ptrdiff_t>(at + 4),
                                      bytes.begin() + static_cast<std::ptrdiff_t>(at + 8)),
                          std::vector<std::uint8_t>(
                              bytes.begin() + static_cast<std::ptrdiff_t>(at + 8),
                              bytes.begin() + static_cast<std::ptrdiff_t>(at + 8 + length))});
        at += length + 12U;
    }
    REQUIRE(chunks.size() >= 3);
    CHECK(chunks.front().type == "IHDR");
    CHECK(chunks.back().type == "IEND");
    CHECK(chunks.back().data.empty());

    const std::vector<std::uint8_t>& ihdr = chunks.front().data;
    REQUIRE(ihdr.size() == 13);
    CHECK(be32(ihdr, 0) == 200U);
    CHECK(be32(ihdr, 4) == 90U);
    CHECK(ihdr[8] == 8);  // bit depth
    CHECK(ihdr[9] == 6);  // RGBA
    CHECK(ihdr[10] == 0); // deflate
    CHECK(ihdr[11] == 0); // adaptive filtering
    CHECK(ihdr[12] == 0); // no interlace

    std::vector<std::uint8_t> zlib;
    for (const Chunk& chunk : chunks) {
        if (chunk.type == "IDAT") {
            zlib.insert(zlib.end(), chunk.data.begin(), chunk.data.end());
        }
    }
    REQUIRE(zlib.size() > 6);
    CHECK((zlib[0] * 256U + zlib[1]) % 31U == 0U);
    CHECK((zlib[0] & 0x0FU) == 8U);

    CHECK((zlib[1] & 0x20U) == 0U); // no preset dictionary
    CHECK((zlib[0] >> 4U) <= 7U);   // window of at most 32 KiB
    // Compressed: far below the 72090 raw bytes of the filtered scanlines.
    CHECK(zlib.size() < 72090U / 4U);
    // The pixel data is checked by the library reader here and by Python's
    // zlib in the render.png_python_zlib test.
    auto decoded = decode_png(bytes);
    REQUIRE(decoded.has_value());
    CHECK(decoded.value().pixels == image.pixels);
}

TEST_CASE("PNG round trip through the library reader", "[render][png]") {
    const ImageRgba8 image = gradient(17, 5);
    auto encoded = encode_png(image);
    REQUIRE(encoded.has_value());
    auto decoded = decode_png(encoded.value());
    REQUIRE(decoded.has_value());
    CHECK(decoded.value().width == 17U);
    CHECK(decoded.value().height == 5U);
    CHECK(decoded.value().pixels == image.pixels);
}

TEST_CASE("PNG reader undoes all five row filters", "[render][png]") {
    // A 3x2 image encoded with filters Sub (1) and Paeth (4), by hand.
    const std::vector<std::uint8_t> pixels{
        10, 20, 30, 255, 15, 25, 35, 255, 200, 100, 50, 0,  //
        11, 22, 33, 44,  16, 27, 38, 49,  250, 5,   60, 70, //
    };
    std::vector<std::uint8_t> raw;
    // Row 0, Sub: x - a.
    raw.push_back(1);
    for (std::size_t i = 0; i < 12; ++i) {
        raw.push_back(static_cast<std::uint8_t>(pixels[i] - (i >= 4 ? pixels[i - 4] : 0)));
    }
    // Row 1, Paeth.
    raw.push_back(4);
    for (std::size_t i = 0; i < 12; ++i) {
        const int a = i >= 4 ? pixels[12 + i - 4] : 0;
        const int b = pixels[i];
        const int c = i >= 4 ? pixels[i - 4] : 0;
        const int p = a + b - c;
        const int pa = std::abs(p - a);
        const int pb = std::abs(p - b);
        const int pc = std::abs(p - c);
        const int predictor = (pa <= pb && pa <= pc) ? a : (pb <= pc ? b : c);
        raw.push_back(static_cast<std::uint8_t>(pixels[12 + i] - predictor));
    }
    std::vector<std::uint8_t> zlib{0x78, 0x01, 0x01};
    zlib.push_back(static_cast<std::uint8_t>(raw.size()));
    zlib.push_back(0);
    zlib.push_back(static_cast<std::uint8_t>(~raw.size()));
    zlib.push_back(0xFF);
    zlib.insert(zlib.end(), raw.begin(), raw.end());
    const std::uint32_t adler = reference_adler32(raw);
    for (int shift = 24; shift >= 0; shift -= 8) {
        zlib.push_back(static_cast<std::uint8_t>(adler >> static_cast<unsigned>(shift)));
    }

    std::vector<std::uint8_t> png{0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    const auto chunk = [&png](const std::string& type, const std::vector<std::uint8_t>& data) {
        for (int shift = 24; shift >= 0; shift -= 8) {
            png.push_back(static_cast<std::uint8_t>(data.size() >> static_cast<unsigned>(shift)));
        }
        std::vector<std::uint8_t> body(type.begin(), type.end());
        body.insert(body.end(), data.begin(), data.end());
        png.insert(png.end(), body.begin(), body.end());
        const std::uint32_t crc = reference_crc32(body);
        for (int shift = 24; shift >= 0; shift -= 8) {
            png.push_back(static_cast<std::uint8_t>(crc >> static_cast<unsigned>(shift)));
        }
    };
    chunk("IHDR", {0, 0, 0, 3, 0, 0, 0, 2, 8, 6, 0, 0, 0});
    chunk("tEXt", {'a', 0, 'b'}); // ancillary, skipped
    chunk("IDAT", zlib);
    chunk("IEND", {});

    auto decoded = decode_png(png);
    REQUIRE(decoded.has_value());
    CHECK(decoded.value().pixels == pixels);
}

TEST_CASE("PNG reader rejects corrupt input with a clear error", "[render][png]") {
    auto encoded = encode_png(ImageRgba8::filled(4, 4, 1, 2, 3));
    REQUIRE(encoded.has_value());
    const std::vector<std::uint8_t> good = encoded.value();

    SECTION("bad signature") {
        std::vector<std::uint8_t> bad = good;
        REQUIRE(bad.size() > 1);
        bad[1] = 'Q';
        auto result = decode_png(bad);
        REQUIRE_FALSE(result.has_value());
        CHECK(result.error().find("signature") != std::string::npos);
    }
    SECTION("flipped pixel byte fails the CRC") {
        std::vector<std::uint8_t> bad = good;
        bad[bad.size() - 20] ^= 0x40U;
        auto result = decode_png(bad);
        REQUIRE_FALSE(result.has_value());
        CHECK(result.error().find("CRC") != std::string::npos);
    }
    SECTION("truncated file") {
        const std::vector<std::uint8_t> bad(good.begin(), good.begin() + 40);
        CHECK_FALSE(decode_png(bad).has_value());
    }
    SECTION("invalid image cannot be encoded") {
        ImageRgba8 empty;
        CHECK_FALSE(encode_png(empty).has_value());
    }
}

TEST_CASE("PNG round trip of random, flat, gradient, noisy and large images", "[render][png]") {
    std::uint64_t state = 0x9E3779B97F4A7C15ULL;
    const auto next = [&state]() {
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<std::uint8_t>(state >> 56U);
    };
    std::vector<std::pair<std::string, ImageRgba8>> images;
    {
        ImageRgba8 random = ImageRgba8::filled(97, 61, 0, 0, 0);
        for (std::uint8_t& byte : random.pixels) {
            byte = next();
        }
        images.emplace_back("random", std::move(random));
    }
    images.emplace_back("flat", ImageRgba8::filled(300, 200, 12, 34, 56));
    images.emplace_back("gradient", gradient(255, 130));
    {
        // Smooth content plus small noise: what rendered images look like.
        ImageRgba8 noisy = gradient(320, 240);
        for (std::uint8_t& byte : noisy.pixels) {
            byte = static_cast<std::uint8_t>(byte + (next() & 3U));
        }
        images.emplace_back("noisy", std::move(noisy));
    }
    {
        // Large: 2048 x 1100 (9 MB of pixels), stars on a dark sky: long
        // runs and matches across the whole window.
        ImageRgba8 large = ImageRgba8::filled(2048, 1100, 3, 4, 8);
        for (int star = 0; star < 20000; ++star) {
            const std::uint32_t x = ((std::uint32_t{next()} << 8U) | next()) % 2048U;
            const std::uint32_t y = ((std::uint32_t{next()} << 8U) | next()) % 1100U;
            const std::size_t o = large.offset(x, y);
            large.pixels[o] = next();
            large.pixels[o + 1] = next();
            large.pixels[o + 2] = 255;
        }
        images.emplace_back("large", std::move(large));
    }
    images.emplace_back("one pixel", ImageRgba8::filled(1, 1, 200, 100, 50));
    images.emplace_back("one column", gradient(1, 300));

    for (const auto& [name, image] : images) {
        INFO(name << " " << image.width << "x" << image.height);
        auto encoded = encode_png(image);
        REQUIRE(encoded.has_value());
        auto decoded = decode_png(encoded.value());
        REQUIRE(decoded.has_value());
        CHECK(decoded.value().width == image.width);
        CHECK(decoded.value().height == image.height);
        CHECK(decoded.value().pixels == image.pixels);
        // Deterministic: the same image always gives the same bytes.
        CHECK(encode_png(image).value() == encoded.value());
    }
}

TEST_CASE("PNG writer compresses typical images and bounds incompressible ones", "[render][png]") {
    const auto size_of = [](const ImageRgba8& image) {
        auto encoded = encode_png(image);
        REQUIRE(encoded.has_value());
        return encoded.value().size();
    };
    const auto raw_size = [](const ImageRgba8& image) {
        return (std::size_t{image.width} * 4U + 1U) * image.height;
    };
    const ImageRgba8 flat = ImageRgba8::filled(512, 512, 10, 20, 30);
    CHECK(size_of(flat) < raw_size(flat) / 200U);
    const ImageRgba8 smooth = gradient(400, 300);
    CHECK(size_of(smooth) < raw_size(smooth) / 4U);
    // Random bytes cannot be compressed: stored blocks keep the overhead tiny.
    ImageRgba8 random = ImageRgba8::filled(256, 256, 0, 0, 0);
    std::uint32_t state = 1U;
    for (std::uint8_t& byte : random.pixels) {
        state = state * 1103515245U + 12345U;
        byte = static_cast<std::uint8_t>(state >> 23U);
    }
    CHECK(size_of(random) < raw_size(random) + raw_size(random) / 500U + 128U);
}

TEST_CASE("PNG reader inflates fixed and dynamic Huffman blocks from another encoder",
          "[render][png]") {
    REQUIRE(((kFixedZlib[2] >> 1U) & 3U) == 1U);   // first block: fixed Huffman
    REQUIRE(((kDynamicZlib[2] >> 1U) & 3U) == 2U); // first block: dynamic Huffman
    REQUIRE(raw_scanlines(6, 4, fixed_pattern).size() == 100U);
    auto fixed = decode_png(png_with_zlib(6, 4, kFixedZlib));
    REQUIRE(fixed.has_value());
    CHECK(fixed.value().pixels == pattern_pixels(6, 4, fixed_pattern));
    auto dynamic = decode_png(png_with_zlib(16, 16, kDynamicZlib));
    REQUIRE(dynamic.has_value());
    CHECK(dynamic.value().pixels == pattern_pixels(16, 16, dynamic_pattern));
}

TEST_CASE("PNG reader rejects truncated and corrupt deflate streams", "[render][png]") {
    const auto error_of = [](const std::vector<std::uint8_t>& png) {
        auto result = decode_png(png);
        REQUIRE_FALSE(result.has_value());
        return result.error();
    };
    const std::vector<std::uint8_t> raw = raw_scanlines(2, 2, fixed_pattern);
    const std::uint32_t adler = reference_adler32(raw);

    SECTION("every truncation of a valid stream") {
        for (std::size_t keep = 0; keep < kDynamicZlib.size(); ++keep) {
            INFO("first " << keep << " bytes");
            const std::vector<std::uint8_t> cut(
                kDynamicZlib.begin(), kDynamicZlib.begin() + static_cast<std::ptrdiff_t>(keep));
            CHECK_FALSE(decode_png(png_with_zlib(16, 16, cut)).has_value());
        }
    }
    SECTION("no single-bit flip of a valid stream gives wrong pixels") {
        // Each flip is caught by the inflater or the Adler-32 check, except
        // in the padding bits after the last block, which deflate ignores.
        const std::vector<std::uint8_t> expected = pattern_pixels(6, 4, fixed_pattern);
        std::size_t rejected = 0;
        for (std::size_t byte = 2; byte < kFixedZlib.size(); ++byte) {
            for (unsigned bit = 0; bit < 8; ++bit) {
                std::vector<std::uint8_t> bad = kFixedZlib;
                bad[byte] = static_cast<std::uint8_t>(bad[byte] ^ (1U << bit));
                INFO("byte " << byte << " bit " << bit);
                auto result = decode_png(png_with_zlib(6, 4, bad));
                if (result.has_value()) {
                    CHECK(byte == kFixedZlib.size() - 5);
                    CHECK(result.value().pixels == expected);
                } else {
                    ++rejected;
                }
            }
        }
        CHECK(rejected + 8 >= (kFixedZlib.size() - 2) * 8);
    }
    SECTION("reserved block type") {
        BitStream bits;
        bits.put(1, 1);
        bits.put(3, 2);
        bits.put_adler(adler);
        CHECK(error_of(png_with_zlib(2, 2, bits.bytes)).find("block type") != std::string::npos);
    }
    SECTION("distance before the start of the output") {
        BitStream bits;
        bits.put(1, 1);
        bits.put(1, 2);          // fixed Huffman
        bits.put_code(0xC5U, 8); // length 258 (symbol 285)
        bits.put_code(0, 5);     // distance 1, but nothing was written yet
        bits.put_fixed_end();
        bits.put_adler(adler);
        CHECK(error_of(png_with_zlib(2, 2, bits.bytes)).find("distance") != std::string::npos);
    }
    SECTION("invalid literal/length symbol") {
        BitStream bits;
        bits.put(1, 1);
        bits.put(1, 2);
        bits.put_code(0xC6U, 8); // symbol 286 does not exist
        bits.put_fixed_end();
        bits.put_adler(adler);
        CHECK(error_of(png_with_zlib(2, 2, bits.bytes)).find("symbol") != std::string::npos);
    }
    SECTION("dynamic block whose code-length code is empty") {
        BitStream bits;
        bits.put(1, 1);
        bits.put(2, 2);
        bits.put(0, 5); // HLIT 257
        bits.put(0, 5); // HDIST 1
        bits.put(0, 4); // HCLEN 4, all lengths zero
        bits.put(0, 12);
        bits.put_adler(adler);
        CHECK(error_of(png_with_zlib(2, 2, bits.bytes)).find("code") != std::string::npos);
    }
    SECTION("over-subscribed code lengths") {
        BitStream bits;
        bits.put(1, 1);
        bits.put(2, 2);
        bits.put(0, 5);
        bits.put(0, 5);
        bits.put(0, 4);
        for (int i = 0; i < 4; ++i) {
            bits.put(1, 3); // four codes of one bit
        }
        bits.put_adler(adler);
        CHECK(error_of(png_with_zlib(2, 2, bits.bytes)).find("code") != std::string::npos);
    }
    SECTION("Adler-32 mismatch") {
        std::vector<std::uint8_t> bad = kFixedZlib;
        bad.back() = static_cast<std::uint8_t>(bad.back() ^ 1U);
        CHECK(error_of(png_with_zlib(6, 4, bad)).find("Adler") != std::string::npos);
    }
    SECTION("data after the end of the stream") {
        std::vector<std::uint8_t> bad = kFixedZlib;
        bad.push_back(0);
        CHECK_FALSE(decode_png(png_with_zlib(6, 4, bad)).has_value());
    }
    SECTION("preset dictionary") {
        std::vector<std::uint8_t> bad = kFixedZlib;
        bad[1] = 0x20; // FDICT set; 0x7820 is a multiple of 31
        REQUIRE((bad[0] * 256U + bad[1]) % 31U == 0U);
        CHECK(error_of(png_with_zlib(6, 4, bad)).find("zlib") != std::string::npos);
    }
}

TEST_CASE("PNG reader stops decompression bombs at the image size", "[render][png]") {
    // A 16 x 16 image needs 1040 bytes of scanlines. This stream expands to
    // 64 MiB: a literal 0 followed by runs of 258 copies at distance 1.
    BitStream bits;
    bits.put(1, 1);
    bits.put(1, 2);
    bits.put_fixed_literal(0);
    const std::size_t runs = (std::size_t{64} << 20U) / 258U;
    for (std::size_t i = 0; i < runs; ++i) {
        bits.put_code(0xC5U, 8);
        bits.put_code(0, 5);
    }
    bits.put_fixed_end();
    bits.put_adler(1);
    REQUIRE(bits.bytes.size() < 450000U);
    auto bomb = decode_png(png_with_zlib(16, 16, bits.bytes));
    REQUIRE_FALSE(bomb.has_value());
    CHECK(bomb.error().find("image size") != std::string::npos);

    // The largest accepted size (2^28 pixels) with a stream that holds a
    // single byte: rejected without inflating or allocating the full image.
    BitStream tiny;
    tiny.put(1, 1);
    tiny.put(1, 2);
    tiny.put_fixed_literal(0);
    tiny.put_fixed_end();
    tiny.put_adler(reference_adler32(std::vector<std::uint8_t>{0}));
    auto short_stream = decode_png(png_with_zlib(16384, 16384, tiny.bytes));
    REQUIRE_FALSE(short_stream.has_value());
    CHECK(short_stream.error().find("size") != std::string::npos);
    // One row more than the limit is rejected from the header alone.
    CHECK_FALSE(decode_png(png_with_zlib(16384, 16385, tiny.bytes)).has_value());
}

// Writes images of known content for tests/render/check_png_zlib.py, which
// decodes them with Python's zlib (an independent inflater) and compares the
// pixels with the same patterns computed in Python. Hidden: run by that test.
TEST_CASE("Export PNG files for the Python zlib check", "[.png_export]") {
    const std::filesystem::path dir = ORRERY_RENDER_OUTPUT_DIR;
    std::filesystem::create_directories(dir);
    const auto write = [&dir](const std::string& name, std::uint32_t width, std::uint32_t height,
                              auto pattern) {
        ImageRgba8 image = ImageRgba8::filled(width, height, 0, 0, 0);
        image.pixels = pattern_pixels(width, height, pattern);
        REQUIRE(write_png(dir / ("png_check_" + name + ".png"), image).has_value());
    };
    write("fixed_pattern", 67, 45, fixed_pattern);
    write("dynamic_pattern", 300, 200, dynamic_pattern);
    write("gradient", 640, 360, [](std::uint32_t x, std::uint32_t y, std::uint32_t c) {
        return static_cast<std::uint8_t>(c == 3 ? 255U : (x * (c + 1U) + y * 2U) % 256U);
    });
    write("flat", 500, 300, [](std::uint32_t, std::uint32_t, std::uint32_t c) {
        return static_cast<std::uint8_t>(40U * c + 7U);
    });
    write("noise", 211, 157, [](std::uint32_t x, std::uint32_t y, std::uint32_t c) {
        std::uint32_t h = (x * 73856093U) ^ (y * 19349663U) ^ (c * 83492791U);
        h ^= h >> 13U;
        h *= 0x5BD1E995U;
        h ^= h >> 15U;
        return static_cast<std::uint8_t>(h);
    });
}

// Re-encodes every golden reference with the current writer into the render
// output directory, after checking that the result decodes to the same
// pixels. Used to prove that a writer change keeps golden pixels intact
// (compare the files with an independent decoder before replacing them).
TEST_CASE("Re-encode the golden references with the current writer", "[.png_recompress]") {
    const std::filesystem::path output = ORRERY_RENDER_OUTPUT_DIR;
    std::filesystem::create_directories(output);
    std::size_t count = 0;
    for (const auto& entry : std::filesystem::directory_iterator(ORRERY_RENDER_GOLDEN_DIR)) {
        if (entry.path().extension() != ".png") {
            continue;
        }
        INFO(entry.path().string());
        auto original = read_png(entry.path());
        REQUIRE(original.has_value());
        const std::filesystem::path target =
            output / (entry.path().stem().string() + "_recompressed.png");
        REQUIRE(write_png(target, original.value()).has_value());
        auto again = read_png(target);
        REQUIRE(again.has_value());
        CHECK(again.value().width == original.value().width);
        CHECK(again.value().height == original.value().height);
        CHECK(again.value().pixels == original.value().pixels);
        WARN(entry.path().filename().string()
             << ": " << std::filesystem::file_size(entry.path()) << " -> "
             << std::filesystem::file_size(target) << " bytes");
        ++count;
    }
    CHECK(count > 0);
}
