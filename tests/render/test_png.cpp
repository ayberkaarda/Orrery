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
    // 200 x 90 RGBA: 72090 raw bytes, more than one stored deflate block.
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

    // Inflate the stored blocks by hand.
    std::vector<std::uint8_t> raw;
    std::size_t pos = 2;
    int blocks = 0;
    bool last = false;
    while (!last) {
        REQUIRE(zlib.size() - pos >= 5);
        last = (zlib[pos] & 1U) != 0;
        CHECK(((zlib[pos] >> 1U) & 3U) == 0U); // stored
        const std::size_t length = std::size_t{zlib[pos + 1]} | (std::size_t{zlib[pos + 2]} << 8U);
        const std::size_t complement =
            std::size_t{zlib[pos + 3]} | (std::size_t{zlib[pos + 4]} << 8U);
        CHECK((length ^ complement) == 0xFFFFU);
        pos += 5;
        raw.insert(raw.end(), zlib.begin() + static_cast<std::ptrdiff_t>(pos),
                   zlib.begin() + static_cast<std::ptrdiff_t>(pos + length));
        pos += length;
        ++blocks;
    }
    CHECK(blocks == 2);
    REQUIRE(zlib.size() - pos == 4);
    CHECK(be32(zlib, pos) == reference_adler32(raw));

    REQUIRE(raw.size() == 90U * (1U + 200U * 4U));
    for (std::uint32_t y = 0; y < 90; ++y) {
        const std::size_t row = y * (1U + 200U * 4U);
        REQUIRE(raw[row] == 0); // filter None
        for (std::size_t i = 0; i < 200U * 4U; ++i) {
            if (raw[row + 1 + i] != image.pixels[image.offset(0, y) + i]) {
                FAIL("pixel byte mismatch at row " << y << " byte " << i);
            }
        }
    }
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
