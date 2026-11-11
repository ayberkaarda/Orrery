#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

// Generated at build time by tests/render/make_embed_fixtures.cmake and the
// production src/render/cmake/embed_shaders.cmake (namespace
// orrery::render::embed_fixture). Compiling this file is itself part of the
// test: every raw string literal must stay within the MSVC limit of 16380
// bytes per literal.
#include "orrery_test/embedded_fixture_shaders.h"

namespace fixture = orrery::render::embed_fixture;

namespace {

constexpr std::size_t kPieceLimit = 8000; // embed_shaders.cmake piece_limit
constexpr std::string_view kOpen = "R\"orrery_glsl(";
constexpr std::string_view kClose = ")orrery_glsl\"";

// File bytes with CR LF pairs read as LF: CMake writes the fixtures and the
// header with the platform line ending, the embedding script normalises
// sources to LF, and a compiler reads a CR LF line ending in a raw string
// literal as LF.
std::string read_bytes(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    std::error_code ec;
    const auto size = std::filesystem::file_size(path, ec);
    std::string text;
    if (file && !ec) {
        text.assign(static_cast<std::size_t>(size), '\0');
        file.read(text.data(), static_cast<std::streamsize>(text.size()));
        text.resize(static_cast<std::size_t>(file.gcount()));
    }
    std::string normalized;
    normalized.reserve(text.size());
    for (std::size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '\r' && i + 1 < text.size() && text[i + 1] == '\n') {
            continue;
        }
        normalized += text[i];
    }
    return normalized;
}

// Contents of every raw string literal in a generated header, in order.
std::vector<std::string> raw_pieces(const std::string& header) {
    std::vector<std::string> pieces;
    std::size_t at = 0;
    while ((at = header.find(kOpen, at)) != std::string::npos) {
        const std::size_t begin = at + kOpen.size();
        const std::size_t end = header.find(kClose, begin);
        if (end == std::string::npos) {
            break;
        }
        pieces.push_back(header.substr(begin, end - begin));
        at = end + kClose.size();
    }
    return pieces;
}

bool is_utf8_continuation(char byte) {
    return (static_cast<unsigned char>(byte) & 0xC0U) == 0x80U;
}

std::size_t longest_line(std::string_view text) {
    std::size_t longest = 0;
    std::size_t start = 0;
    while (start < text.size()) {
        std::size_t end = text.find('\n', start);
        if (end == std::string_view::npos) {
            end = text.size();
        }
        longest = std::max(longest, end - start);
        start = end + 1;
    }
    return longest;
}

void check_pieces(const std::filesystem::path& header_path) {
    INFO(header_path.string());
    const std::string header = read_bytes(header_path);
    REQUIRE_FALSE(header.empty());
    const std::vector<std::string> pieces = raw_pieces(header);
    REQUIRE_FALSE(pieces.empty());
    for (const std::string& piece : pieces) {
        CHECK(piece.size() <= kPieceLimit);
        // No multi-byte UTF-8 character is split across two literals.
        if (!piece.empty()) {
            CHECK_FALSE(is_utf8_continuation(piece.front()));
        }
    }
}

} // namespace

TEST_CASE("Overlong shader lines are embedded byte for byte in bounded pieces",
          "[render][shaders]") {
    const std::filesystem::path dir = ORRERY_EMBED_FIXTURE_DIR;
    REQUIRE(fixture::all.size() == 3);
    for (const auto* shader : fixture::all) {
        INFO(shader->name);
        const std::string expected = read_bytes(dir / std::string(shader->name));
        REQUIRE_FALSE(expected.empty());
        CHECK(shader->source.size() == expected.size());
        CHECK(shader->source == expected);
    }
    CHECK(longest_line(fixture::long_line_frag.source) == 20000);
    CHECK(longest_line(fixture::utf8_line_frag.source) == 30003);

    // The header holds bounded pieces whose concatenation is the file.
    check_pieces(ORRERY_EMBED_FIXTURE_HEADER);
    const std::vector<std::string> pieces = raw_pieces(read_bytes(ORRERY_EMBED_FIXTURE_HEADER));
    std::string joined;
    for (const std::string& piece : pieces) {
        joined += piece;
    }
    std::string files;
    for (const auto* shader : fixture::all) {
        files += read_bytes(dir / std::string(shader->name));
    }
    CHECK(joined == files);
}

TEST_CASE("Production shaders are embedded in bounded pieces", "[render][shaders]") {
    check_pieces(ORRERY_RENDER_SHADER_HEADER);
}
