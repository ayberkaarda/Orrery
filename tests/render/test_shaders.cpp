#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "orrery/render/embedded_shaders.h"
#include "orrery/render/shader_compose.h"

using namespace orrery::render;

namespace {

std::string read_normalized(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    std::error_code ec;
    const auto size = std::filesystem::file_size(path, ec);
    std::string text;
    if (file && !ec) {
        text.assign(static_cast<std::size_t>(size), '\0');
        file.read(text.data(), static_cast<std::streamsize>(text.size()));
        text.resize(static_cast<std::size_t>(file.gcount()));
    }
    std::string out;
    out.reserve(text.size());
    for (std::size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '\r' && i + 1 < text.size() && text[i + 1] == '\n') {
            continue;
        }
        out += text[i];
    }
    return out;
}

} // namespace

TEST_CASE("Every shader file is embedded byte for byte", "[render][shaders]") {
    const std::filesystem::path dir = ORRERY_RENDER_SHADER_DIR;
    std::set<std::string> files;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (entry.path().extension() == ".glsl") {
            files.insert(entry.path().filename().string());
        }
    }
    REQUIRE(files.size() == shaders::all.size());
    std::set<std::string> embedded;
    for (const shaders::EmbeddedShader* shader : shaders::all) {
        embedded.insert(std::string(shader->name));
        INFO(shader->name);
        CHECK(shader->source == read_normalized(dir / std::string(shader->name)));
    }
    CHECK(embedded == files);
    CHECK(shaders::test_shape_frag.source.find("orrery_log_depth") != std::string_view::npos);
}

TEST_CASE("Stage composition inserts snippets after #version with #line markers",
          "[render][shaders]") {
    const ShaderSource main{"main.frag.glsl", "#version 460 core\nvoid main() {}\n"};
    const std::array<ShaderSource, 2> includes{
        ShaderSource{"a.glsl", "float a() { return 1.0; }"},
        ShaderSource{"b.glsl", "float b() { return 2.0; }\n"},
    };
    auto composed = compose_shader(main, includes);
    REQUIRE(composed.has_value());
    const ComposedShader& parts = composed.value();
    REQUIRE(parts.strings.size() == 4);
    CHECK(parts.strings[0] == "#version 460 core\n");
    CHECK(parts.strings[1] == "#line 1 1\nfloat a() { return 1.0; }\n");
    CHECK(parts.strings[2] == "#line 1 2\nfloat b() { return 2.0; }\n");
    CHECK(parts.strings[3] == "#line 2 0\nvoid main() {}\n");
    CHECK(parts.names == std::vector<std::string>{"main.frag.glsl", "a.glsl", "b.glsl"});

    const ShaderSource no_version{"bad.glsl", "void main() {}\n"};
    auto failed = compose_shader(no_version, {});
    REQUIRE_FALSE(failed.has_value());
    CHECK(failed.error().find("bad.glsl:1") != std::string::npos);
}

TEST_CASE("Compiler logs are rewritten to file:line for common driver formats",
          "[render][shaders]") {
    const std::vector<std::string> names{"main.frag.glsl", "log_depth.glsl"};
    // AMD / Intel style, Mesa style, NVIDIA style, and an out-of-range index.
    const std::string log = "ERROR: 0:12: 'x' : undeclared identifier\n"
                            "1:3(7): error: syntax error\n"
                            "0(44) : error C1008: undefined variable \"y\"\n"
                            "ERROR: 5:2: unknown string\n";
    const std::string annotated = annotate_shader_log(log, names);
    CHECK(annotated.find("ERROR: main.frag.glsl:12: 'x'") != std::string::npos);
    CHECK(annotated.find("log_depth.glsl:3(7): error") != std::string::npos);
    CHECK(annotated.find("main.frag.glsl:44 : error C1008") != std::string::npos);
    CHECK(annotated.find("ERROR: 5:2: unknown string") != std::string::npos);
}
