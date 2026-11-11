#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include "gl_test_support.h"
#include "orrery/render/gl_resources.h"
#include "orrery/render/log.h"
#include "orrery/render/tone_map.h"

using namespace orrery::render;
using orrery::render::test::renderer_or_skip;

TEST_CASE("OpenGL 4.6 core context reports version and renderer", "[render][gl]") {
    OffscreenRenderer renderer = renderer_or_skip(32, 32);
    const GlInfo& info = renderer.context().info();
    std::cout << "GL_VERSION: " << info.version << " | GL_RENDERER: " << info.renderer
              << " | GL_VENDOR: " << info.vendor << " | GLSL: " << info.glsl_version << '\n';
    CHECK(info.major == 4);
    CHECK(info.minor >= 6);
    CHECK_FALSE(info.renderer.empty());
}

TEST_CASE("Shader compile errors name the file and line", "[render][gl]") {
    OffscreenRenderer renderer = renderer_or_skip(16, 16);
    // Driver compile diagnostics may also arrive through KHR_debug; keep the
    // test output clean.
    const LogSink previous = set_log_sink([](LogLevel, std::string_view) {});

    const std::string main_code = "#version 460 core\n"
                                  "out vec4 o_color;\n"
                                  "void main() {\n"
                                  "    o_color = vec4(undefined_name);\n"
                                  "}\n";
    const StageSource broken_main{ShaderStage::fragment, {"broken.frag.glsl", main_code}, {}};
    auto main_failure = Shader::compile(broken_main);
    REQUIRE_FALSE(main_failure.has_value());
    INFO(main_failure.error());
    CHECK(main_failure.error().find("fragment shader 'broken.frag.glsl'") != std::string::npos);
    CHECK(main_failure.error().find("broken.frag.glsl:4") != std::string::npos);

    const std::string snippet = "float helper() {\n"
                                "    return missing_value;\n"
                                "}\n";
    const std::string good_main = "#version 460 core\n"
                                  "out vec4 o_color;\n"
                                  "void main() { o_color = vec4(helper()); }\n";
    const StageSource broken_include{
        ShaderStage::fragment, {"good.frag.glsl", good_main}, {{"snippet.glsl", snippet}}};
    auto include_failure = Shader::compile(broken_include);
    REQUIRE_FALSE(include_failure.has_value());
    INFO(include_failure.error());
    CHECK(include_failure.error().find("snippet.glsl:2") != std::string::npos);

    set_log_sink(previous);
}

TEST_CASE("Incomplete framebuffers are reported by status name", "[render][gl]") {
    OffscreenRenderer renderer = renderer_or_skip(16, 16);
    const Framebuffer empty = Framebuffer::create();
    auto status = empty.check();
    REQUIRE_FALSE(status.has_value());
    CHECK(status.error().find("GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT") != std::string::npos);

    const auto color = Texture::create_2d(8, 8, TextureFormat::rgba16f);
    REQUIRE(color.has_value());
    const Framebuffer complete = Framebuffer::create();
    complete.attach_color(0, color.value());
    CHECK(complete.check().has_value());
}

TEST_CASE("GPU HDR resolve matches the CPU ACES + sRGB reference", "[render][gl]") {
    HdrSettings settings;
    settings.samples = 1;
    OffscreenRenderer renderer = renderer_or_skip(24, 16, settings);

    struct Case {
        glm::dvec3 color;
        double exposure;
    };
    const std::vector<Case> cases{
        {{0.0, 0.0, 0.0}, 1.0},  {{0.001, 0.004, 0.02}, 1.0}, {{0.18, 0.18, 0.18}, 1.0},
        {{0.5, 1.0, 2.0}, 1.0},  {{4.0, 16.0, 100.0}, 1.0},   {{0.3, 0.05, 0.9}, 2.5},
        {{1.0, 1.0, 1.0}, 0.25},
    };
    for (const Case& c : cases) {
        renderer.pipeline().set_clear_color(c.color);
        renderer.pipeline().set_exposure(c.exposure);
        auto image = renderer.render({});
        REQUIRE(image.has_value());
        const Rgb8 expected = resolve_hdr_pixel(c.color, c.exposure);
        const ImageRgba8& pixels = image.value();
        int worst = 0;
        for (std::size_t i = 0; i < pixels.pixels.size(); i += 4) {
            worst = std::max({worst, std::abs(pixels.pixels[i] - expected.r),
                              std::abs(pixels.pixels[i + 1] - expected.g),
                              std::abs(pixels.pixels[i + 2] - expected.b)});
            CHECK(pixels.pixels[i + 3] == 255);
        }
        INFO("colour " << c.color.x << ", " << c.color.y << ", " << c.color.z << " exposure "
                       << c.exposure << " expected " << int{expected.r} << ", " << int{expected.g}
                       << ", " << int{expected.b} << " got " << int{pixels.pixels[0]} << ", "
                       << int{pixels.pixels[1]} << ", " << int{pixels.pixels[2]});
        // RGBA16F storage, GPU float math and the hardware sRGB encode each
        // round a little; one 8-bit level is the tolerance.
        CHECK(worst <= 1);
    }
}

TEST_CASE("Renderer setup failures after a working context fail instead of skipping",
          "[render][gl]") {
    // Skips only when no OpenGL 4.6 context exists at all.
    {
        const OffscreenRenderer probe = renderer_or_skip(16, 16);
    }
    const LogSink previous = set_log_sink([](LogLevel, std::string_view) {});

    const std::string broken_tone_map = "#version 460 core\n"
                                        "out vec4 o_color;\n"
                                        "void main() {\n"
                                        "    o_color = vec4(undefined_name);\n"
                                        "}\n";
    HdrPipelineOverrides tone_map;
    tone_map.tone_map_fragment = ShaderSource{"tonemap.frag.glsl", broken_tone_map};
    const test::RendererAttempt compile_failure = test::try_create_renderer(16, 16, {}, tone_map);
    INFO(compile_failure.message);
    CHECK(compile_failure.outcome == test::RendererOutcome::setup_failed);
    CHECK_FALSE(compile_failure.renderer.has_value());
    CHECK(compile_failure.message.find("tonemap.frag.glsl:4") != std::string::npos);

    // A depth format cannot be a colour attachment: the output framebuffer
    // is incomplete.
    HdrPipelineOverrides framebuffer;
    framebuffer.output_format = TextureFormat::depth32f;
    const test::RendererAttempt framebuffer_failure =
        test::try_create_renderer(16, 16, {}, framebuffer);
    INFO(framebuffer_failure.message);
    CHECK(framebuffer_failure.outcome == test::RendererOutcome::setup_failed);
    CHECK_FALSE(framebuffer_failure.renderer.has_value());
    CHECK(framebuffer_failure.message.find("incomplete") != std::string::npos);

    set_log_sink(previous);
}

TEST_CASE("A context that cannot be created is classified as unavailable", "[render]") {
    // A zero-sized window is rejected by the window system layer before any
    // OpenGL work, exactly like a machine without a usable driver.
    ContextOptions options = OffscreenRenderer::context_options();
    options.width = 0;
    const test::RendererAttempt attempt = test::try_create_renderer(16, 16, {}, {}, options);
    INFO(attempt.message);
    CHECK(attempt.outcome == test::RendererOutcome::context_unavailable);
    CHECK_FALSE(attempt.renderer.has_value());
}

TEST_CASE("Empty and invalid GL resource sizes are rejected without GL errors", "[render][gl]") {
    OffscreenRenderer renderer = renderer_or_skip(16, 16);
    const LogSink previous = set_log_sink([](LogLevel, std::string_view) {});
    renderer.context().reset_debug_error_count();

    const std::size_t live_before = detail::gl_handle_stats().live;
    const auto empty = Buffer::create({});
    REQUIRE_FALSE(empty.has_value());
    CHECK(empty.error().find("size 0") != std::string::npos);
    CHECK_FALSE(Buffer::create_uninitialized(0).has_value());
    for (const auto& [width, height, samples] :
         {std::array<int, 3>{0, 16, 1}, std::array<int, 3>{16, -3, 1},
          std::array<int, 3>{1 << 30, 16, 1}, std::array<int, 3>{16, 16, 0},
          std::array<int, 3>{16, 16, 4096}}) {
        const auto texture = Texture::create_2d(width, height, TextureFormat::rgba16f, samples);
        INFO(width << "x" << height << " samples " << samples);
        REQUIRE_FALSE(texture.has_value());
        CHECK(texture.error().find("texture:") != std::string::npos);
    }
    // Rejected before any GL call: no error, and no object name was kept.
    CHECK(renderer.context().debug_error_count() == 0);
    CHECK(detail::gl_handle_stats().live == live_before);

    // Valid sizes still work and report what was allocated.
    const std::array<std::byte, 12> bytes{};
    const auto buffer = Buffer::create(bytes);
    REQUIRE(buffer.has_value());
    CHECK(buffer.value().size() == bytes.size());
    const auto texture = Texture::create_2d(5, 3, TextureFormat::depth32f, 4);
    REQUIRE(texture.has_value());
    CHECK(texture.value().width() == 5);
    CHECK(texture.value().samples() == 4);
    CHECK(renderer.context().debug_error_count() == 0);

    set_log_sink(previous);
}

TEST_CASE("A failed GL allocation returns an error and releases the object", "[render][gl]") {
    OffscreenRenderer renderer = renderer_or_skip(16, 16);
    const LogSink previous = set_log_sink([](LogLevel, std::string_view) {});
    const std::size_t live_before = detail::gl_handle_stats().live;
    {
        // Every storage allocation in this scope fails after the GL object
        // was created, as with GL_OUT_OF_MEMORY.
        const detail::SimulatedAllocationFailure out_of_memory;
        const std::array<std::byte, 64> bytes{};
        const auto buffer = Buffer::create(bytes);
        REQUIRE_FALSE(buffer.has_value());
        CHECK(buffer.error().find("allocating 64 bytes failed") != std::string::npos);
        const auto storage = Buffer::create_uninitialized(1024, true);
        REQUIRE_FALSE(storage.has_value());
        const auto texture = Texture::create_2d(32, 32, TextureFormat::rgba16f, 4);
        REQUIRE_FALSE(texture.has_value());
        CHECK(texture.error().find("allocating 32x32 with 4 sample(s) failed") !=
              std::string::npos);
        // Pipeline setup in a working context fails (and fails the test,
        // see renderer_or_skip) instead of looking like a missing context.
        const test::RendererAttempt attempt = test::try_create_renderer(16, 16);
        CHECK(attempt.outcome == test::RendererOutcome::setup_failed);
        CHECK(attempt.message.find("simulated allocation failure") != std::string::npos);
        renderer.context().make_current();
        // Every name created for the attempts was deleted again: no usable
        // object of the requested size exists.
        CHECK(detail::gl_handle_stats().live == live_before);
    }
    const auto recovered = Buffer::create_uninitialized(1024, true);
    REQUIRE(recovered.has_value());
    CHECK(recovered.value().size() == 1024);
    // The context still works.
    renderer.pipeline().set_clear_color(glm::dvec3(0.18));
    auto image = renderer.render({});
    REQUIRE(image.has_value());
    CHECK(image.value().valid());
    set_log_sink(previous);
}
