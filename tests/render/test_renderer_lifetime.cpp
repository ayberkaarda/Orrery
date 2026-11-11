#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <optional>
#include <string_view>
#include <utility>

#include "gl_test_support.h"
#include "orrery/render/gl_resources.h"
#include "orrery/render/log.h"
#include "orrery/render/tone_map.h"

// OffscreenRenderer lifetime: every renderer owns its own (unshared) context,
// so its GL objects must be deleted while that context is current and never
// in another renderer's context, whatever context is current when the
// renderer is destroyed, moved or nested inside another renderer's frame.

using namespace orrery::render;
using orrery::render::detail::gl_handle_stats;
using orrery::render::test::renderer_or_skip;

namespace {

const glm::dvec3 kColourA{0.9, 0.2, 0.05};
const glm::dvec3 kColourB{0.05, 0.4, 1.6};
const glm::dvec3 kColourInner{0.3, 1.2, 0.3};

// Largest channel difference between every pixel and the CPU reference.
int worst_difference(const ImageRgba8& image, const glm::dvec3& colour, double exposure) {
    const Rgb8 expected = resolve_hdr_pixel(colour, exposure);
    int worst = 0;
    for (std::size_t i = 0; i + 3 < image.pixels.size(); i += 4) {
        worst = std::max({worst, std::abs(image.pixels[i] - expected.r),
                          std::abs(image.pixels[i + 1] - expected.g),
                          std::abs(image.pixels[i + 2] - expected.b),
                          std::abs(image.pixels[i + 3] - 255)});
    }
    return worst;
}

// Renders a frame cleared to `colour` and checks every pixel.
void expect_frame(OffscreenRenderer& renderer, const glm::dvec3& colour) {
    renderer.pipeline().set_clear_color(colour);
    auto image = renderer.render({});
    REQUIRE(image.has_value());
    REQUIRE(image.value().valid());
    CHECK(worst_difference(image.value(), colour, renderer.pipeline().settings().exposure) <= 1);
}

} // namespace

TEST_CASE("Destroying a renderer while another renderer is current keeps the other intact",
          "[render][gl][lifetime]") {
    const std::size_t skipped_before = gl_handle_stats().skipped_deletes;
    std::optional<OffscreenRenderer> a(renderer_or_skip(24, 16));
    std::optional<OffscreenRenderer> b(renderer_or_skip(24, 16));
    // Same creation sequence in two unshared contexts: the object names
    // coincide, so a delete issued in the wrong context hits the other
    // renderer's objects.
    INFO("A tone map target " << a->pipeline().hdr_texture().id() << ", B "
                              << b->pipeline().hdr_texture().id());
    expect_frame(*a, kColourA);
    expect_frame(*b, kColourB); // B is current now
    a.reset();
    CHECK(b->context().is_current()); // B stays current
    expect_frame(*b, kColourB);

    // The other order: destroy the renderer that is not current.
    std::optional<OffscreenRenderer> c(renderer_or_skip(24, 16));
    expect_frame(*c, kColourA);
    expect_frame(*b, kColourB); // B current, C not
    c.reset();
    CHECK(b->context().is_current());
    expect_frame(*b, kColourB);

    // Destroying the current renderer as well.
    b.reset();
    // Every GL object was deleted in the context that created it.
    CHECK(gl_handle_stats().skipped_deletes == skipped_before);
}

TEST_CASE("Move assignment releases the old renderer in its own context",
          "[render][gl][lifetime]") {
    const std::size_t skipped_before = gl_handle_stats().skipped_deletes;
    SECTION("destination current") {
        OffscreenRenderer a = renderer_or_skip(24, 16);
        OffscreenRenderer b = renderer_or_skip(24, 16);
        expect_frame(b, kColourB);
        expect_frame(a, kColourA); // A is current
        a = std::move(b);
        expect_frame(a, kColourB);
        CHECK(a.context().is_current());
    }
    SECTION("source current") {
        OffscreenRenderer a = renderer_or_skip(24, 16);
        OffscreenRenderer b = renderer_or_skip(24, 16);
        expect_frame(a, kColourA);
        expect_frame(b, kColourB); // B is current
        a = std::move(b);
        CHECK(a.context().is_current()); // the moved context stays current
        expect_frame(a, kColourB);
    }
    SECTION("move construction keeps the frame") {
        OffscreenRenderer a = renderer_or_skip(24, 16);
        OffscreenRenderer moved(std::move(a));
        expect_frame(moved, kColourA);
    }
    CHECK(gl_handle_stats().skipped_deletes == skipped_before);
}

TEST_CASE("A one-shot snapshot nested in a frame leaves the outer renderer working",
          "[render][gl][lifetime]") {
    const std::size_t skipped_before = gl_handle_stats().skipped_deletes;
    OffscreenRenderer outer = renderer_or_skip(24, 16);
    outer.pipeline().set_clear_color(kColourA);
    std::optional<ImageRgba8> inner;
    auto image = outer.render([&](const FrameInfo&) {
        HdrSettings settings;
        settings.clear_color = kColourInner;
        auto nested = render_snapshot(12, 12, {}, settings);
        if (nested) {
            inner = std::move(nested).value();
        }
        // The snapshot made the outer frame context current again.
        CHECK(outer.context().is_current());
    });
    REQUIRE(inner.has_value());
    CHECK(worst_difference(*inner, kColourInner, 1.0) <= 1);
    REQUIRE(image.has_value());
    CHECK(worst_difference(image.value(), kColourA, 1.0) <= 1);

    // Outside a frame: a snapshot taken while the outer renderer is current.
    HdrSettings settings;
    settings.clear_color = kColourInner;
    auto standalone = render_snapshot(12, 12, {}, settings);
    REQUIRE(standalone.has_value());
    CHECK(worst_difference(standalone.value(), kColourInner, 1.0) <= 1);
    CHECK(outer.context().is_current());
    // Objects of the outer context can be created right away.
    const auto texture = Texture::create_2d(4, 4, TextureFormat::rgba8);
    REQUIRE(texture.has_value());
    expect_frame(outer, kColourB);
    CHECK(gl_handle_stats().skipped_deletes == skipped_before);
}

TEST_CASE("GL objects are never deleted in a foreign context", "[render][gl][lifetime]") {
    const LogSink previous = set_log_sink([](LogLevel, std::string_view) {});
    OffscreenRenderer a = renderer_or_skip(16, 16);
    // A texture created in the context of A, destroyed while B is current:
    // deleting its name in B would delete an object of B.
    std::optional<Texture> texture_of_a;
    {
        auto created = Texture::create_2d(4, 4, TextureFormat::rgba8);
        REQUIRE(created.has_value());
        texture_of_a = std::move(created).value();
    }
    OffscreenRenderer b = renderer_or_skip(16, 16); // B is current
    const std::size_t skipped_before = gl_handle_stats().skipped_deletes;
    texture_of_a.reset();
    CHECK(gl_handle_stats().skipped_deletes == skipped_before + 1);
    expect_frame(b, kColourB);
    expect_frame(a, kColourA);
    set_log_sink(previous);
}
