#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <utility>

#include <glm/geometric.hpp>

#include "gl_test_support.h"
#include "orrery/render/space_renderer.h"
#include "orrery/render/trails.h"

// Rendering of degenerate scene input: coincident bodies and trails whose
// segments have no length in the world or on screen.

using namespace orrery;
using namespace orrery::render;
using orrery::render::test::renderer_or_skip;

namespace {

constexpr double kAu = 149597870700.0;

core::BodyInit planet_at(const glm::dvec3& position_m) {
    return {"Rock",
            core::Planet{},
            core::space::Mass(3.0e-6),
            {core::space::Length(position_m.x / kAu), core::space::Length(position_m.y / kAu),
             core::space::Length(position_m.z / kAu)},
            {}};
}

// One planet of 0.15 m radius at `position_m`: far below a pixel at the
// distances used here, so that only its trail can reach the image.
std::shared_ptr<const core::SimSnapshot> tiny_planet(const glm::dvec3& position_m) {
    core::BodyStore bodies;
    REQUIRE(bodies.add(planet_at(position_m)));
    REQUIRE(bodies.set_radius(0, core::space::Length(1.0e-12)));
    return core::SimSnapshot::capture(core::SimState(core::space::Time(0.0), std::move(bodies)));
}

Camera origin_camera() {
    Camera camera;
    camera.set_mode(FreeMode{});
    camera.update(0.0, CameraInput{}, BodyFrameLookup{});
    return camera;
}

int largest_difference(const ImageRgba8& a, const ImageRgba8& b) {
    REQUIRE(a.pixels.size() == b.pixels.size());
    int worst = 0;
    for (std::size_t i = 0; i < a.pixels.size(); ++i) {
        worst = std::max(worst, std::abs(int{a.pixels[i]} - int{b.pixels[i]}));
    }
    return worst;
}

double luma(const ImageRgba8& image, std::uint32_t x, std::uint32_t y) {
    const std::size_t o = image.offset(x, y);
    return 0.2126 * image.pixels[o] + 0.7152 * image.pixels[o + 1] + 0.0722 * image.pixels[o + 2];
}

} // namespace

TEST_CASE("Stationary and screen-degenerate trails leave the image unchanged",
          "[render][gl][trails]") {
    OffscreenRenderer renderer = renderer_or_skip(256, 256);
    auto space = SpaceRenderer::create();
    REQUIRE(space.has_value());
    const Camera camera = origin_camera();
    const glm::dvec3 head{2.0e5, 1.0e5, -1.0e6};
    const auto at_head = tiny_planet(head);

    RenderSettings settings;
    settings.show_starfield = false;
    settings.scale_mode = ScaleMode::true_scale;
    const auto draw = [&](const core::SimSnapshot& snapshot, const TrailHistory* trails,
                          bool show_trails, SpaceDrawStats* stats) {
        RenderSettings s = settings;
        s.show_trails = show_trails;
        auto image = renderer.render([&](const FrameInfo& frame) {
            const SpaceDrawStats drawn =
                space.value().draw(SpaceView{&snapshot, trails, &camera, s}, frame);
            if (stats != nullptr) {
                *stats = drawn;
            }
        });
        REQUIRE(image.has_value());
        return std::move(image).value();
    };
    const ImageRgba8 without = draw(*at_head, nullptr, false, nullptr);

    TrailSettings trail_settings;
    trail_settings.length = 512;
    trail_settings.min_step_angle = 0.0;
    SpaceDrawStats stats;

    SECTION("empty ring") {
        const TrailHistory history(trail_settings);
        CHECK(largest_difference(draw(*at_head, &history, true, &stats), without) == 0);
        CHECK(stats.trail_segments == 0);
    }
    SECTION("one stationary sample") {
        TrailHistory history(trail_settings);
        history.record(*at_head);
        CHECK(largest_difference(draw(*at_head, &history, true, &stats), without) == 0);
        CHECK(stats.trail_segments == 0);
    }
    SECTION("hundreds of stationary duplicates") {
        TrailHistory history(trail_settings);
        for (int i = 0; i < 512; ++i) {
            history.record(*at_head);
        }
        REQUIRE(history.find(at_head->ids()[0]) != nullptr);
        CHECK(history.find(at_head->ids()[0])->size() == 512);
        CHECK(largest_difference(draw(*at_head, &history, true, &stats), without) == 0);
        CHECK(stats.trail_segments == 0);
    }
    SECTION("a segment along the view axis projects to a point") {
        // The older sample lies on the ray from the eye through the head.
        TrailHistory history(trail_settings);
        history.record(*tiny_planet(head * 1.5));
        CHECK(largest_difference(draw(*at_head, &history, true, &stats), without) == 0);
        CHECK(stats.trail_segments == 1);
    }
    SECTION("a moving one-sample trail is drawn") {
        TrailHistory history(trail_settings);
        history.record(*tiny_planet(head + glm::dvec3(3.0e4, 0.0, 0.0)));
        const ImageRgba8 with = draw(*at_head, &history, true, &stats);
        CHECK(stats.trail_segments == 1);
        CHECK(largest_difference(with, without) > 10);
        // Only around the segment: the far corner stays untouched.
        CHECK(luma(with, 2, 2) == luma(without, 2, 2));
    }
}

TEST_CASE("A planet enclosing a coincident star renders a finite lit disc",
          "[render][gl][lighting]") {
    OffscreenRenderer renderer = renderer_or_skip(320, 240);
    auto space = SpaceRenderer::create();
    REQUIRE(space.has_value());
    for (const double offset_au : {0.0, 1.0e-200}) {
        INFO("planet offset " << offset_au << " AU");
        core::BodyStore bodies;
        REQUIRE(bodies.add({"Sun", core::Star{}, core::space::Mass(1.0), {}, {}}));
        REQUIRE(bodies.add(planet_at(glm::dvec3(offset_au * kAu, 0.0, 0.0))));
        REQUIRE(bodies.set_radius(0, core::space::Length(1.0e-4)));
        REQUIRE(bodies.set_radius(1, core::space::Length(2.0e-4)));
        const auto snapshot =
            core::SimSnapshot::capture(core::SimState(core::space::Time(0.0), std::move(bodies)));
        Camera camera;
        OrbitMode orbit;
        orbit.distance = 0.004 * kAu;
        orbit.yaw = 0.4;
        orbit.pitch = 0.3;
        camera.set_mode(orbit);
        camera.update(0.0, CameraInput{}, BodyFrameLookup{});
        RenderSettings settings;
        settings.show_starfield = false;
        settings.star_glow = false;
        auto image = renderer.render([&](const FrameInfo& frame) {
            (void)space.value().draw(SpaceView{snapshot.get(), nullptr, &camera, settings}, frame);
        });
        REQUIRE(image.has_value());
        const PreparedBodies prepared =
            prepare_bodies(*snapshot, camera, FrameInfo{320, 240}, settings);
        const BodyScreenInfo& planet = prepared.screen[1];
        REQUIRE(planet.visible);
        REQUIRE(planet.radius_px > 10.0);
        // The visible face of the planet is evenly lit.
        const auto cx = static_cast<std::uint32_t>(planet.pixel.x);
        const auto cy = static_cast<std::uint32_t>(planet.pixel.y);
        const double centre = luma(image.value(), cx, cy);
        CHECK(centre > 20.0);
        const auto step = static_cast<std::uint32_t>(planet.radius_px * 0.4);
        for (const auto& [x, y] : {std::pair{cx - step, cy}, std::pair{cx + step, cy},
                                   std::pair{cx, cy - step}, std::pair{cx, cy + step}}) {
            CHECK(luma(image.value(), x, y) > 0.5 * centre);
        }
        CHECK(renderer.context().debug_error_count() == 0);
    }
}
