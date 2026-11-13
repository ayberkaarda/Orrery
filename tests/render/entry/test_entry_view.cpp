#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>

#include "../gl_test_support.h"
#include "orrery/render/entry_geometry.h"
#include "orrery/render/entry_view.h"

// Phase 9b: the entry view (docs/entry-workspace.md, "View"). The geometry
// (view frame, placement, framing, decimation, colour positions) is plain
// double math tested without a GPU; the GL test draws a track and checks the
// statistics and the rejection of bad input.

using namespace orrery;
using namespace orrery::render;
using Catch::Approx;

namespace {

// An eastward track along the equator descending from 120 km to the ground.
std::vector<EntryTrackPoint> equatorial_track(std::size_t n) {
    std::vector<EntryTrackPoint> track;
    for (std::size_t i = 0; i < n; ++i) {
        const double f = static_cast<double>(i) / static_cast<double>(n - 1);
        track.push_back({0.0, 10.0 + 20.0 * f, 120'000.0 * (1.0 - f), 8.0 * f * (1.0 - f)});
    }
    return track;
}

// A copy of 	rack whose point index has no latitude.
std::vector<EntryTrackPoint> with_nan_latitude(std::vector<EntryTrackPoint> track,
                                               std::size_t index) {
    if (index < track.size()) {
        track[index].latitude_deg = std::numeric_limits<double>::quiet_NaN();
    }
    return track;
}

} // namespace

TEST_CASE("The entry view frame puts the track overhead and its flight along +x",
          "[render][entry]") {
    const EntryGlobe globe;
    const std::vector<EntryTrackPoint> track = equatorial_track(50);
    const EntryFrame frame = make_entry_frame(track, globe);
    // The middle of the track (longitude 20) is on +z.
    const glm::dvec3 middle = frame.to_view * planet_direction(0.0, 20.0);
    CHECK(middle.z == Approx(1.0).margin(1e-9));
    // Eastward flight runs along +x of the view frame.
    const glm::dvec3 east =
        frame.to_view * glm::dvec3(-std::sin(0.349066), std::cos(0.349066), 0.0);
    CHECK(east.x == Approx(1.0).margin(1e-6));
    // A point keeps its distance from the centre (exaggerated altitude).
    const glm::dvec3 high = view_position(frame, globe, 0.0, 20.0, 100'000.0);
    CHECK(glm::length(high) == Approx(globe.radius_m + 100'000.0 * globe.altitude_scale));
    // The bounds hold the track and its ground track.
    // (the top point is 10 degrees from the middle).
    CHECK(frame.bounds.max.z >=
          (globe.radius_m + 120'000.0 * globe.altitude_scale) * std::cos(0.174533) - 1.0);
    CHECK(frame.bounds.min.z < globe.radius_m);

    // Non-finite points are ignored; an empty track still frames a patch.
    const EntryFrame tolerant = make_entry_frame(with_nan_latitude(track, 3), globe);
    CHECK(std::isfinite(tolerant.bounds.min.x));
    const EntryFrame empty = make_entry_frame({}, globe);
    CHECK(empty.bounds.max.z > empty.bounds.min.z);
}

TEST_CASE("The entry framing sees the whole track", "[render][entry]") {
    const EntryGlobe globe;
    const EntryFrame frame = make_entry_frame(equatorial_track(80), globe);
    CameraLens lens;
    lens.vertical_fov = 0.56;
    const TurntableMode mode = frame_entry(frame, 16.0 / 9.0, lens);
    CHECK(mode.distance > 0.0);
    CHECK(std::isfinite(mode.focus.x));
    const glm::dvec3 size =
        glm::dvec3(frame.bounds.max.x - frame.bounds.min.x, frame.bounds.max.y - frame.bounds.min.y,
                   frame.bounds.max.z - frame.bounds.min.z);
    CHECK(mode.distance > 0.5 * glm::length(size));
}

TEST_CASE("Entry tracks decimate to a bound keeping the ends and the peak", "[render][entry]") {
    const std::vector<EntryTrackPoint> track = equatorial_track(1001);
    for (const std::size_t bound : {std::size_t{3}, std::size_t{10}, std::size_t{100}}) {
        const std::vector<EntryTrackPoint> out = decimate_track(track, bound);
        INFO("bound " << bound);
        CHECK(out.size() <= bound);
        CHECK(out.front().longitude_deg == track.front().longitude_deg);
        CHECK(out.back().longitude_deg == track.back().longitude_deg);
        double peak = 0.0;
        for (const EntryTrackPoint& p : out) {
            peak = std::max(peak, p.value);
        }
        CHECK(peak == 2.0); // 8 f (1 - f) at f = 0.5
    }
    CHECK(decimate_track(track, 5000).size() == track.size());
}

TEST_CASE("Entry colour positions use the upper part of the sequential map", "[render][entry]") {
    const ScalarRange range{0.0, 10.0};
    CHECK(entry_map_position(range, 0.0) == Approx(kEntryMapStart));
    CHECK(entry_map_position(range, 10.0) == Approx(1.0));
    CHECK(entry_map_position(range, 50.0) == Approx(1.0));
    CHECK(entry_map_position(range, -5.0) == Approx(kEntryMapStart));
}

TEST_CASE("The entry view draws a track and rejects bad input", "[render][entry][gl]") {
    OffscreenRenderer renderer = test::renderer_or_skip(480, 270, entry_hdr_settings());
    auto created = EntryView::create();
    INFO((created ? std::string() : created.error()));
    REQUIRE(created.has_value());
    EntryView view = std::move(created).value();
    const EntryGlobe globe;
    REQUIRE(view.set_track(equatorial_track(200), globe).has_value());
    CHECK(view.track_size() == 200);

    // Bad points and globes are refused; the previous track stays.
    std::vector<EntryTrackPoint> bad = equatorial_track(10);
    bad[4].value = std::numeric_limits<double>::infinity();
    const auto refused = view.set_track(bad, globe);
    REQUIRE_FALSE(refused.has_value());
    CHECK(refused.error().find("point 4") != std::string::npos);
    EntryGlobe flat = globe;
    flat.altitude_scale = 0.5;
    CHECK_FALSE(view.set_track(equatorial_track(10), flat).has_value());
    CHECK(view.track_size() == 200);

    Camera camera;
    CameraLens lens;
    lens.vertical_fov = 0.56;
    lens.near_plane = 100.0;
    lens.log_depth_far = 1.0e10;
    REQUIRE(camera.set_lens(lens).has_value());
    camera.set_mode(frame_entry(view.frame(), 480.0 / 270.0, lens));
    (void)camera.update(0.0, CameraInput{}, BodyFrameLookup{});
    EntryViewOptions options;
    options.range = {0.0, 2.0};
    options.markers.push_back({0.0, 20.0, 60'000.0, glm::dvec3(1.0), 6.0});
    std::optional<EntryDrawStats> stats;
    auto image = renderer.render([&](const FrameInfo& frame) {
        auto drawn = view.draw(camera, frame, options);
        INFO((drawn ? std::string() : drawn.error()));
        REQUIRE(drawn.has_value());
        stats = drawn.value();
    });
    REQUIRE(image.has_value());
    REQUIRE(stats.has_value());
    CHECK(stats->planet_triangles > 10'000);
    CHECK(stats->ribbon_segments == 199);
    CHECK(stats->ground_segments == 199);
    CHECK(stats->markers == 1);
    // Something bright and saturated (the ribbon) is in the image.
    std::size_t coloured = 0;
    const ImageRgba8& pixels = image.value();
    for (std::size_t i = 0; i + 3 < pixels.pixels.size(); i += 4) {
        const int r = pixels.pixels[i];
        const int g = pixels.pixels[i + 1];
        const int b = pixels.pixels[i + 2];
        coloured +=
            (std::max({r, g, b}) > 120 && std::max({r, g, b}) - std::min({r, g, b}) > 60) ? 1U : 0U;
    }
    CHECK(coloured > 100);
}
