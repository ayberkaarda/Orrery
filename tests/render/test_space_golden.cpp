#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>

#include "../../src/render/gl_context_internal.h"
#include "gl_test_support.h"
#include "orrery/render/png.h"
#include "orrery/render/space_renderer.h"
#include "orrery/render/ssim.h"
#include "orrery/render/ui_app_bar.h"
#include "orrery/render/ui_context.h"
#include "orrery/render/ui_panels.h"
#include "space_test_support.h"

// Golden snapshots of the Phase 4b space scene (docs/rendering.md, "Golden
// snapshots"): the J2000 solar system with trails and starfield, the
// Earth-Moon system in true and clamped scale, and the ImGui layer rendered
// into the offscreen target. References live in tests/render/golden/ and are
// written only with ORRERY_UPDATE_GOLDEN=1 (review them before committing).

using namespace orrery;
using namespace orrery::render;
using orrery::render::test::environment_flag;
using orrery::render::test::renderer_or_skip;

namespace {

constexpr double kSsimThreshold = 0.98;
// Bodies closer than this to a star are left to the whole-image comparison.
constexpr double kStarClearancePx = 14.0;
// Per-body region checks (golden_mismatches): a body must stand out from its
// surroundings by kMinBodyContrast luma levels in both images, and its summed
// and peak luma above the background must agree within these fractions (plus
// a small absolute allowance for anti-aliasing differences between drivers).
constexpr double kMinBodyContrast = 25.0;
constexpr double kRegionSignalTolerance = 0.35;
constexpr double kRegionPeakTolerance = 0.3;
constexpr double kAu = 149597870700.0;
constexpr double kDegree = 3.141592653589793 / 180.0;

std::size_t index_of(const core::SimSnapshot& snapshot, const std::string& name) {
    for (std::size_t i = 0; i < snapshot.size(); ++i) {
        if (snapshot.names()[i] == name) {
            return i;
        }
    }
    FAIL("no body named " << name);
    return 0;
}

// Orbit camera whose eye sits at `eye` and looks at `target` (world, metres).
Camera look_at(const glm::dvec3& eye, const glm::dvec3& target, double fov_degrees) {
    Camera camera;
    CameraLens lens;
    lens.vertical_fov = fov_degrees * kDegree;
    REQUIRE(camera.set_lens(lens).has_value());
    const glm::dvec3 offset = eye - target;
    OrbitMode orbit;
    orbit.focus = target;
    orbit.distance = glm::length(offset);
    orbit.yaw = std::atan2(offset.y, offset.x);
    orbit.pitch = std::asin(offset.z / orbit.distance);
    camera.set_mode(orbit);
    camera.update(0.0, CameraInput{}, BodyFrameLookup{});
    return camera;
}

Camera solar_system_camera(const core::SimSnapshot& snapshot, double distance_au) {
    const glm::dvec3 sun = world_position(snapshot, index_of(snapshot, "Sun"));
    const double yaw = 72.0 * kDegree;
    const double pitch = 30.0 * kDegree;
    const glm::dvec3 direction{std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw),
                               std::sin(pitch)};
    // Look slightly past the Sun so that the outer orbits fill the frame.
    const glm::dvec3 target = sun + glm::dvec3(2.2 * kAu, 3.6 * kAu, 0.0);
    return look_at(target + direction * distance_au * kAu, target, 40.0);
}

// Earth-Moon view: the camera 40,000 km from the Earth, turned 32 degrees
// from the anti-Moon direction towards the Sun, so that the Moon appears
// beside the half-lit Earth (at J2000 the Moon is 58 degrees from the Sun as
// seen from the Earth, so no close viewpoint shows both bodies more than
// about half lit).
Camera earth_moon_camera(const core::SimSnapshot& snapshot) {
    const glm::dvec3 earth = world_position(snapshot, index_of(snapshot, "Earth"));
    const glm::dvec3 moon = world_position(snapshot, index_of(snapshot, "Moon"));
    const glm::dvec3 sun = world_position(snapshot, index_of(snapshot, "Sun"));
    const glm::dvec3 m = glm::normalize(moon - earth);
    const glm::dvec3 s = glm::normalize(sun - earth);
    const glm::dvec3 p = glm::normalize(s - glm::dot(s, m) * m);
    const double angle = 32.0 * kDegree;
    const glm::dvec3 u =
        glm::normalize(-std::cos(angle) * m + std::sin(angle) * p + glm::dvec3(0.0, 0.0, 0.15));
    const glm::dvec3 eye = earth + u * 3.2e7;
    // Aim between the Earth and the Moon.
    const glm::dvec3 target = earth + (moon - earth) * 0.06;
    return look_at(eye, target, 55.0);
}

double luma(const ImageRgba8& image, std::uint32_t x, std::uint32_t y) {
    const std::size_t o = image.offset(x, y);
    return 0.2126 * image.pixels[o] + 0.7152 * image.pixels[o + 1] + 0.0722 * image.pixels[o + 2];
}

double window_luma(const ImageRgba8& image, double cx, double cy, double radius) {
    double sum = 0.0;
    const auto x0 = static_cast<std::uint32_t>(std::max(0.0, cx - radius));
    const auto y0 = static_cast<std::uint32_t>(std::max(0.0, cy - radius));
    const auto x1 = static_cast<std::uint32_t>(std::min<double>(image.width - 1, cx + radius));
    const auto y1 = static_cast<std::uint32_t>(std::min<double>(image.height - 1, cy + radius));
    for (std::uint32_t y = y0; y <= y1; ++y) {
        for (std::uint32_t x = x0; x <= x1; ++x) {
            sum += luma(image, x, y);
        }
    }
    return sum;
}

// A small body whose pixels the golden comparison checks on their own:
// whole-image SSIM cannot see a few missing pixels in a mostly empty frame.
struct BodyRegion {
    std::string name;
    double x = 0.0; // projected centre [px]
    double y = 0.0;
    double radius_px = 0.0;
};

// Every visible non-star body inside the image, at least star_clearance_px
// from every star (whose glare dominates there); bodies closer than 2 px to
// an earlier one share its region (the Moon beside the Earth).
std::vector<BodyRegion> small_body_regions(const core::SimSnapshot& snapshot,
                                           const PreparedBodies& prepared, const FrameInfo& frame,
                                           double star_clearance_px) {
    std::vector<BodyRegion> regions;
    for (const BodyScreenInfo& info : prepared.screen) {
        if (!info.visible || snapshot.kind_tag(info.index) == core::BodyKindTag::star) {
            continue;
        }
        const double margin = info.radius_px + 6.0;
        if (info.pixel.x < margin || info.pixel.y < margin || info.pixel.x > frame.width - margin ||
            info.pixel.y > frame.height - margin) {
            continue;
        }
        bool clear = true;
        for (const BodyScreenInfo& other : prepared.screen) {
            if (other.visible && snapshot.kind_tag(other.index) == core::BodyKindTag::star &&
                glm::length(other.pixel - info.pixel) < star_clearance_px) {
                clear = false;
            }
        }
        bool merged = false;
        for (BodyRegion& region : regions) {
            if (glm::length(glm::dvec2(region.x, region.y) - info.pixel) < 2.0) {
                region.name += "/" + snapshot.names()[info.index];
                region.radius_px = std::max(region.radius_px, info.radius_px);
                merged = true;
            }
        }
        if (clear && !merged) {
            regions.push_back(
                {snapshot.names()[info.index], info.pixel.x, info.pixel.y, info.radius_px});
        }
    }
    return regions;
}

// Luma above the local background inside the square window of half-size
// ceil(radius) + 2 around a body; the background is the median luma of the
// window border (stars and trail pixels there do not move it).
struct RegionStats {
    double signal = 0.0; // sum of the luma above the background
    double peak = 0.0;   // largest luma above the background
};

RegionStats region_stats(const ImageRgba8& image, const BodyRegion& region) {
    const int h = static_cast<int>(std::ceil(region.radius_px)) + 2;
    const auto cx = static_cast<int>(std::lround(region.x));
    const auto cy = static_cast<int>(std::lround(region.y));
    const auto at = [&](int x, int y) {
        return luma(image, static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y));
    };
    std::vector<double> border;
    for (int k = -h; k < h; ++k) {
        border.push_back(at(cx + k, cy - h));
        border.push_back(at(cx + h, cy + k));
        border.push_back(at(cx - k, cy + h));
        border.push_back(at(cx - h, cy - k));
    }
    RegionStats stats;
    if (border.empty()) {
        return stats;
    }
    const auto middle = border.begin() + static_cast<std::ptrdiff_t>(border.size() / 2);
    std::nth_element(border.begin(), middle, border.end());
    const double background = *middle;
    for (int y = cy - h + 1; y < cy + h; ++y) {
        for (int x = cx - h + 1; x < cx + h; ++x) {
            const double above = std::max(0.0, at(x, y) - background);
            stats.signal += above;
            stats.peak = std::max(stats.peak, above);
        }
    }
    return stats;
}

// Compares an image with its golden reference: whole-image SSIM above the
// threshold, plus the pixels of every listed body region. Returns one
// message per failed check (empty: the image matches).
std::vector<std::string> golden_mismatches(const ImageRgba8& reference, const ImageRgba8& actual,
                                           const std::vector<BodyRegion>& regions) {
    std::vector<std::string> problems;
    auto score = ssim(reference, actual);
    if (!score) {
        problems.push_back("SSIM: " + score.error());
        return problems;
    }
    if (!(score.value().value > kSsimThreshold)) {
        problems.push_back("SSIM " + std::to_string(score.value().value) + " (R " +
                           std::to_string(score.value().channel[0]) + " G " +
                           std::to_string(score.value().channel[1]) + " B " +
                           std::to_string(score.value().channel[2]) +
                           ") <= " + std::to_string(kSsimThreshold));
    }
    if (reference.width != actual.width || reference.height != actual.height) {
        return problems;
    }
    for (const BodyRegion& region : regions) {
        const RegionStats r = region_stats(reference, region);
        const RegionStats a = region_stats(actual, region);
        const std::string where = region.name + " at (" + std::to_string(region.x) + ", " +
                                  std::to_string(region.y) + ")";
        if (r.peak < kMinBodyContrast) {
            problems.push_back(where + ": the reference shows no body (peak contrast " +
                               std::to_string(r.peak) + ")");
            continue;
        }
        if (a.peak < kMinBodyContrast) {
            problems.push_back(where + ": the image shows no body (peak contrast " +
                               std::to_string(a.peak) + ")");
            continue;
        }
        if (std::abs(a.signal - r.signal) > kRegionSignalTolerance * r.signal + 20.0) {
            problems.push_back(where + ": body signal " + std::to_string(a.signal) +
                               " differs from the reference " + std::to_string(r.signal));
        }
        if (std::abs(a.peak - r.peak) > kRegionPeakTolerance * r.peak + 6.0) {
            problems.push_back(where + ": body peak contrast " + std::to_string(a.peak) +
                               " differs from the reference " + std::to_string(r.peak));
        }
    }
    return problems;
}

// Writes the actual image, then compares it with (or, on request, writes)
// the golden reference.
void check_golden(const ImageRgba8& image, const std::string& name,
                  const std::vector<BodyRegion>& regions = {}) {
    const std::filesystem::path output_dir = ORRERY_RENDER_OUTPUT_DIR;
    std::filesystem::create_directories(output_dir);
    const std::filesystem::path actual_path = output_dir / (name + "_actual.png");
    REQUIRE(write_png(actual_path, image).has_value());
    const std::filesystem::path golden_path =
        std::filesystem::path(ORRERY_RENDER_GOLDEN_DIR) / (name + ".png");
    if (environment_flag("ORRERY_UPDATE_GOLDEN")) {
        std::filesystem::create_directories(golden_path.parent_path());
        REQUIRE(write_png(golden_path, image).has_value());
        WARN("ORRERY_UPDATE_GOLDEN=1: wrote the golden reference " << golden_path.string()
                                                                   << "; review it before "
                                                                      "committing");
        return;
    }
    if (!std::filesystem::exists(golden_path)) {
        FAIL("golden reference " << golden_path.string()
                                 << " is missing; generate it with ORRERY_UPDATE_GOLDEN=1 "
                                    "orrery_test_render \"[golden]\" and review it");
    }
    auto golden = read_png(golden_path);
    REQUIRE(golden.has_value());
    const std::vector<std::string> problems = golden_mismatches(golden.value(), image, regions);
    for (const std::string& problem : problems) {
        UNSCOPED_INFO(problem);
    }
    INFO("actual image: " << actual_path.string());
    CHECK(problems.empty());
}

struct UiScene {
    std::vector<std::string> scenarios{"figure_eight", "solar_system_j2000", "sun_earth_moon"};
    AppBarData bar;
    BodyInspectorData inspector;
    SystemMonitorData monitor;
    RenderSettings settings;
    std::vector<SceneLabel> labels;
};

// The workspace of the window (docs/app.md, "Window"): the J2000 solar system
// with the Earth selected, the drift series of the backward run and a running
// simulation at the solar_system experiment's warp.
UiScene workspace_scene(const test::SolarSystemHistory& history) {
    const core::SimSnapshot& snapshot = *history.snapshot;
    const core::BodyId earth = snapshot.ids()[index_of(snapshot, "Earth")];
    UiScene scene;
    scene.bar.paused = false;
    scene.bar.time_warp = 1'000'000;
    scene.bar.effective_warp = 1'000'000.0;
    scene.bar.sim_time_days = 0.0;
    scene.bar.scenario = 1;
    scene.bar.fps = 144.0;
    scene.inspector = make_body_inspector_data(snapshot, earth);
    scene.monitor.sim_time_days = 0.0;
    scene.monitor.body_count = snapshot.size();
    scene.monitor.frame_time_ms = 2.4;
    scene.monitor.physics_step_ms = 0.021;
    scene.monitor.integrator = "leapfrog_kdk";
    scene.monitor.series_label = "Relative energy drift (150 yr backward run)";
    scene.monitor.series_time_days = history.drift_days;
    scene.monitor.series_values = history.drift_values;
    return scene;
}

// One frame of the window's UI: the app bar, the dock space (returns its
// central area), the scene labels and the three panels.
UiRect draw_workspace(UiContext& ui, const UiScene& scene, const FrameInfo& frame) {
    UiFrameInput input;
    input.width = frame.width;
    input.height = frame.height;
    ui.begin_frame(input);
    AppBarData bar = scene.bar;
    bar.scenarios = scene.scenarios;
    (void)draw_app_bar(bar);
    const UiRect central = ui.dockspace();
    draw_scene_labels(scene.labels);
    (void)draw_body_inspector(scene.inspector);
    (void)draw_system_monitor(scene.monitor);
    (void)draw_render_settings(scene.settings);
    ui.end_frame();
    return central;
}

} // namespace

TEST_CASE("Golden snapshot: J2000 solar system with trails and starfield",
          "[render][gl][golden][space]") {
    const test::SolarSystemHistory history = test::solar_system_history(150.0);
    const core::SimSnapshot& snapshot = *history.snapshot;
    HdrSettings hdr;
    hdr.samples = 4;
    OffscreenRenderer renderer = renderer_or_skip(960, 540, hdr);
    auto space = SpaceRenderer::create();
    REQUIRE(space.has_value());
    CHECK(space.value().star_count() == StarfieldParams{}.count);

    const Camera camera = solar_system_camera(snapshot, 12.5);
    SpaceView view{&snapshot, &history.trails, &camera, RenderSettings{}};
    SpaceDrawStats stats;
    auto rendered =
        renderer.render([&](const FrameInfo& frame) { stats = space.value().draw(view, frame); });
    REQUIRE(rendered.has_value());
    const ImageRgba8& image = rendered.value();

    CHECK(stats.bodies_drawn + stats.bodies_culled == snapshot.size());
    CHECK(stats.bodies_drawn >= 8);
    CHECK(stats.trail_segments > 5000);
    CHECK(stats.stars == StarfieldParams{}.count);
    CHECK(stats.glows == 1);

    // The Sun is the brightest object; the sky stays dark and nothing but
    // the Sun clips to white.
    const PreparedBodies prepared =
        prepare_bodies(snapshot, camera, FrameInfo{960, 540}, RenderSettings{});
    const BodyScreenInfo& sun = prepared.screen[index_of(snapshot, "Sun")];
    REQUIRE(sun.visible);
    CHECK(luma(image, static_cast<std::uint32_t>(sun.pixel.x),
               static_cast<std::uint32_t>(sun.pixel.y)) > 235.0);
    std::size_t clipped = 0;
    double total = 0.0;
    for (std::uint32_t y = 0; y < image.height; ++y) {
        for (std::uint32_t x = 0; x < image.width; ++x) {
            const double l = luma(image, x, y);
            total += l;
            const double dx = x - sun.pixel.x;
            const double dy = y - sun.pixel.y;
            clipped += l >= 254.0 && dx * dx + dy * dy > 40.0 * 40.0 ? 1U : 0U;
        }
    }
    CHECK(clipped < 40);
    CHECK(total / (960.0 * 540.0) < 30.0); // mostly dark space
    CHECK(total / (960.0 * 540.0) > 2.0);  // but stars, trails and the Milky Way show
    // Planets are checked region by region, besides the whole-image SSIM.
    const std::vector<BodyRegion> regions =
        small_body_regions(snapshot, prepared, FrameInfo{960, 540}, kStarClearancePx);
    CHECK(regions.size() >= 5);
    check_golden(image, "space_solar_system", regions);
}

TEST_CASE("Golden validation rejects a solar-system image with one small planet erased",
          "[render][golden_check]") {
    // CPU only: the reference image and the projected body positions.
    const std::filesystem::path path =
        std::filesystem::path(ORRERY_RENDER_GOLDEN_DIR) / "space_solar_system.png";
    auto loaded = read_png(path);
    REQUIRE(loaded.has_value());
    const ImageRgba8& reference = loaded.value();
    const auto snapshot = core::SimSnapshot::capture(test::load_j2000_state());
    const Camera camera = solar_system_camera(*snapshot, 12.5);
    const FrameInfo frame{960, 540};
    const PreparedBodies prepared = prepare_bodies(*snapshot, camera, frame, RenderSettings{});
    const std::vector<BodyRegion> regions =
        small_body_regions(*snapshot, prepared, frame, kStarClearancePx);
    REQUIRE(regions.size() >= 5);
    for (const BodyRegion& region : regions) {
        UNSCOPED_INFO(region.name << " at (" << region.x << ", " << region.y << "), "
                                  << region.radius_px << " px");
    }
    CHECK(golden_mismatches(reference, reference, regions).empty());

    // Small driver-dependent changes pass: a half-pixel shift of every body
    // (the mean with the right neighbour) and +-2 levels of noise around it.
    ImageRgba8 perturbed = reference;
    std::uint32_t state = 12345U;
    for (const BodyRegion& region : regions) {
        const int h = static_cast<int>(std::ceil(region.radius_px)) + 2;
        const auto cx = static_cast<int>(std::lround(region.x));
        const auto cy = static_cast<int>(std::lround(region.y));
        for (int y = cy - h; y <= cy + h; ++y) {
            for (int x = cx - h; x <= cx + h; ++x) {
                const std::size_t o =
                    perturbed.offset(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y));
                const std::size_t right = reference.offset(static_cast<std::uint32_t>(x + 1),
                                                           static_cast<std::uint32_t>(y));
                for (std::size_t c = 0; c < 3; ++c) {
                    state = state * 1664525U + 1013904223U;
                    const int noise = static_cast<int>(state >> 29U) % 5 - 2;
                    const int shifted =
                        (reference.pixels[o + c] + reference.pixels[right + c] + 1) / 2;
                    perturbed.pixels[o + c] =
                        static_cast<std::uint8_t>(std::clamp(shifted + noise, 0, 255));
                }
            }
        }
    }
    const std::vector<std::string> tolerated = golden_mismatches(reference, perturbed, regions);
    for (const std::string& problem : tolerated) {
        UNSCOPED_INFO(problem);
    }
    CHECK(tolerated.empty());

    // Erasing any one planet (its window filled with the median colour of the
    // window border) keeps the whole-image SSIM far above the threshold, yet
    // must be rejected, whether the actual or the reference image lost it.
    for (const BodyRegion& region : regions) {
        INFO("erased " << region.name << " at (" << region.x << ", " << region.y << "), "
                       << region.radius_px << " px");
        ImageRgba8 erased = reference;
        const int h = static_cast<int>(std::ceil(region.radius_px)) + 2;
        const auto cx = static_cast<int>(std::lround(region.x));
        const auto cy = static_cast<int>(std::lround(region.y));
        for (std::size_t c = 0; c < 3; ++c) {
            std::vector<std::uint8_t> border;
            for (int k = -h; k <= h; ++k) {
                for (const auto& [x, y] : {std::pair{cx + k, cy - h}, std::pair{cx + k, cy + h},
                                           std::pair{cx - h, cy + k}, std::pair{cx + h, cy + k}}) {
                    border.push_back(
                        reference.pixels[reference.offset(static_cast<std::uint32_t>(x),
                                                          static_cast<std::uint32_t>(y)) +
                                         c]);
                }
            }
            REQUIRE_FALSE(border.empty());
            const auto middle = border.begin() + static_cast<std::ptrdiff_t>(border.size() / 2);
            std::nth_element(border.begin(), middle, border.end());
            const std::uint8_t fill = *middle;
            for (int y = cy - h + 1; y < cy + h; ++y) {
                for (int x = cx - h + 1; x < cx + h; ++x) {
                    erased.pixels[erased.offset(static_cast<std::uint32_t>(x),
                                                static_cast<std::uint32_t>(y)) +
                                  c] = fill;
                }
            }
        }
        auto score = ssim(reference, erased);
        REQUIRE(score.has_value());
        CHECK(score.value().value > 0.99);
        const std::vector<std::string> problems = golden_mismatches(reference, erased, regions);
        CHECK_FALSE(problems.empty());
        bool names_body = false;
        for (const std::string& problem : problems) {
            names_body = names_body || problem.find(region.name) != std::string::npos;
        }
        CHECK(names_body);
        CHECK_FALSE(golden_mismatches(erased, reference, regions).empty());
    }
}

TEST_CASE("Golden snapshot: Earth and Moon in true and clamped scale",
          "[render][gl][golden][space]") {
    const auto snapshot = core::SimSnapshot::capture(test::load_j2000_state());
    HdrSettings hdr;
    hdr.samples = 4;
    OffscreenRenderer renderer = renderer_or_skip(640, 360, hdr);
    auto space = SpaceRenderer::create();
    REQUIRE(space.has_value());
    const Camera camera = earth_moon_camera(*snapshot);
    const FrameInfo frame{640, 360};

    RenderSettings settings;
    settings.show_trails = false;
    settings.scale_mode = ScaleMode::true_scale;
    const PreparedBodies true_prep = prepare_bodies(*snapshot, camera, frame, settings);
    auto true_scale = renderer.render([&](const FrameInfo& f) {
        (void)space.value().draw(SpaceView{snapshot.get(), nullptr, &camera, settings}, f);
    });
    REQUIRE(true_scale.has_value());
    settings.scale_mode = ScaleMode::clamped;
    settings.min_radius_scale = 2.0;
    const PreparedBodies clamped_prep = prepare_bodies(*snapshot, camera, frame, settings);
    auto clamped = renderer.render([&](const FrameInfo& f) {
        (void)space.value().draw(SpaceView{snapshot.get(), nullptr, &camera, settings}, f);
    });
    REQUIRE(clamped.has_value());

    const std::size_t earth = index_of(*snapshot, "Earth");
    const std::size_t moon = index_of(*snapshot, "Moon");
    const BodyScreenInfo& earth_true = true_prep.screen[earth];
    const BodyScreenInfo& moon_true = true_prep.screen[moon];
    const BodyScreenInfo& moon_clamped = clamped_prep.screen[moon];
    REQUIRE(earth_true.visible);
    REQUIRE(moon_true.visible);
    // The Earth is resolved: the same physical radius in both modes.
    CHECK(earth_true.radius_px > 40.0);
    CHECK(clamped_prep.screen[earth].visual_radius == earth_true.visual_radius);
    // The Moon is a few pixels in true scale and enlarged to the clamp.
    CHECK(moon_true.radius_px < 4.0);
    CHECK(moon_clamped.radius_px > moon_true.radius_px * 1.3);
    CHECK(window_luma(clamped.value(), moon_clamped.pixel.x, moon_clamped.pixel.y, 8.0) >
          1.3 * window_luma(true_scale.value(), moon_true.pixel.x, moon_true.pixel.y, 8.0));
    // The Earth disc is the same image in both modes.
    const double r = earth_true.radius_px * 0.8;
    const double earth_a =
        window_luma(true_scale.value(), earth_true.pixel.x, earth_true.pixel.y, r);
    const double earth_b = window_luma(clamped.value(), earth_true.pixel.x, earth_true.pixel.y, r);
    CHECK(std::abs(earth_a - earth_b) <= 1.0e-3 * earth_a + 1.0);
    // The day side is lit.
    CHECK(earth_a / ((2.0 * r + 1.0) * (2.0 * r + 1.0)) > 20.0);

    check_golden(true_scale.value(), "space_earth_moon_true_scale");
    check_golden(clamped.value(), "space_earth_moon_clamped");
}

TEST_CASE("Golden snapshot: docked UI panels over the space scene", "[render][gl][golden][ui]") {
    const test::SolarSystemHistory history = test::solar_system_history(150.0);
    const core::SimSnapshot& snapshot = *history.snapshot;
    constexpr int kWidth = 1280;
    constexpr int kHeight = 720;
    HdrSettings hdr;
    hdr.samples = 4;
    OffscreenRenderer renderer = renderer_or_skip(kWidth, kHeight, hdr);
    auto space = SpaceRenderer::create();
    REQUIRE(space.has_value());
    auto created = UiContext::create(renderer.context());
    REQUIRE(created.has_value());
    UiContext ui = std::move(created).value();
    CHECK(ui.has_renderer());

    const core::BodyId earth = snapshot.ids()[index_of(snapshot, "Earth")];
    UiScene scene = workspace_scene(history);

    // The scene is framed for the central area left free by the panels.
    const Camera camera = solar_system_camera(snapshot, 14.0);
    const FrameInfo frame{kWidth, kHeight};
    const PreparedBodies prepared = prepare_bodies(snapshot, camera, frame, scene.settings);
    scene.labels = make_scene_labels(snapshot, prepared.screen, earth);
    CHECK(scene.labels.size() >= 4);
    CHECK(scene.labels.front().selected);
    CHECK(scene.labels.front().name == "Earth");

    UiRect central{};
    const auto draw_ui = [&](const FrameInfo& f) { central = draw_workspace(ui, scene, f); };
    const auto draw_scene = [&](const FrameInfo& f) {
        (void)space.value().draw(SpaceView{&snapshot, &history.trails, &camera, scene.settings}, f);
    };
    // The first frames settle the dock layout and window sizes.
    for (int i = 0; i < 2; ++i) {
        REQUIRE(renderer.render(draw_scene, draw_ui).has_value());
    }
    auto rendered = renderer.render(draw_scene, draw_ui);
    REQUIRE(rendered.has_value());
    const ImageRgba8& image = rendered.value();

    // Panels are opaque theme surfaces with readable text.
    const auto left = ui.window_rect(kBodyInspectorWindow);
    const auto bottom = ui.window_rect(kSystemMonitorWindow);
    REQUIRE(left.has_value());
    REQUIRE(bottom.has_value());
    CHECK(ui.window_docked(kSystemMonitorWindow));
    std::size_t text_pixels = 0;
    for (auto y = static_cast<std::uint32_t>(left->y); y < left->y + left->height && y < kHeight;
         ++y) {
        for (auto x = static_cast<std::uint32_t>(left->x); x < left->x + left->width; ++x) {
            text_pixels += luma(image, x, y) > 150.0 ? 1U : 0U;
        }
    }
    CHECK(text_pixels > 1500);
    // The app bar runs across the top; the dock space starts below it.
    CHECK(central.y > 20.0);
    CHECK(central.y < 60.0);
    CHECK(left->y >= central.y - 0.5);
    // The central area still shows the scene: the Sun is there.
    CHECK(central.width > kWidth * 0.4);
    const BodyScreenInfo& sun = prepared.screen[index_of(snapshot, "Sun")];
    REQUIRE(sun.visible);
    CHECK(sun.pixel.x > central.x);
    CHECK(sun.pixel.x < central.x + central.width);
    CHECK(luma(image, static_cast<std::uint32_t>(sun.pixel.x),
               static_cast<std::uint32_t>(sun.pixel.y)) > 235.0);
    check_golden(image, "ui_space_workspace");
}

TEST_CASE("The workspace layout holds from 1024 to 1920 px and in its empty state",
          "[render][gl][ui]") {
    // The window's composition at common sizes, plus the state before the
    // first snapshot (no bodies, no series) with a limited warp and a
    // simulation message. The images are written for review.
    const test::SolarSystemHistory history = test::solar_system_history(150.0);
    const core::SimSnapshot& snapshot = *history.snapshot;
    const core::BodyId earth = snapshot.ids()[index_of(snapshot, "Earth")];
    struct Case {
        int width;
        int height;
        bool empty;
    };
    for (const Case c : {Case{1920, 1080, false}, Case{1366, 768, false}, Case{1024, 640, false},
                         Case{1024, 640, true}}) {
        const std::string name = "ui_workspace_" + std::to_string(c.width) + "x" +
                                 std::to_string(c.height) + (c.empty ? "_empty" : "");
        INFO(name);
        HdrSettings hdr;
        hdr.samples = 4;
        OffscreenRenderer renderer = renderer_or_skip(c.width, c.height, hdr);
        auto space = SpaceRenderer::create();
        REQUIRE(space.has_value());
        auto created = UiContext::create(renderer.context());
        REQUIRE(created.has_value());
        UiContext ui = std::move(created).value();

        UiScene scene = workspace_scene(history);
        const Camera camera = solar_system_camera(snapshot, 14.0);
        if (c.empty) {
            scene.bar.warp_limited = true;
            scene.bar.effective_warp = 640'000.0;
            scene.bar.message = "Step failed: non-finite state; simulation paused";
            scene.inspector = BodyInspectorData{};
            scene.monitor = SystemMonitorData{};
            scene.monitor.series_label = "Relative energy drift";
        } else {
            const PreparedBodies prepared =
                prepare_bodies(snapshot, camera, FrameInfo{c.width, c.height}, scene.settings);
            scene.labels = make_scene_labels(snapshot, prepared.screen, earth);
        }
        UiRect central{};
        const auto draw_ui = [&](const FrameInfo& f) { central = draw_workspace(ui, scene, f); };
        const auto draw_scene = [&](const FrameInfo& f) {
            if (!c.empty) {
                (void)space.value().draw(
                    SpaceView{&snapshot, &history.trails, &camera, scene.settings}, f);
            }
        };
        for (int i = 0; i < 2; ++i) {
            REQUIRE(renderer.render(draw_scene, draw_ui).has_value());
        }
        auto rendered = renderer.render(draw_scene, draw_ui);
        REQUIRE(rendered.has_value());
        const std::filesystem::path output_dir = ORRERY_RENDER_OUTPUT_DIR;
        std::filesystem::create_directories(output_dir);
        REQUIRE(write_png(output_dir / (name + "_actual.png"), rendered.value()).has_value());

        CHECK(ui.window_docked(kBodyInspectorWindow));
        CHECK(ui.window_docked(kSystemMonitorWindow));
        CHECK(ui.window_docked(kRenderSettingsWindow));
        CHECK(central.y > 20.0);
        CHECK(central.width > c.width * 0.4);
        CHECK(central.height > c.height * 0.5);
    }
}

TEST_CASE("The Milky Way band is brightest towards the galactic centre and absent at the pole",
          "[render][gl][sky]") {
    OffscreenRenderer renderer = renderer_or_skip(480, 270);
    auto space = SpaceRenderer::create();
    REQUIRE(space.has_value());
    const auto empty = core::SimSnapshot::capture(core::SimState{});
    const glm::dmat3 to_ecliptic = glm::transpose(ecliptic_to_galactic());
    RenderSettings settings;
    settings.star_brightness = 0.0; // the band alone
    const auto mean_luma = [&](const glm::dvec3& galactic_direction, const std::string& name) {
        const glm::dvec3 d = to_ecliptic * galactic_direction;
        const Camera camera = look_at(-d * 10.0, glm::dvec3(0.0), 60.0);
        auto image = renderer.render([&](const FrameInfo& f) {
            (void)space.value().draw(SpaceView{empty.get(), nullptr, &camera, settings}, f);
        });
        REQUIRE(image.has_value());
        const std::filesystem::path output_dir = ORRERY_RENDER_OUTPUT_DIR;
        std::filesystem::create_directories(output_dir);
        REQUIRE(write_png(output_dir / ("sky_" + name + ".png"), image.value()).has_value());
        return window_luma(image.value(), 240.0, 135.0, 134.0) / (269.0 * 269.0);
    };
    const double centre = mean_luma({1.0, 0.0, 0.0}, "galactic_centre");
    const double anticentre = mean_luma({-1.0, 0.0, 0.0}, "galactic_anticentre");
    const double pole = mean_luma({0.0, 0.0, 1.0}, "galactic_pole");
    CHECK(centre > 2.0 * anticentre);
    CHECK(anticentre > 1.5 * pole);
    CHECK(pole < 2.0);
    CHECK(centre < 120.0); // a subtle band, not a bright cloud
}

TEST_CASE("UI context lifetime is safe across contexts and without a window", "[render][gl][ui]") {
    OffscreenRenderer renderer = renderer_or_skip(320, 200);
    {
        auto created = UiContext::create(renderer.context());
        REQUIRE(created.has_value());
        UiContext ui = std::move(created).value();
        // Another context is current when the UI is destroyed: the UI makes
        // its own context current for the shutdown and restores the other one.
        OffscreenRenderer other = renderer_or_skip(64, 64);
        other.context().make_current();
        const std::size_t skipped_before = detail::gl_handle_stats().skipped_deletes;
        {
            const UiContext released = std::move(ui);
        }
        CHECK(other.context().is_current());
        CHECK(detail::gl_handle_stats().skipped_deletes == skipped_before);
        auto image = other.render({});
        CHECK(image.has_value());
    }
    {
        // Nothing is current when the UI is destroyed: the UI makes its own
        // context current for the backend shutdown and detaches it again.
        auto created = UiContext::create(renderer.context());
        REQUIRE(created.has_value());
        UiContext ui = std::move(created).value();
        detail::make_native_context_current(nullptr);
        REQUIRE(detail::current_native_context() == nullptr);
        REQUIRE(detail::current_context_serial() == 0);
        {
            const UiContext released = std::move(ui);
        }
        CHECK(detail::current_native_context() == nullptr);
        CHECK(detail::current_context_serial() == 0);
        CHECK_FALSE(renderer.context().is_current());
        auto image = renderer.render({});
        CHECK(image.has_value());
    }
    // A moved-from context is rejected without leaking an ImGui context.
    GlContext moved_from = std::move(renderer.context());
    GlContext& empty = renderer.context();
    auto rejected = UiContext::create(empty);
    CHECK_FALSE(rejected.has_value());
    // Give the context back so that the renderer releases it normally.
    renderer.context() = std::move(moved_from);
}

TEST_CASE("Solar system with trails renders a 1080p frame well within 16 ms",
          "[render][gl][perf]") {
    const test::SolarSystemHistory history = test::solar_system_history(150.0);
    const core::SimSnapshot& snapshot = *history.snapshot;
    HdrSettings hdr;
    hdr.samples = 4;
    OffscreenRenderer renderer = renderer_or_skip(1920, 1080, hdr);
    auto space = SpaceRenderer::create();
    REQUIRE(space.has_value());
    const Camera camera = solar_system_camera(snapshot, 12.5);
    const SpaceView view{&snapshot, &history.trails, &camera, RenderSettings{}};
    renderer.context().make_current();
    HdrPipeline& pipeline = renderer.pipeline();
    const GpuTimer timer = GpuTimer::create();

    std::vector<double> gpu_ms;
    std::vector<double> cpu_ms;
    for (int frame = 0; frame < 65; ++frame) {
        const auto start = std::chrono::steady_clock::now();
        timer.begin();
        pipeline.begin_scene();
        (void)space.value().draw(view, pipeline.frame());
        pipeline.resolve();
        timer.end();
        const auto submitted = std::chrono::steady_clock::now();
        const double gpu = static_cast<double>(timer.elapsed_ns()) * 1.0e-6;
        if (frame >= 5) { // warm-up: shader and driver caches
            gpu_ms.push_back(gpu);
            cpu_ms.push_back(std::chrono::duration<double, std::milli>(submitted - start).count());
        }
    }
    std::sort(gpu_ms.begin(), gpu_ms.end());
    std::sort(cpu_ms.begin(), cpu_ms.end());
    const double gpu_median = gpu_ms[gpu_ms.size() / 2];
    const double gpu_p95 = gpu_ms[gpu_ms.size() * 95 / 100];
    const double cpu_median = cpu_ms[cpu_ms.size() / 2];
    WARN("1920x1080, 4x MSAA, " << snapshot.size() << " bodies, "
                                << history.trails.settings().length << "-sample trails, "
                                << StarfieldParams{}.count << " stars on "
                                << renderer.context().info().renderer << ": GPU median "
                                << gpu_median << " ms, p95 " << gpu_p95 << " ms; CPU submit median "
                                << cpu_median << " ms");
    CHECK(gpu_p95 < 16.0);
    CHECK(gpu_median + cpu_median < 16.0);
    CHECK(renderer.context().debug_error_count() == 0);
}
