#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <sstream>
#include <string>
#include <utility>

#include "../render/gl_test_support.h"
#include "app_test_support.h"
#include "orrery/app/cli.h"
#include "orrery/app/headless.h"
#include "orrery/app/space_view.h"
#include "orrery/core/schwarzschild_geodesic.h"
#include "orrery/core/state_hash.h"
#include "orrery/render/black_hole.h"
#include "orrery/render/png.h"
#include "orrery/render/ssim.h"
#include "orrery/sim/space_experiment.h"

// Headless path of orrery_app (spec 7.7): the experiment is advanced through
// the application's Simulation and drawn by SpaceRenderer offscreen. The
// golden reference lives in tests/app/golden/ and is written only with
// ORRERY_UPDATE_GOLDEN=1; tests tagged [gl] skip without an OpenGL 4.6
// context unless ORRERY_REQUIRE_GL=1.

using namespace orrery;
using orrery::render::test::environment_flag;

namespace {

constexpr double kSsimThreshold = 0.98;
constexpr std::int64_t kGoldenFrames = 600;

sim::SpaceExperiment solar_experiment() {
    auto experiment = sim::load_space_experiment(app_test::experiment_dir() / "solar_system.json");
    REQUIRE(experiment.has_value());
    return std::move(experiment).value();
}

std::filesystem::path output_dir() {
    const std::filesystem::path dir(ORRERY_RENDER_OUTPUT_DIR);
    std::filesystem::create_directories(dir);
    return dir;
}

// Renders, or skips when no context exists (fails under ORRERY_REQUIRE_GL=1).
app::HeadlessFrame render_or_skip(const sim::SpaceExperiment& experiment,
                                  const app::HeadlessRequest& request) {
    auto frame = app::render_headless(experiment, request);
    if (!frame && frame.error().kind == app::HeadlessErrorKind::no_context) {
        if (environment_flag("ORRERY_REQUIRE_GL")) {
            FAIL("OpenGL 4.6 required (ORRERY_REQUIRE_GL=1) but unavailable: "
                 << frame.error().message);
        }
        SKIP("no OpenGL 4.6 core context: " << frame.error().message);
    }
    REQUIRE(frame.has_value());
    return std::move(frame).value();
}

} // namespace

TEST_CASE("Headless frames advance the experiment deterministically", "[app][headless]") {
    const sim::SpaceExperiment experiment = solar_experiment();
    auto zero = app::advance_headless(experiment, 0);
    REQUIRE(zero.has_value());
    CHECK(zero.value().status().steps == 0);
    CHECK(core::state_hash(zero.value().state()) == core::state_hash(experiment.scene.state));

    auto first = app::advance_headless(experiment, kGoldenFrames);
    auto second = app::advance_headless(experiment, kGoldenFrames);
    REQUIRE(first.has_value());
    REQUIRE(second.has_value());
    // 600 frames of 1/60 s at 1e6: 1.00000002e16 ns / 4.32e12 ns per step.
    CHECK(first.value().status().steps == 2314);
    CHECK(core::state_hash(first.value().state()) == core::state_hash(second.value().state()));
    CHECK_FALSE(first.value().status().warp_limited);
}

TEST_CASE("Golden snapshot: headless solar system experiment", "[app][headless][gl][golden]") {
    const sim::SpaceExperiment experiment = solar_experiment();
    app::HeadlessRequest request;
    request.width = 960;
    request.height = 540;
    request.frames = kGoldenFrames;
    const app::HeadlessFrame frame = render_or_skip(experiment, request);
    const render::ImageRgba8& image = frame.image;
    REQUIRE(image.width == 960);
    REQUIRE(image.height == 540);
    CHECK(frame.status.steps == 2314);
    CHECK(frame.snapshot->time().value() == Catch::Approx(2314 * 0.05));

    // Not a blank frame: the Sun is bright, space mostly dark.
    double total = 0.0;
    double peak = 0.0;
    for (std::uint32_t y = 0; y < image.height; ++y) {
        for (std::uint32_t x = 0; x < image.width; ++x) {
            const std::size_t o = image.offset(x, y);
            const double luma = 0.2126 * image.pixels[o] + 0.7152 * image.pixels[o + 1] +
                                0.0722 * image.pixels[o + 2];
            total += luma;
            peak = std::max(peak, luma);
        }
    }
    const double mean = total / (960.0 * 540.0);
    CHECK(peak > 235.0);
    CHECK(mean > 1.0);
    CHECK(mean < 40.0);

    const std::filesystem::path actual = output_dir() / "headless_solar_system_actual.png";
    REQUIRE(render::write_png(actual, image).has_value());
    const std::filesystem::path golden =
        app_test::test_source_dir() / "golden" / "headless_solar_system.png";
    if (environment_flag("ORRERY_UPDATE_GOLDEN")) {
        std::filesystem::create_directories(golden.parent_path());
        REQUIRE(render::write_png(golden, image).has_value());
        WARN("ORRERY_UPDATE_GOLDEN=1: wrote " << golden.string()
                                              << "; review it before committing");
        return;
    }
    if (!std::filesystem::exists(golden)) {
        FAIL("golden reference " << golden.string()
                                 << " is missing; generate it with ORRERY_UPDATE_GOLDEN=1 "
                                    "orrery_test_app \"[golden]\" and review it");
    }
    const auto reference = render::read_png(golden);
    REQUIRE(reference.has_value());
    const auto score = render::ssim(reference.value(), image);
    REQUIRE(score.has_value());
    INFO("actual image: " << actual.string());
    CHECK(score.value().value > kSsimThreshold);
}

TEST_CASE("Headless command: exit codes and messages", "[app][headless][cli]") {
    const std::array<std::filesystem::path, 1> dirs{app_test::experiment_dir()};
    std::ostringstream out;
    std::ostringstream err;

    app::CliOptions options;
    options.mode = app::RunMode::headless;
    options.experiment = "no_such_experiment";
    options.out = output_dir() / "never.png";
    CHECK(app::run_headless_command(options, dirs, out, err) == app::kExitFailure);
    CHECK(err.str().starts_with("orrery_app: experiment 'no_such_experiment' not found"));
    CHECK_FALSE(std::filesystem::exists(options.out));

    // An aero experiment goes to the aero renderer, which reports what is
    // wrong with this one (an unknown geometry source) before any rendering.
    err.str("");
    options.experiment = (app_test::test_source_dir() / "fixtures" / "aero_domain.json").string();
    CHECK(app::run_headless_command(options, dirs, out, err) == app::kExitFailure);
    CHECK(err.str().find("inputs.geometry: unknown geometry source 'sphere'") != std::string::npos);
    CHECK_FALSE(std::filesystem::exists(options.out));
}

TEST_CASE("Headless command writes the PNG", "[app][headless][cli][gl]") {
    const std::array<std::filesystem::path, 1> dirs{app_test::experiment_dir()};
    std::ostringstream out;
    std::ostringstream err;
    app::CliOptions options;
    options.mode = app::RunMode::headless;
    options.experiment = "figure_eight";
    options.frame = 30;
    options.width = 320;
    options.height = 180;
    options.out = output_dir() / "headless_command.png";
    std::filesystem::remove(options.out);
    const int code = app::run_headless_command(options, dirs, out, err);
    if (code == app::kExitNoGlContext) {
        if (environment_flag("ORRERY_REQUIRE_GL")) {
            FAIL("OpenGL 4.6 required (ORRERY_REQUIRE_GL=1) but unavailable: " << err.str());
        }
        SKIP(err.str());
    }
    INFO(err.str());
    REQUIRE(code == app::kExitOk);
    CHECK(err.str().empty());
    CHECK(out.str().find("Figure-eight") != std::string::npos);
    const auto written = render::read_png(options.out);
    REQUIRE(written.has_value());
    CHECK(written.value().width == 320);
    CHECK(written.value().height == 180);

    // An unwritable destination is a failure with a message, not a crash.
    err.str("");
    options.out = output_dir() / "missing_dir" / "deeper" / "x.png";
    CHECK(app::run_headless_command(options, dirs, out, err) == app::kExitFailure);
    CHECK(err.str().find("cannot write") != std::string::npos);
}

// Phase 8b: aero experiments through the same command. The panel solve of
// the bundled NACA 0012 section is direct; the image shows the body in the
// Cp colours (blue suction above, red stagnation at the nose) on the dark
// studio backdrop, with the Cp legend in the overlay.
TEST_CASE("Headless command renders an aero experiment", "[app][headless][cli][gl][aero]") {
    const std::array<std::filesystem::path, 1> dirs{app_test::experiment_dir()};
    std::ostringstream out;
    std::ostringstream err;
    app::CliOptions options;
    options.mode = app::RunMode::headless;
    options.experiment = "aero_naca0012_panel";
    options.width = 480;
    options.height = 270;
    options.out = output_dir() / "headless_aero_naca0012.png";
    std::filesystem::remove(options.out);
    const int code = app::run_headless_command(options, dirs, out, err);
    if (code == app::kExitNoGlContext) {
        if (environment_flag("ORRERY_REQUIRE_GL")) {
            FAIL("OpenGL 4.6 required (ORRERY_REQUIRE_GL=1) but unavailable: " << err.str());
        }
        SKIP(err.str());
    }
    INFO(err.str());
    REQUIRE(code == app::kExitOk);
    CHECK(out.str().find("NACA 0012 section, chord 1 m, Panel method, AoA 4 deg") !=
          std::string::npos);
    CHECK(out.str().find("Cl 0.") != std::string::npos);
    const auto written = render::read_png(options.out);
    REQUIRE(written.has_value());
    const render::ImageRgba8& image = written.value();
    REQUIRE(image.width == 480);
    REQUIRE(image.height == 270);
    // Not blank: the backdrop is dark, the body adds saturated Cp colours.
    std::size_t bluish = 0;
    std::size_t reddish = 0;
    double total = 0.0;
    for (std::uint32_t y = 0; y < image.height; ++y) {
        for (std::uint32_t x = 0; x < image.width; ++x) {
            const std::size_t o = image.offset(x, y);
            const int r = image.pixels[o];
            const int g = image.pixels[o + 1];
            const int b = image.pixels[o + 2];
            total += 0.2126 * r + 0.7152 * g + 0.0722 * b;
            bluish += (b > r + 40 && b > 80) ? 1U : 0U;
            reddish += (r > b + 40 && r > 80) ? 1U : 0U;
        }
    }
    const double mean = total / (480.0 * 270.0);
    CHECK(mean > 2.0);
    CHECK(mean < 120.0);
    CHECK(bluish > 100);
    CHECK(reddish > 20);
}

TEST_CASE("Framing camera centres the dominant body", "[app][headless]") {
    const sim::SpaceExperiment experiment = solar_experiment();
    const auto snapshot = core::SimSnapshot::capture(experiment.scene.state);
    const render::OrbitMode orbit = app::framing_orbit(*snapshot, 16.0 / 9.0);
    REQUIRE(orbit.target.has_value());
    CHECK(snapshot->names()[*snapshot->index_of(*orbit.target)] == "Sun");
    const double au = 149597870700.0;
    // The median planet distance (Mars, about 1.5 AU) fills the frame.
    CHECK(orbit.distance > 2.0 * au);
    CHECK(orbit.distance < 10.0 * au);

    // Equal masses: no dominant body, the camera looks at the centre of mass.
    auto figure = scene::load_scene_file(app_test::scenario_dir() / "figure_eight.json");
    REQUIRE(figure.has_value());
    const auto three = core::SimSnapshot::capture(figure.value().state);
    const render::OrbitMode around = app::framing_orbit(*three, 1.0);
    CHECK_FALSE(around.target.has_value());
    CHECK(std::abs(around.focus.x) < 1.0e-6 * au);
    CHECK(std::abs(around.focus.y) < 1.0e-6 * au);
}

// Phase 5: the black_hole experiment opens on its accretion disk, 12 degrees
// above the disk plane, lensed by the black-hole pass, and the headless
// benchmark times that view.
TEST_CASE("Headless black hole experiment frames the disk and lenses it", "[app][headless][gl]") {
    auto experiment = sim::load_space_experiment(app_test::experiment_dir() / "black_hole.json");
    REQUIRE(experiment.has_value());
    const auto snapshot = core::SimSnapshot::capture(experiment.value().scene.state);
    const render::OrbitMode orbit = app::framing_orbit(*snapshot, 16.0 / 9.0);
    const double pi = 3.141592653589793;
    CHECK(orbit.pitch == Catch::Approx(12.0 * pi / 180.0));
    // G M / c^2 of 4.3e6 Msun is 6.35e9 m; at 16:9 the 30 M disk spans the
    // width: its widest rim points, seen in perspective, lie at
    // 30 / sqrt(D^2 - (30 cos 12 deg)^2), 1 / 1.08 of the half field of view.
    const double m = 0.5 * 2953.25 * 4.3e6;
    const double in_plane = 30.0 * std::cos(12.0 * pi / 180.0);
    const double wide = 1.08 * 30.0 / (std::tan(20.0 * pi / 180.0) * 16.0 / 9.0);
    CHECK(orbit.distance / m ==
          Catch::Approx(std::sqrt(in_plane * in_plane + wide * wide)).epsilon(1e-3));

    app::HeadlessRequest request;
    request.width = 480;
    request.height = 270;
    const app::HeadlessFrame frame = render_or_skip(experiment.value(), request);
    const render::ImageRgba8& image = frame.image;
    const auto luma = [&](std::uint32_t x, std::uint32_t y) {
        const std::size_t o = image.offset(x, y);
        return 0.2126 * image.pixels[o] + 0.7152 * image.pixels[o + 1] +
               0.0722 * image.pixels[o + 2];
    };
    // The shadow above the image centre is black; the disk around it bright,
    // brighter on the approaching (left) side.
    CHECK(luma(240, 115) < 3.0);
    double left = 0.0;
    double right = 0.0;
    for (std::uint32_t y = 140; y < 170; ++y) {
        for (std::uint32_t x = 40; x < 140; ++x) {
            left += luma(x, y);
            right += luma(479 - x, y);
        }
    }
    CHECK(left > 1.5 * right);
    CHECK(right > 3000.0); // the receding side still shows

    const auto bench = app::benchmark_headless(experiment.value(), request, 5);
    REQUIRE(bench.has_value());
    CHECK(bench.value().frames == 5);
    CHECK(bench.value().lensing);
    CHECK(bench.value().fps > 0.0);
    CHECK(bench.value().gpu_median_ms > 0.0);
}

// Review fix: the black-hole framing fits the lensed disk in both directions
// and keeps the camera outside it (and so outside the horizon) at any aspect
// ratio. Rays through the border of the frame, traced with the
// double-precision geodesics, must miss the disk, while rays through a
// rectangle at 65 percent of the frame must meet it (the disk still fills
// the view in its limiting direction).
TEST_CASE("Black hole framing fits the disk at wide and portrait aspect ratios",
          "[app][headless]") {
    namespace geo = core::geodesic;
    auto experiment = sim::load_space_experiment(app_test::experiment_dir() / "black_hole.json");
    REQUIRE(experiment.has_value());
    const auto snapshot = core::SimSnapshot::capture(experiment.value().scene.state);
    const geo::DiskModel disk = render::lensing_disk();
    struct Size {
        int width;
        int height;
    };
    constexpr std::array<Size, 6> kSizes{
        {{8192, 64}, {3840, 1080}, {2560, 1080}, {1920, 1080}, {1080, 1920}, {256, 1024}}};
    for (const Size& size : kSizes) {
        INFO("frame " << size.width << "x" << size.height);
        const render::FrameInfo frame{size.width, size.height};
        const render::Camera camera = app::framing_camera(*snapshot, frame.aspect());
        const auto target =
            render::select_lensed_black_hole(*snapshot, camera, frame, render::RenderSettings{});
        REQUIRE(target.has_value());
        CHECK(target->observer_radius > render::kDiskOuterRadius);
        const glm::dvec3 origin = -target->hole_render / target->mass_length;
        const auto hits_disk_on_rectangle = [&](double scale) {
            const glm::dvec2 centre{0.5 * size.width, 0.5 * size.height};
            const glm::dvec2 half = 0.5 * scale * glm::dvec2(size.width, size.height);
            constexpr int kSamples = 96;
            for (int k = 0; k <= kSamples; ++k) {
                const double t = 2.0 * static_cast<double>(k) / kSamples - 1.0;
                const std::array<glm::dvec2, 4> points{centre + glm::dvec2(t * half.x, -half.y),
                                                       centre + glm::dvec2(t * half.x, half.y),
                                                       centre + glm::dvec2(-half.x, t * half.y),
                                                       centre + glm::dvec2(half.x, t * half.y)};
                for (const glm::dvec2& p : points) {
                    const glm::dvec3 d = render::pixel_ray(camera, frame, p);
                    const geo::RayTrace trace =
                        geo::trace_ray({origin.x, origin.y, origin.z}, {d.x, d.y, d.z}, disk);
                    if (std::any_of(trace.crossings.begin(), trace.crossings.end(),
                                    [](const geo::PlaneCrossing& c) { return c.on_disk; })) {
                        return true;
                    }
                }
            }
            return false;
        };
        CHECK_FALSE(hits_disk_on_rectangle(1.0));
        CHECK(hits_disk_on_rectangle(0.65));
    }
}

// Review fix: the asset directories come from where the executable really
// is, not from argv[0] (a bare name started through PATH).
TEST_CASE("The running executable is located through the operating system", "[app][headless]") {
    const auto running = app::running_executable();
    REQUIRE(running.has_value());
    CHECK(running->is_absolute());
    CHECK(std::filesystem::is_regular_file(*running));
    CHECK(running->stem() == "orrery_test_app");
    // argv[0] is only a fallback: a bare name does not override the system.
    CHECK(app::executable_path("orrery_test_app") == *running);
}