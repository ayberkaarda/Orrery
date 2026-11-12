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
#include "orrery/core/state_hash.h"
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

    err.str("");
    options.experiment = (app_test::test_source_dir() / "fixtures" / "aero_domain.json").string();
    CHECK(app::run_headless_command(options, dirs, out, err) == app::kExitFailure);
    CHECK(err.str().find("domain 'aero'") != std::string::npos);
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