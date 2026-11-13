#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "app_test_support.h"
#include "orrery/app/entry_workspace.h"
#include "orrery/app/workspace_strip.h"
#include "orrery/render/ui_context.h"
#include "orrery/sim/entry_experiments.h"

// Phase 9b: the Entry workspace's user interface under a headless UiContext
// (no window, no OpenGL): layout, a run through the solver thread with pause
// and stop, results that follow the inputs, saving, loading and comparing
// experiments, and what the view is handed.

using namespace orrery;

namespace {

std::filesystem::path scratch(const char* name) {
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / "orrery_test_entry_ui" / name;
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

struct Harness {
    explicit Harness(const std::filesystem::path& dir, double pace = 0.0)
        : ui(make_ui()), workspace(options(dir, pace)) {}
    explicit Harness(const app::EntryWorkspaceOptions& o) : ui(make_ui()), workspace(o) {}

    static render::UiContext make_ui() {
        auto created = render::UiContext::create_headless();
        REQUIRE(created.has_value());
        return std::move(created).value();
    }

    static app::EntryWorkspaceOptions options(const std::filesystem::path& dir, double pace) {
        app::EntryWorkspaceOptions o;
        o.experiment_dir = dir;
        o.pace = pace;
        return o;
    }

    // One frame of the window loop for the Entry workspace.
    std::optional<app::Workspace> frame() {
        render::UiFrameInput input;
        input.width = 1600;
        input.height = 900;
        ui.begin_frame(input);
        workspace.update(1.0 / 60.0);
        workspace.draw_bar(60.0);
        const auto picked = app::draw_workspace_strip(app::Workspace::entry);
        scene = workspace.draw_panels();
        workspace.handle_scene_input(render::read_scene_input(), 1.0 / 60.0);
        ui.end_frame();
        (void)workspace.camera(scene, 1600, 900);
        return picked;
    }

    bool frames_until(const std::function<bool()>& done) {
        return app_test::wait_until([&] {
            (void)frame();
            return done();
        });
    }

    // Starts a run and waits until this run (not an earlier one) finished.
    bool run_to_end() {
        const std::uint64_t before = workspace.progress().status.run;
        return workspace.start_run() && frames_until([&] {
                   const sim::EntryRunStatus& status = workspace.progress().status;
                   return status.run > before && status.state == sim::EntryRunState::finished;
               });
    }

    render::UiContext ui;
    app::EntryWorkspace workspace;
    render::UiRect scene;
};

} // namespace

TEST_CASE("Entry workspace lays out its panels around the scene", "[app][ui][entry]") {
    Harness h(scratch("layout"));
    for (int i = 0; i < 4; ++i) {
        CHECK_FALSE(h.frame().has_value());
    }
    for (const std::string_view name :
         {app::kEntrySetupWindow, app::kEntryProfileWindow, app::kGroundTrackWindow,
          app::kEntryExperimentsWindow, app::kEntryViewWindow}) {
        INFO(name);
        CHECK(h.ui.window_rect(name).has_value());
        CHECK(h.ui.window_docked(name));
    }
    CHECK(h.scene.width > 400.0);
    CHECK(h.scene.height > 300.0);
    const auto setup = h.ui.window_rect(app::kEntrySetupWindow);
    const auto profile = h.ui.window_rect(app::kEntryProfileWindow);
    const auto map = h.ui.window_rect(app::kGroundTrackWindow);
    REQUIRE(setup.has_value());
    REQUIRE(profile.has_value());
    REQUIRE(map.has_value());
    CHECK(setup->x < h.scene.x);
    CHECK(profile->x >= h.scene.x + h.scene.width - 1.0);
    CHECK(map->y >= h.scene.y + h.scene.height - 1.0);
    // Nothing to show before a run: an empty view.
    CHECK(h.workspace.view_state().track.empty());
}

TEST_CASE("Entry workspace runs an entry on its thread and hands the view a track",
          "[app][ui][entry]") {
    Harness h(scratch("run"));
    REQUIRE(h.run_to_end());
    const std::shared_ptr<const sim::EntryTrace> trace = h.workspace.shown_trace();
    REQUIRE(trace != nullptr);
    CHECK(trace->summary.outcome == sim::EntryOutcome::landed);
    CHECK_FALSE(h.workspace.shows_record());
    const app::EntryViewState state = h.workspace.view_state();
    CHECK(state.track.size() > 10);
    CHECK(state.options.range.min == 0.0);
    CHECK(state.options.range.max >= trace->summary.peak_g);
    REQUIRE(state.options.markers.size() == 2); // peak and cursor
    // The cursor defaults to the peak g-load; a pinned time moves it.
    CHECK(h.workspace.cursor_time() == trace->summary.peak_g_time_s);
    h.workspace.set_cursor_time(10.0);
    CHECK(h.workspace.cursor_time() == 10.0);
    // The camera frames the track with the turntable.
    const render::Camera& camera = h.workspace.camera(h.scene, 1600, 900);
    CHECK(camera.mode_kind() == render::CameraModeKind::turntable);

    // Heat-flux colouring swaps the range to kW/m^2.
    h.workspace.layers().coloring = app::EntryColoring::heat_flux;
    const app::EntryViewState heat = h.workspace.view_state();
    CHECK(heat.options.range.max >= trace->summary.peak_heat_flux_w_m2 * 1.0e-3);
    // ... and the ribbon's values: the viewport uploads the track only when
    // the revision changes, so the same run in another quantity needs a new
    // revision.
    CHECK(heat.revision != state.revision);
    REQUIRE(heat.track.size() == state.track.size());
    bool values_differ = false;
    for (std::size_t i = 0; i < heat.track.size(); ++i) {
        values_differ = values_differ || heat.track[i].value != state.track[i].value;
    }
    CHECK(values_differ);
    // Back to g-load: the original upload again.
    h.workspace.layers().coloring = app::EntryColoring::g_load;
    CHECK(h.workspace.view_state().revision == state.revision);
}

TEST_CASE("Entry workspace pauses and stops its run", "[app][ui][entry]") {
    Harness h(scratch("pause"), 30.0); // slow enough to catch it running
    sim::EntrySetup setup = h.workspace.setup();
    setup.max_time_s = 3'600.0;
    h.workspace.set_setup(setup);
    REQUIRE(h.workspace.start_run());
    REQUIRE(h.frames_until(
        [&] { return h.workspace.progress().status.state == sim::EntryRunState::running; }));
    h.workspace.pause(true);
    REQUIRE(h.frames_until(
        [&] { return h.workspace.progress().status.state == sim::EntryRunState::paused; }));
    h.workspace.stop();
    REQUIRE(h.frames_until(
        [&] { return h.workspace.progress().status.state == sim::EntryRunState::stopped; }));
    // What was flown before the stop stays shown, without an outcome.
    const std::shared_ptr<const sim::EntryTrace> trace = h.workspace.shown_trace();
    REQUIRE(trace != nullptr);
    CHECK(trace->summary.outcome == sim::EntryOutcome::none);
}

TEST_CASE("A paused run stays on its own planet when the planet input changes",
          "[app][ui][entry]") {
    Harness h(scratch("planet"), 30.0);
    sim::EntrySetup setup = h.workspace.setup();
    setup.planet = sim::EntryPlanet::earth;
    setup.max_time_s = 3'600.0;
    h.workspace.set_setup(setup);
    REQUIRE(h.workspace.start_run());
    REQUIRE(h.frames_until([&] {
        const std::shared_ptr<const sim::EntryTrace> trace = h.workspace.shown_trace();
        return trace != nullptr && trace->series.size() > 2;
    }));
    h.workspace.pause(true);
    REQUIRE(h.frames_until(
        [&] { return h.workspace.progress().status.state == sim::EntryRunState::paused; }));
    const app::EntryViewState earth = h.workspace.view_state();
    sim::EntrySetup mars = setup;
    mars.planet = sim::EntryPlanet::mars;
    h.workspace.set_setup(mars);
    // The paused run is still shown, and still drawn on the Earth it flew around.
    const app::EntryViewState state = h.workspace.view_state();
    REQUIRE(state.trace != nullptr);
    CHECK(state.planet == sim::EntryPlanet::earth);
    CHECK(state.globe.radius_m == sim::planet_constants(sim::EntryPlanet::earth).radius_m);
    CHECK(state.globe.atmosphere_top_m == earth.globe.atmosphere_top_m);
    CHECK(state.options.style == render::EntryPlanetStyle::earth);
    CHECK(state.revision == earth.revision);
    h.workspace.stop();
    REQUIRE(h.frames_until(
        [&] { return h.workspace.progress().status.state == sim::EntryRunState::stopped; }));
}

TEST_CASE("Entry results follow the inputs they were made with", "[app][ui][entry]") {
    Harness h(scratch("stale"));
    REQUIRE(h.run_to_end());
    REQUIRE(h.workspace.shown_trace() != nullptr);
    const sim::EntrySetup original = h.workspace.setup();
    sim::EntrySetup changed = original;
    changed.mass_kg *= 2.0;
    h.workspace.set_setup(changed);
    CHECK(h.workspace.shown_trace() == nullptr);
    CHECK(h.workspace.view_state().track.empty());
    // A different name alone keeps the run.
    sim::EntrySetup renamed = original;
    renamed.name = "renamed";
    h.workspace.set_setup(renamed);
    CHECK(h.workspace.shown_trace() != nullptr);
}

TEST_CASE("Entry experiments save, load and compare through the workspace", "[app][ui][entry]") {
    const std::filesystem::path dir = scratch("experiments");
    Harness h(dir);
    REQUIRE(h.run_to_end());
    const std::optional<std::filesystem::path> saved = h.workspace.save();
    REQUIRE(saved.has_value());

    sim::EntrySetup steeper = h.workspace.setup();
    steeper.name = "=steeper";
    steeper.flight_path_angle_deg -= 1.0;
    h.workspace.set_setup(steeper);
    REQUIRE(h.run_to_end());
    REQUIRE(h.workspace.save().has_value());
    // Inputs without a finished run of them are saved without a result.
    sim::EntrySetup unrun = steeper;
    unrun.name = "inputs only";
    unrun.mass_kg += 100.0;
    h.workspace.set_setup(unrun);
    REQUIRE(h.workspace.save().has_value());

    h.workspace.refresh_experiments();
    const sim::EntryListing& listing = h.workspace.experiments();
    REQUIRE(listing.entries.size() == 3);
    CHECK(listing.problems.empty());
    std::size_t with_result = 0;
    for (const sim::StoredEntry& e : listing.entries) {
        with_result += e.result ? 1U : 0U;
    }
    CHECK(with_result == 2);

    const std::optional<std::filesystem::path> csv = h.workspace.export_comparison();
    REQUIRE(csv.has_value());
    std::ifstream in(csv.value_or(std::filesystem::path{}), std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    CHECK(text.find("\"'=steeper\"") != std::string::npos);
    CHECK(text.find(",landed,") != std::string::npos);

    // Loading shows the recorded run without running again.
    const sim::StoredEntry first = listing.entries.front();
    h.workspace.open(first);
    REQUIRE(h.workspace.shown_trace() != nullptr);
    CHECK(h.workspace.shows_record());
    CHECK(h.workspace.setup() == first.setup);
}

TEST_CASE("A late Cd sampling result does not overwrite other inputs", "[app][ui][entry]") {
    // Declared before the workspace: the sampler thread reads it until the
    // workspace (and its task) is gone.
    std::atomic<bool> release{false};
    const std::vector<sim::MachCd> sampled{{5.0, 1.55}, {25.0, 1.62}};
    app::EntryWorkspaceOptions o = Harness::options(scratch("cd_late"), 0.0);
    o.cd_sampler = [&release,
                    sampled](const sim::CdSampling&) -> core::Result<sim::CdSamples, std::string> {
        while (!release.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        sim::CdSamples samples;
        samples.table = sampled;
        samples.label = "late sphere";
        return samples;
    };
    Harness h(o);
    (void)h.frame();
    h.workspace.sample_cd(sim::CdBody::sphere);
    CHECK(h.workspace.sampling_cd());
    // Another experiment is opened while the sampling still runs.
    sim::StoredEntry other;
    other.setup = sim::entry_presets().back().setup; // Mars, its own table
    h.workspace.open(other);
    release.store(true);
    REQUIRE(h.frames_until([&] { return !h.workspace.sampling_cd(); }));
    CHECK(h.workspace.setup().cd_table == other.setup.cd_table);
    CHECK(h.workspace.setup().cd_source == sim::CdSource::table);
    CHECK(h.workspace.setup() == other.setup);

    // A sampling for the inputs still loaded lands in them.
    h.workspace.sample_cd(sim::CdBody::sphere);
    REQUIRE(h.frames_until([&] { return !h.workspace.sampling_cd(); }));
    CHECK(h.workspace.setup().cd_table == sampled);
    CHECK(h.workspace.setup().cd_source == sim::CdSource::aero_solver);
}

TEST_CASE("An invalid entry setup is refused before it reaches the solver", "[app][ui][entry]") {
    Harness h(scratch("invalid"));
    sim::EntrySetup setup = h.workspace.setup();
    setup.reference_area_m2 = -1.0;
    h.workspace.set_setup(setup);
    CHECK_FALSE(h.workspace.start_run());
    CHECK(h.workspace.notice().find("Reference area") != std::string::npos);
    (void)h.frame(); // the panel lists the problem without crashing
    CHECK(h.workspace.progress().status.state == sim::EntryRunState::idle);
}
