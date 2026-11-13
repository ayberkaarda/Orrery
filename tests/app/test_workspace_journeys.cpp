#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "app_test_support.h"
#include "orrery/app/aero_workspace.h"
#include "orrery/app/entry_workspace.h"
#include "orrery/app/shortcut_input.h"
#include "orrery/app/shortcuts.h"
#include "orrery/app/space_experiment_panel.h"
#include "orrery/app/workspace_strip.h"
#include "orrery/render/ui_app_bar.h"
#include "orrery/render/ui_context.h"
#include "orrery/render/ui_panels.h"
#include "orrery/sim/aero_comparison.h"
#include "orrery/sim/entry_experiments.h"
#include "orrery/sim/physics_thread.h"
#include "orrery/sim/space_experiment.h"

// End to end, one journey per workspace (docs/app.md, "Workspaces"): from a
// fresh start to a saved result that loads back, driven the way the window
// drives it - headless UI frames, keys turned into actions by the default
// shortcut table (Space runs, Ctrl+S saves, "]" raises the warp) - without
// a window or OpenGL.

using namespace orrery;
using namespace orrery::app;

namespace {

std::filesystem::path scratch(const char* name) {
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / "orrery_test_journeys" / name;
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

render::UiContext make_ui() {
    auto created = render::UiContext::create_headless();
    REQUIRE(created.has_value());
    return std::move(created).value();
}

render::UiFrameInput frame_input() {
    render::UiFrameInput input;
    input.width = 1600;
    input.height = 900;
    return input;
}

// The actions the default table gives for one key chord or typed text.
std::vector<ShortcutAction> keys(std::string_view chord, Workspace workspace) {
    KeyFrame frame;
    auto parsed = parse_key_chord(chord);
    REQUIRE(parsed.has_value());
    if (parsed.value().character != '\0') {
        frame.typed.push_back(parsed.value().character);
    } else {
        frame.pressed.push_back(parsed.value());
    }
    return match_shortcuts(default_shortcuts(), frame, workspace);
}

bool has(std::span<const ShortcutAction> actions, ShortcutAction action) {
    return std::find(actions.begin(), actions.end(), action) != actions.end();
}

} // namespace

TEST_CASE("Space journey: run, raise the warp, save and open the experiment again",
          "[app][journey][space]") {
    const std::filesystem::path dir = scratch("space");
    auto experiment =
        sim::load_space_experiment(app_test::experiment_dir() / "sun_earth_moon.json");
    REQUIRE(experiment.has_value());
    sim::SimulationConfig config = experiment.value().config;
    config.paused = true; // a fresh start: nothing runs until Space
    config.warp_budget = sim::WarpBudget{};
    auto simulation = sim::Simulation::create(experiment.value().scene, config);
    REQUIRE(simulation.has_value());
    sim::PhysicsThread physics(std::move(simulation).value());
    render::UiContext ui = make_ui();
    SpaceExperimentPanel panel(dir, app_test::experiment_dir());
    CHECK(panel.saved().entries.empty());
    // The bundled list holds the space experiments only.
    CHECK(panel.bundled().entries.size() == 5);

    sim::PublishedFrame latest = physics.latest();
    const auto frame = [&](std::span<const ShortcutAction> actions) {
        physics.wake();
        if (physics.fetch()) {
            latest = physics.latest();
        }
        ui.begin_frame(frame_input());
        render::AppBarData bar;
        bar.time_warp = latest.status.time_warp;
        bar.paused = latest.status.paused;
        const render::AppBarIntent intent = render::draw_app_bar(bar);
        CHECK_FALSE(intent.set_paused.has_value());
        (void)draw_workspace_strip(Workspace::space);
        (void)ui.dockspace();
        const render::SceneInput input = scene_input_from(render::SceneInput{}, actions);
        if (input.toggle_pause) {
            physics.send(sim::SetPaused{!latest.status.paused});
        }
        for (const ShortcutAction action : actions) {
            if (action == ShortcutAction::warp_up) {
                physics.send(sim::SetTimeWarp{render::step_time_warp(latest.status.time_warp, +1)});
            }
        }
        render::SystemMonitorData monitor;
        monitor.integrator = latest.status.integrator;
        (void)render::draw_system_monitor(monitor);
        SpaceRunView run;
        run.scenario = latest.status.scenario;
        const SpaceExperimentIntent panel_intent = panel.draw(run);
        if (has(actions, ShortcutAction::save) || panel_intent.save) {
            REQUIRE(panel
                        .save(experiment.value().scenario_path, latest.status,
                              latest.snapshot->time().value())
                        .has_value());
        }
        ui.end_frame();
    };
    const auto frames_until = [&](const std::function<bool()>& done) {
        return app_test::wait_until([&] {
            frame({});
            return done();
        });
    };

    frame({});
    CHECK(ui.window_docked(kSpaceExperimentWindow));
    REQUIRE(latest.status.paused);
    frame(keys("Space", Workspace::space));
    REQUIRE(frames_until([&] { return !latest.status.paused && latest.status.steps > 20; }));
    const std::int64_t warp = latest.status.time_warp;
    frame(keys("]", Workspace::space));
    REQUIRE(frames_until([&] { return latest.status.time_warp > warp; }));
    const std::int64_t raised = latest.status.time_warp;
    CHECK(raised == render::step_time_warp(warp, +1));

    panel.set_name("Sun Earth Moon run");
    frame(keys("Ctrl+S", Workspace::space));
    REQUIRE(panel.saved().entries.size() == 1);
    const std::filesystem::path saved = panel.saved().entries.front().path;
    CHECK(saved.filename() == "sun_earth_moon_run.json");
    CHECK(panel.notice() == "Saved sun_earth_moon_run.json");

    // Open it in a fresh simulation: the raised warp and the integrator
    // come back, and the result holds the run so far.
    auto reopened = sim::load_space_experiment(saved);
    REQUIRE(reopened.has_value());
    CHECK(reopened.value().config.time_warp == raised);
    CHECK(reopened.value().config.integrator == experiment.value().config.integrator);
    physics.send(sim::LoadExperiment{saved});
    REQUIRE(frames_until([&] { return latest.status.experiment_file == saved; }));
    CHECK(latest.status.time_warp == raised);
    CHECK(latest.status.message.empty());
}

TEST_CASE("Aero journey: run the default section, save, and open the record",
          "[app][journey][aero]") {
    const std::filesystem::path dir = scratch("aero");
    render::UiContext ui = make_ui();
    AeroWorkspaceOptions options;
    options.experiment_dir = dir;
    AeroWorkspace workspace(options);
    const auto frame = [&](std::span<const ShortcutAction> actions) {
        ui.begin_frame(frame_input());
        workspace.update(1.0 / 60.0);
        workspace.draw_bar(60.0);
        (void)draw_workspace_strip(Workspace::aero);
        const render::UiRect scene = workspace.draw_panels();
        workspace.handle_scene_input(scene_input_from(render::SceneInput{}, actions), 1.0 / 60.0);
        if (has(actions, ShortcutAction::save)) {
            REQUIRE(workspace.save().has_value());
        }
        ui.end_frame();
        (void)workspace.camera(scene, 1600, 900);
    };
    const auto frames_until = [&](const std::function<bool()>& done) {
        return app_test::wait_until([&] {
            frame({});
            return done();
        });
    };

    REQUIRE(frames_until([&] { return workspace.geometry_ready(); }));
    CHECK_FALSE(workspace.shown_result().has_value());
    frame(keys("Space", Workspace::aero));
    REQUIRE(frames_until(
        [&] { return workspace.progress().status.state == sim::AeroRunState::finished; }));
    const auto shown = workspace.shown_result();
    REQUIRE(shown.has_value());
    CHECK(shown->cl > 0.3); // NACA 0012 at 4 degrees

    frame(keys("Ctrl+S", Workspace::aero));
    const sim::ExperimentListing listing = sim::list_aero_experiments(dir);
    REQUIRE(listing.experiments.size() == 1);
    CHECK(listing.problems.empty());
    const scene::AeroExperiment& record = listing.experiments.front().experiment;
    REQUIRE(record.result.has_value());
    CHECK(record.result->cl == shown->cl);

    // A fresh workspace opens the record and shows its result unchanged.
    AeroWorkspace second(options);
    second.open(record, dir);
    const auto recorded = second.shown_result();
    REQUIRE(recorded.has_value());
    CHECK(recorded->recorded);
    CHECK(recorded->cl == shown->cl);
}

TEST_CASE("Entry journey: fly the Apollo-like entry, save, and load the record",
          "[app][journey][entry]") {
    const std::filesystem::path dir = scratch("entry");
    render::UiContext ui = make_ui();
    EntryWorkspaceOptions options;
    options.experiment_dir = dir;
    options.pace = 0.0; // as fast as possible
    EntryWorkspace workspace(options);
    const auto frame = [&](std::span<const ShortcutAction> actions) {
        ui.begin_frame(frame_input());
        workspace.update(1.0 / 60.0);
        workspace.draw_bar(60.0);
        (void)draw_workspace_strip(Workspace::entry);
        const render::UiRect scene = workspace.draw_panels();
        workspace.handle_scene_input(scene_input_from(render::SceneInput{}, actions), 1.0 / 60.0);
        if (has(actions, ShortcutAction::save)) {
            REQUIRE(workspace.save().has_value());
        }
        if (has(actions, ShortcutAction::close)) {
            workspace.set_cursor_time(std::nullopt);
        }
        ui.end_frame();
        (void)workspace.camera(scene, 1600, 900);
    };
    const auto frames_until = [&](const std::function<bool()>& done) {
        return app_test::wait_until([&] {
            frame({});
            return done();
        });
    };

    const std::vector<sim::EntryPreset> presets = sim::entry_presets();
    REQUIRE_FALSE(presets.empty());
    workspace.set_setup(presets.front().setup);
    frame({});
    CHECK(workspace.shown_trace() == nullptr);
    frame(keys("Space", Workspace::entry));
    REQUIRE(frames_until(
        [&] { return workspace.progress().status.state == sim::EntryRunState::finished; }));
    const auto trace = workspace.shown_trace();
    REQUIRE(trace != nullptr);
    CHECK(trace->summary.outcome == sim::EntryOutcome::landed);
    CHECK(trace->summary.peak_g > 6.0);
    CHECK(trace->summary.peak_g < 8.0);

    workspace.set_cursor_time(10.0);
    frame(keys("Esc", Workspace::entry));
    frame(keys("Ctrl+S", Workspace::entry));
    workspace.refresh_experiments();
    const sim::EntryListing& listing = workspace.experiments();
    REQUIRE(listing.entries.size() == 1);
    CHECK(listing.problems.empty());
    REQUIRE(listing.entries.front().result != nullptr);
    CHECK(listing.entries.front().result->summary.peak_g == trace->summary.peak_g);

    // A fresh workspace loads the record and shows it as recorded.
    EntryWorkspace second(options);
    second.open(listing.entries.front());
    CHECK(second.shows_record());
    REQUIRE(second.shown_trace() != nullptr);
    CHECK(second.shown_trace()->summary.outcome == sim::EntryOutcome::landed);
}
