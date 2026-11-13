#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <imgui.h>

#include "app_test_support.h"
#include "key_events.h"
#include "orrery/app/aero_workspace.h"
#include "orrery/app/entry_workspace.h"
#include "orrery/app/shortcut_dispatch.h"
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

// End to end, one journey per workspace (docs/app.md, "End to end"): from a
// fresh start to a saved result that loads back, without a window or OpenGL.
// Keys are Dear ImGui key and character events, queued as the GLFW backend
// queues them, read by frame_shortcuts (read_key_frame) and carried out by
// the window's own dispatch (shortcut_dispatch.h), with the default table.

using namespace orrery;
using namespace orrery::app;

namespace {

constexpr double kFrameSeconds = 1.0 / 60.0;
constexpr int kWidth = 1600;
constexpr int kHeight = 900;

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

sim::SpaceExperiment space_experiment() {
    auto experiment =
        sim::load_space_experiment(app_test::experiment_dir() / "sun_earth_moon.json");
    REQUIRE(experiment.has_value());
    return std::move(experiment).value();
}

// A paused simulation of `experiment` with the window's warp budget.
sim::Simulation paused_simulation(const sim::SpaceExperiment& experiment) {
    sim::SimulationConfig config = experiment.config;
    config.paused = true; // a fresh start: nothing runs until Space
    config.warp_budget = sim::WarpBudget{};
    auto simulation = sim::Simulation::create(experiment.scene, config);
    REQUIRE(simulation.has_value());
    return std::move(simulation).value();
}

AeroWorkspaceOptions aero_options(const std::filesystem::path& dir) {
    AeroWorkspaceOptions options;
    options.experiment_dir = dir;
    return options;
}

EntryWorkspaceOptions entry_options(const std::filesystem::path& dir) {
    EntryWorkspaceOptions options;
    options.experiment_dir = dir;
    options.pace = 0.0; // as fast as possible
    return options;
}

// Keys of the default table, as a layout types them.
const app_test::KeyStroke kSpace{ImGuiKey_Space, false, false, false, ' '};
const app_test::KeyStroke kEscape{ImGuiKey_Escape};
const app_test::KeyStroke kCtrlS{ImGuiKey_S, true};
const app_test::KeyStroke kCtrl1{ImGuiKey_1, true};
const app_test::KeyStroke kCtrl2{ImGuiKey_2, true};
const app_test::KeyStroke kCtrl3{ImGuiKey_3, true};
const app_test::KeyStroke kQuestion{ImGuiKey_Slash, false, true, false, '?'};
const app_test::KeyStroke kCloseBracket{ImGuiKey_RightBracket, false, false, false, ']'};

// The window's frame without the window: the UI calls of WindowApp::build_ui
// in the same order, every workspace and the physics thread alive, the keys
// read and dispatched by the window's functions.
class Shell {
public:
    Shell(const std::filesystem::path& dir, Workspace start)
        : experiment_(space_experiment()), physics_(paused_simulation(experiment_)), ui_(make_ui()),
          aero_(aero_options(dir)), entry_(entry_options(dir)),
          panel_(dir, app_test::experiment_dir()), workspace_(start), latest_(physics_.latest()) {}

    // Queues `events` (key_events.h), then runs one frame.
    void frame(const std::function<void()>& events) {
        physics_.wake();
        if (physics_.fetch()) {
            latest_ = physics_.latest();
        }
        events();
        render::UiFrameInput input;
        input.width = kWidth;
        input.height = kHeight;
        input.delta_time = kFrameSeconds;
        ui_.begin_frame(input);
        draw_bar();
        if (const auto picked = draw_workspace_strip(workspace_)) {
            workspace_ = *picked;
        }
        const std::vector<ShortcutAction> actions = frame_shortcuts(table_, workspace_);
        ShortcutTargets targets = shortcut_targets();
        apply_shortcuts(actions, targets);
        draw_panels(scene_input_from(render::SceneInput{}, actions), targets);
        apply_late_shortcuts(actions, targets);
        draw_shortcut_help(table_, workspace_, help_open_, "config.toml");
        ui_.end_frame();
        if (workspace_ == Workspace::aero) {
            (void)aero_.camera(scene_, kWidth, kHeight);
        } else if (workspace_ == Workspace::entry) {
            (void)entry_.camera(scene_, kWidth, kHeight);
        }
    }

    void idle() {
        frame([] {});
    }

    // A key pressed in one frame and released in the next.
    void press(const app_test::KeyStroke& stroke) {
        frame([&] { app_test::key_down(stroke); });
        frame([&] { app_test::key_up(stroke); });
    }

    bool frames_until(const std::function<bool()>& done) {
        return app_test::wait_until([&] {
            idle();
            return done();
        });
    }

    // Shows a text field of its own (focused when it appears), or hides it.
    void show_text_field(bool shown) {
        focus_field_ = shown && !field_shown_;
        field_shown_ = shown;
    }

    [[nodiscard]] Workspace workspace() const noexcept { return workspace_; }
    [[nodiscard]] bool help_open() const noexcept { return help_open_; }
    [[nodiscard]] const sim::PublishedFrame& latest() const noexcept { return latest_; }
    [[nodiscard]] const sim::SpaceExperiment& experiment() const noexcept { return experiment_; }
    sim::PhysicsThread& physics() noexcept { return physics_; }
    render::UiContext& ui() noexcept { return ui_; }
    AeroWorkspace& aero() noexcept { return aero_; }
    EntryWorkspace& entry() noexcept { return entry_; }
    SpaceExperimentPanel& panel() noexcept { return panel_; }

private:
    ShortcutTargets shortcut_targets() {
        const sim::SimStatus& status = latest_.status;
        return ShortcutTargets{workspace_,
                               help_open_,
                               selected_,
                               aero_,
                               entry_,
                               panel_,
                               physics_,
                               status,
                               running_scenario(status, experiment_.scenario_path),
                               latest_.snapshot ? latest_.snapshot->time().value() : 0.0};
    }

    void draw_bar() {
        if (workspace_ == Workspace::aero) {
            aero_.update(kFrameSeconds);
            aero_.draw_bar(60.0);
        } else if (workspace_ == Workspace::entry) {
            entry_.update(kFrameSeconds);
            entry_.draw_bar(60.0);
        } else {
            render::AppBarData bar;
            bar.time_warp = latest_.status.time_warp;
            bar.paused = latest_.status.paused;
            const render::AppBarIntent intent = render::draw_app_bar(bar);
            CHECK_FALSE(intent.set_paused.has_value());
        }
    }

    void draw_panels(const render::SceneInput& input, ShortcutTargets& targets) {
        if (workspace_ == Workspace::aero) {
            scene_ = aero_.draw_panels();
            aero_.handle_scene_input(input, kFrameSeconds);
        } else if (workspace_ == Workspace::entry) {
            scene_ = entry_.draw_panels();
            entry_.handle_scene_input(input, kFrameSeconds);
        } else {
            scene_ = ui_.dockspace();
            (void)apply_space_scene_input(input, targets);
            render::SystemMonitorData monitor;
            monitor.integrator = latest_.status.integrator;
            (void)render::draw_system_monitor(monitor);
            SpaceRunView run;
            run.scenario = latest_.status.scenario;
            if (panel_.draw(run).save) {
                save_workspace_experiment(targets);
            }
        }
        if (field_shown_) {
            draw_text_field();
        }
    }

    void draw_text_field() {
        ImGui::Begin("Journey notes");
        if (focus_field_) {
            ImGui::SetKeyboardFocusHere();
            focus_field_ = false;
        }
        (void)ImGui::InputText("##notes", notes_.data(), notes_.size());
        ImGui::End();
    }

    sim::SpaceExperiment experiment_;
    sim::PhysicsThread physics_;
    render::UiContext ui_;
    AeroWorkspace aero_;
    EntryWorkspace entry_;
    SpaceExperimentPanel panel_;
    ShortcutTable table_ = default_shortcuts();
    Workspace workspace_;
    bool help_open_ = false;
    std::optional<core::BodyId> selected_;
    sim::PublishedFrame latest_;
    render::UiRect scene_;
    bool field_shown_ = false;
    bool focus_field_ = false;
    std::array<char, 64> notes_{};
};

} // namespace

TEST_CASE("Space journey: run, raise the warp, save and open the experiment again",
          "[app][journey][space]") {
    const std::filesystem::path dir = scratch("space");
    Shell shell(dir, Workspace::space);
    CHECK(shell.panel().saved().entries.empty());
    // The bundled list holds the space experiments only.
    CHECK(shell.panel().bundled().entries.size() == 5);

    shell.idle();
    CHECK(shell.ui().window_docked(kSpaceExperimentWindow));
    REQUIRE(shell.latest().status.paused);
    shell.press(kSpace);
    REQUIRE(shell.frames_until(
        [&] { return !shell.latest().status.paused && shell.latest().status.steps > 20; }));

    // "]" raises the warp, and again on each key repeat while held.
    const std::int64_t warp = shell.latest().status.time_warp;
    const std::int64_t once = render::step_time_warp(warp, +1);
    const std::int64_t twice = render::step_time_warp(once, +1);
    shell.frame([] { app_test::key_down(kCloseBracket); });
    REQUIRE(shell.frames_until([&] { return shell.latest().status.time_warp == once; }));
    shell.frame([] { app_test::key_repeat(kCloseBracket); });
    REQUIRE(shell.frames_until([&] { return shell.latest().status.time_warp == twice; }));
    shell.frame([] { app_test::key_up(kCloseBracket); });

    // A held "?" toggles the help once; Esc closes it.
    shell.frame([] { app_test::key_down(kQuestion); });
    for (int i = 0; i < 4; ++i) {
        shell.frame([] { app_test::key_repeat(kQuestion); });
    }
    shell.frame([] { app_test::key_up(kQuestion); });
    CHECK(shell.help_open());
    shell.press(kEscape);
    CHECK_FALSE(shell.help_open());

    // Ctrl+2, Ctrl+3 and Ctrl+1 switch workspaces.
    shell.press(kCtrl2);
    CHECK(shell.workspace() == Workspace::aero);
    shell.press(kCtrl3);
    CHECK(shell.workspace() == Workspace::entry);
    shell.press(kCtrl1);
    CHECK(shell.workspace() == Workspace::space);

    // While a text field has the keyboard, no shortcut fires.
    shell.show_text_field(true);
    shell.idle();
    shell.idle();
    shell.press(kCtrlS);
    shell.press(kQuestion);
    CHECK(shell.panel().saved().entries.empty());
    CHECK_FALSE(shell.help_open());
    shell.show_text_field(false);
    shell.idle();
    shell.idle();

    shell.panel().set_name("Sun Earth Moon run");
    shell.press(kCtrlS);
    REQUIRE(shell.panel().saved().entries.size() == 1);
    const std::filesystem::path saved = shell.panel().saved().entries.front().path;
    CHECK(saved.filename() == "sun_earth_moon_run.json");
    CHECK(shell.panel().notice() == "Saved sun_earth_moon_run.json");

    // Open it in a fresh simulation: the raised warp and the integrator
    // come back, and the result holds the run so far.
    auto reopened = sim::load_space_experiment(saved);
    REQUIRE(reopened.has_value());
    CHECK(reopened.value().config.time_warp == twice);
    CHECK(reopened.value().config.integrator == shell.experiment().config.integrator);
    shell.physics().send(sim::LoadExperiment{saved});
    REQUIRE(shell.frames_until([&] { return shell.latest().status.experiment_file == saved; }));
    CHECK(shell.latest().status.time_warp == twice);
    CHECK(shell.latest().status.message.empty());
}

TEST_CASE("The Experiment panel names the experiment that runs, or none", "[app][journey][space]") {
    const std::filesystem::path startup_file = app_test::experiment_dir() / "figure_eight.json";
    auto startup = sim::load_space_experiment(startup_file);
    REQUIRE(startup.has_value());
    auto created = sim::Simulation::create(startup.value().scene, startup.value().config);
    REQUIRE(created.has_value());
    sim::Simulation simulation = std::move(created).value();
    const auto view = [&] { return space_run_view(simulation.status(), startup_file, 0.0); };
    const auto scenario = [&] {
        return running_scenario(simulation.status(), startup.value().scenario_path);
    };
    CHECK(view().experiment == "figure_eight.json");
    CHECK(scenario() == startup.value().scenario_path);

    // A standalone scenario is not the startup experiment.
    const std::filesystem::path standalone = app_test::scenario_dir() / "sun_earth.json";
    simulation.apply(sim::LoadScenario{standalone});
    REQUIRE(simulation.status().message.empty());
    CHECK(view().experiment.empty());
    CHECK(view().scenario == simulation.status().scenario);
    CHECK(scenario() == standalone);
    simulation.apply(sim::Reset{}); // the same system again
    CHECK(view().experiment.empty());
    CHECK(scenario() == standalone);

    const std::filesystem::path opened = app_test::experiment_dir() / "sun_earth_moon.json";
    simulation.apply(sim::LoadExperiment{opened});
    REQUIRE(simulation.status().message.empty());
    CHECK(view().experiment == "sun_earth_moon.json");
    CHECK(scenario().filename() == "sun_earth_moon.json");
}

TEST_CASE("Aero journey: run the default section, save, and open the record",
          "[app][journey][aero]") {
    const std::filesystem::path dir = scratch("aero");
    Shell shell(dir, Workspace::aero);
    REQUIRE(shell.frames_until([&] { return shell.aero().geometry_ready(); }));
    CHECK_FALSE(shell.aero().shown_result().has_value());
    shell.press(kSpace);
    REQUIRE(shell.frames_until(
        [&] { return shell.aero().progress().status.state == sim::AeroRunState::finished; }));
    const auto shown = shell.aero().shown_result();
    REQUIRE(shown.has_value());
    CHECK(shown->cl > 0.3); // NACA 0012 at 4 degrees

    shell.press(kCtrlS);
    const sim::ExperimentListing listing = sim::list_aero_experiments(dir);
    REQUIRE(listing.experiments.size() == 1);
    CHECK(listing.problems.empty());
    const scene::AeroExperiment& record = listing.experiments.front().experiment;
    REQUIRE(record.result.has_value());
    CHECK(record.result->cl == shown->cl);

    // A fresh workspace opens the record and shows its result unchanged.
    AeroWorkspace second(aero_options(dir));
    second.open(record, dir);
    const auto recorded = second.shown_result();
    REQUIRE(recorded.has_value());
    CHECK(recorded->recorded);
    CHECK(recorded->cl == shown->cl);
}

TEST_CASE("Entry journey: fly the Apollo-like entry, save, and load the record",
          "[app][journey][entry]") {
    const std::filesystem::path dir = scratch("entry");
    Shell shell(dir, Workspace::entry);
    const std::vector<sim::EntryPreset> presets = sim::entry_presets();
    REQUIRE_FALSE(presets.empty());
    shell.entry().set_setup(presets.front().setup);
    shell.idle();
    CHECK(shell.entry().shown_trace() == nullptr);
    shell.press(kSpace);
    REQUIRE(shell.frames_until(
        [&] { return shell.entry().progress().status.state == sim::EntryRunState::finished; }));
    const auto trace = shell.entry().shown_trace();
    REQUIRE(trace != nullptr);
    CHECK(trace->summary.outcome == sim::EntryOutcome::landed);
    CHECK(trace->summary.peak_g > 6.0);
    CHECK(trace->summary.peak_g < 8.0);

    // Esc releases the pinned cursor: it goes back to the peak.
    REQUIRE(trace->summary.peak_g_time_s != 10.0);
    shell.entry().set_cursor_time(10.0);
    shell.idle();
    CHECK(shell.entry().cursor_time() == 10.0);
    shell.press(kEscape);
    CHECK(shell.entry().cursor_time() == trace->summary.peak_g_time_s);
    shell.press(kCtrlS);
    shell.entry().refresh_experiments();
    const sim::EntryListing& listing = shell.entry().experiments();
    REQUIRE(listing.entries.size() == 1);
    CHECK(listing.problems.empty());
    REQUIRE(listing.entries.front().result != nullptr);
    CHECK(listing.entries.front().result->summary.peak_g == trace->summary.peak_g);

    // A fresh workspace loads the record and shows it as recorded.
    EntryWorkspace second(entry_options(dir));
    second.open(listing.entries.front());
    CHECK(second.shows_record());
    REQUIRE(second.shown_trace() != nullptr);
    CHECK(second.shown_trace()->summary.outcome == sim::EntryOutcome::landed);
}
