#pragma once

#include <filesystem>
#include <optional>
#include <span>
#include <vector>

#include "orrery/app/shortcuts.h"
#include "orrery/app/workspace.h"
#include "orrery/core/body.h"
#include "orrery/render/ui_app_bar.h"
#include "orrery/sim/simulation.h"

namespace orrery::sim {
class PhysicsThread;
} // namespace orrery::sim

// What the window does with the shortcut actions of a frame (docs/app.md,
// "Keyboard shortcuts"). The window and the end-to-end tests
// (test_workspace_journeys.cpp) both read the keys through frame_shortcuts
// and carry them out through these functions, so the tests exercise the
// window's own key handling under a headless UI context.
namespace orrery::app {

class AeroWorkspace;
class EntryWorkspace;
class SpaceExperimentPanel;

// What the shortcuts act on: the window's state, borrowed for one frame.
struct ShortcutTargets {
    Workspace& workspace;
    bool& help_open;
    std::optional<core::BodyId>& selected; // the Space workspace's selection
    AeroWorkspace& aero;
    EntryWorkspace& entry;
    SpaceExperimentPanel& space_panel;
    sim::PhysicsThread& physics;
    const sim::SimStatus& status; // the Space simulation's latest status
    // Scenario file of the running space system, and its simulated time.
    std::filesystem::path space_scenario;
    double sim_time_days = 0.0;
};

// The shortcut actions of this frame in `workspace`: read_key_frame matched
// against `table`. Call between UiContext::begin_frame and end_frame.
[[nodiscard]] std::vector<ShortcutAction> frame_shortcuts(const ShortcutTable& table,
                                                          Workspace workspace);

// Carries out the actions that act before the panels are drawn: switching
// workspaces, the help overlay, close, save, the warp keys and reset.
// toggle_pause, single_step and reset_view reach the scenes through
// scene_input_from; open waits for apply_late_shortcuts.
void apply_shortcuts(std::span<const ShortcutAction> actions, ShortcutTargets& targets);

// Carries out the actions that need this frame's panels: open brings the
// workspace's experiments panel to the front (re-read first).
void apply_late_shortcuts(std::span<const ShortcutAction> actions, ShortcutTargets& targets);

// Saves the current workspace's experiment (the save shortcut and the Space
// Experiment panel's Save button).
void save_workspace_experiment(ShortcutTargets& targets);

// The Space workspace's scene keys: run or pause and single step go to the
// physics thread. True when the view is to be framed again (reset_view).
bool apply_space_scene_input(const render::SceneInput& input, ShortcutTargets& targets);

} // namespace orrery::app
