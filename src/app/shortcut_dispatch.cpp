#include "orrery/app/shortcut_dispatch.h"

#include <algorithm>
#include <vector>

#include "orrery/app/aero_workspace.h"
#include "orrery/app/entry_workspace.h"
#include "orrery/app/shortcut_input.h"
#include "orrery/app/space_experiment_panel.h"
#include "orrery/render/ui_app_bar.h"
#include "orrery/sim/physics_thread.h"

namespace orrery::app {

namespace {

void close_something(ShortcutTargets& targets) {
    if (targets.help_open) {
        targets.help_open = false;
    } else if (targets.workspace == Workspace::space) {
        targets.selected.reset();
    } else if (targets.workspace == Workspace::entry) {
        targets.entry.set_cursor_time(std::nullopt);
    }
}

void step_warp(ShortcutTargets& targets, int direction) {
    targets.physics.send(
        sim::SetTimeWarp{render::step_time_warp(targets.status.time_warp, direction)});
}

} // namespace

std::vector<ShortcutAction> frame_shortcuts(const ShortcutTable& table, Workspace workspace) {
    return match_shortcuts(table, read_key_frame(), workspace);
}

void apply_shortcuts(std::span<const ShortcutAction> actions, ShortcutTargets& targets) {
    for (const ShortcutAction action : actions) {
        switch (action) {
            case ShortcutAction::workspace_space:
                targets.workspace = Workspace::space;
                break;
            case ShortcutAction::workspace_aero:
                targets.workspace = Workspace::aero;
                break;
            case ShortcutAction::workspace_entry:
                targets.workspace = Workspace::entry;
                break;
            case ShortcutAction::help:
                targets.help_open = !targets.help_open;
                break;
            case ShortcutAction::close:
                close_something(targets);
                break;
            case ShortcutAction::save:
                save_workspace_experiment(targets);
                break;
            case ShortcutAction::warp_down:
                step_warp(targets, -1);
                break;
            case ShortcutAction::warp_up:
                step_warp(targets, +1);
                break;
            case ShortcutAction::reset:
                targets.physics.send(sim::Reset{});
                break;
            case ShortcutAction::open:
            case ShortcutAction::toggle_pause:
            case ShortcutAction::single_step:
            case ShortcutAction::reset_view:
                break; // after the panels, and through the scene input
        }
    }
}

void apply_late_shortcuts(std::span<const ShortcutAction> actions, ShortcutTargets& targets) {
    if (std::find(actions.begin(), actions.end(), ShortcutAction::open) == actions.end()) {
        return;
    }
    if (targets.workspace == Workspace::aero) {
        focus_window(kExperimentsWindow);
    } else if (targets.workspace == Workspace::entry) {
        targets.entry.refresh_experiments();
        focus_window(kEntryExperimentsWindow);
    } else {
        targets.space_panel.refresh();
        focus_window(kSpaceExperimentWindow);
    }
}

void save_workspace_experiment(ShortcutTargets& targets) {
    if (targets.workspace == Workspace::aero) {
        (void)targets.aero.save();
    } else if (targets.workspace == Workspace::entry) {
        (void)targets.entry.save();
    } else {
        (void)targets.space_panel.save(targets.space_scenario, targets.status,
                                       targets.sim_time_days);
    }
}

bool apply_space_scene_input(const render::SceneInput& input, ShortcutTargets& targets) {
    if (input.toggle_pause) {
        targets.physics.send(sim::SetPaused{!targets.status.paused});
    }
    if (input.single_step) {
        targets.physics.send(sim::SingleStep{});
    }
    return input.reset_view;
}

} // namespace orrery::app
