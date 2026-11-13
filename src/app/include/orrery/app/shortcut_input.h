#pragma once

#include <span>
#include <string_view>

#include "orrery/app/shortcuts.h"
#include "orrery/render/ui_app_bar.h"

// The Dear ImGui side of the keyboard shortcuts (shortcuts.h): reading the
// keys of a frame, the help overlay, and the scene input the shortcuts
// drive. Call between UiContext::begin_frame and UiContext::end_frame.
namespace orrery::app {

inline constexpr std::string_view kShortcutHelpWindow = "Keyboard Shortcuts";

// The keys pressed, repeated and typed in the current frame; text_input is
// set while a text field has the keyboard.
[[nodiscard]] KeyFrame read_key_frame();

// `pointer` (render::read_scene_input: drag and wheel) with its key fields
// replaced by the shortcut actions of this frame (toggle_pause,
// single_step, reset_view), so that the configured keys drive every
// workspace's scene.
[[nodiscard]] render::SceneInput scene_input_from(render::SceneInput pointer,
                                                  std::span<const ShortcutAction> actions);

// The help overlay: every action with its chords, grouped by where it
// applies, the ones of `workspace` first. Drawn while `open`; its close
// button clears `open`. `config_path` is shown as the place to change them.
void draw_shortcut_help(const ShortcutTable& table, Workspace workspace, bool& open,
                        std::string_view config_path);

// Brings a panel to the front (its dock tab is selected); unknown names are
// ignored.
void focus_window(std::string_view name);

} // namespace orrery::app
