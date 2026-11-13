#pragma once

#include <optional>

#include "orrery/app/config.h"

// Workspace switcher of the window (spec 8: Space | Aero | Entry as top
// tabs; docs/app.md "Workspaces"): a slim strip under the main bar, in the
// look of the bar's workspace tab. Entry is shown disabled until its phase.
// Call after the main bar and before the dock space, between
// UiContext::begin_frame and end_frame.
namespace orrery::app {

// Returns the workspace the user picked this frame (never a disabled one).
[[nodiscard]] std::optional<Workspace> draw_workspace_strip(Workspace current);

// Ctrl+1 / Ctrl+2 switch between the Space and Aero workspaces while the UI
// does not want the keyboard.
[[nodiscard]] std::optional<Workspace> read_workspace_shortcut();

} // namespace orrery::app
