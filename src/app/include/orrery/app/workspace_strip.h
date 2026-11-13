#pragma once

#include <optional>

#include "orrery/app/config.h"

// Workspace switcher of the window (spec 8: Space | Aero | Entry as top
// tabs; docs/app.md "Workspaces"): a slim strip under the main bar, in the
// look of the bar's workspace tab. Call after the main bar and before the dock space, between
// UiContext::begin_frame and end_frame.
namespace orrery::app {

// Returns the workspace the user picked this frame (never a disabled one).
[[nodiscard]] std::optional<Workspace> draw_workspace_strip(Workspace current);

// Keys (Ctrl+1 / Ctrl+2 / Ctrl+3 by default) are in the shortcut table,
// shortcuts.h.

} // namespace orrery::app
