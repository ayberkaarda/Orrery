#pragma once

#include <cstdint>

namespace orrery::app {

// The workspaces of the window (docs/app.md, "Workspaces").
enum class Workspace : std::uint8_t { space, aero, entry };

} // namespace orrery::app
