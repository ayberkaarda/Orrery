#pragma once

#include "orrery/core/module.h"

// Geometry pipeline: primitives, import, repair, voxelization.
namespace orrery::geom {

[[nodiscard]] core::ModuleInfo module_info() noexcept;

} // namespace orrery::geom
