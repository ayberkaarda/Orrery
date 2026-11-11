#pragma once

#include "orrery/core/module.h"

// Rendering and the GPU compute backend implementation.
namespace orrery::render {

[[nodiscard]] core::ModuleInfo module_info() noexcept;

} // namespace orrery::render
