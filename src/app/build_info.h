#pragma once

#include <array>

#include "orrery/aero/module.h"
#include "orrery/atmo/module.h"
#include "orrery/core/module.h"
#include "orrery/geom/module.h"
#include "orrery/render/module.h"
#include "orrery/scene/module.h"

namespace orrery::app {

inline constexpr bool kHeadless =
#if defined(ORRERY_HEADLESS)
    true;
#else
    false;
#endif

// All libraries linked into the application, in dependency order.
[[nodiscard]] inline std::array<core::ModuleInfo, 6> linked_modules() noexcept {
    return {core::module_info(), geom::module_info(),  atmo::module_info(),
            aero::module_info(), scene::module_info(), render::module_info()};
}

} // namespace orrery::app
