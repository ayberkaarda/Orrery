#include "orrery/scene/module.h"

#include <array>
#include <string_view>

namespace orrery::scene {

namespace {
constexpr std::array<std::string_view, 4> kDependencies{"core", "geom", "aero", "atmo"};
} // namespace

core::ModuleInfo module_info() noexcept {
    return core::ModuleInfo{"scene", kDependencies};
}

} // namespace orrery::scene
