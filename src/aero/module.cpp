#include "orrery/aero/module.h"

#include <array>
#include <string_view>

namespace orrery::aero {

namespace {
constexpr std::array<std::string_view, 2> kDependencies{"core", "geom"};
} // namespace

core::ModuleInfo module_info() noexcept {
    return core::ModuleInfo{"aero", kDependencies};
}

} // namespace orrery::aero
