#include "orrery/atmo/module.h"

#include <array>
#include <string_view>

namespace orrery::atmo {

namespace {
// aero: the Cd(Mach) table is sampled from the aerodynamic solvers and the
// heat flux uses their Sutton-Graves correlation.
constexpr std::array<std::string_view, 2> kDependencies{"core", "aero"};
} // namespace

core::ModuleInfo module_info() noexcept {
    return core::ModuleInfo{"atmo", kDependencies};
}

} // namespace orrery::atmo
