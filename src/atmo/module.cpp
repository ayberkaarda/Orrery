#include "orrery/atmo/module.h"

#include <array>
#include <string_view>

namespace orrery::atmo {

namespace {
constexpr std::array<std::string_view, 1> kDependencies{"core"};
} // namespace

core::ModuleInfo module_info() noexcept {
    return core::ModuleInfo{"atmo", kDependencies};
}

} // namespace orrery::atmo
