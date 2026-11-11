#include "orrery/geom/module.h"

#include <array>
#include <string_view>

namespace orrery::geom {

namespace {
constexpr std::array<std::string_view, 1> kDependencies{"core"};
} // namespace

core::ModuleInfo module_info() noexcept {
    return core::ModuleInfo{"geom", kDependencies};
}

} // namespace orrery::geom
