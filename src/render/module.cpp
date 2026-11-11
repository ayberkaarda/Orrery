#include "orrery/render/module.h"

#include <array>
#include <string_view>

namespace orrery::render {

namespace {
constexpr std::array<std::string_view, 3> kDependencies{"core", "geom", "aero"};
} // namespace

core::ModuleInfo module_info() noexcept {
    return core::ModuleInfo{"render", kDependencies};
}

} // namespace orrery::render
