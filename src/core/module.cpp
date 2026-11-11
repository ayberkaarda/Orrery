#include "orrery/core/module.h"

#ifndef ORRERY_VERSION_MAJOR
#error "ORRERY_VERSION_MAJOR must be defined by the build system"
#endif

namespace orrery::core {

Version version() noexcept {
    return Version{ORRERY_VERSION_MAJOR, ORRERY_VERSION_MINOR, ORRERY_VERSION_PATCH};
}

std::string version_string() {
    const Version v = version();
    return std::to_string(v.major) + '.' + std::to_string(v.minor) + '.' + std::to_string(v.patch);
}

ModuleInfo module_info() noexcept {
    return ModuleInfo{"core", {}};
}

} // namespace orrery::core
