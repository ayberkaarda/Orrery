#pragma once

#include <span>
#include <string>
#include <string_view>

namespace orrery::core {

// Semantic version of the Orrery build.
struct Version {
    int major;
    int minor;
    int patch;
};

// Static description of one library in the target graph.
struct ModuleInfo {
    std::string_view name;
    // Direct (non-transitive) library dependencies, by module name.
    std::span<const std::string_view> dependencies;
};

[[nodiscard]] Version version() noexcept;

// "MAJOR.MINOR.PATCH"
[[nodiscard]] std::string version_string();

[[nodiscard]] ModuleInfo module_info() noexcept;

} // namespace orrery::core
