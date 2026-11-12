#include <cstddef>
#include <iostream>
#include <span>
#include <string_view>

#include "build_info.h"

namespace {

void print_modules(std::ostream& out) {
    out << "modules:\n";
    for (const orrery::core::ModuleInfo& module : orrery::app::linked_modules()) {
        out << "  " << module.name << " ->";
        if (module.dependencies.empty()) {
            out << " (none)";
        }
        for (const std::string_view dependency : module.dependencies) {
            out << ' ' << dependency;
        }
        out << '\n';
    }
}

int run(int argc, char** argv) {
    const std::span<char*> args(argv, static_cast<std::size_t>(argc));

    std::cout << "Orrery " << orrery::core::version_string() << '\n';

    for (const char* arg : args.empty() ? args : args.subspan(1)) {
        if (std::string_view(arg) == "--version") {
            return 0;
        }
    }

    if constexpr (orrery::app::kHeadless) {
        std::cout << "build: headless\n";
    }
    print_modules(std::cout);
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    try {
        return run(argc, argv);
    } catch (...) {
        return 1;
    }
}
