// Runs orrery_app's terminate handler in a process of its own (it aborts):
//   terminate_probe exception [log]     an unhandled exception, memory available
//   terminate_probe out_of_memory [log] the same with every allocation failing
//                                       from the moment std::terminate is called
// With [log] the handler also appends the report to that crash log (its
// directory is created). tests/app/expect_abort.cmake checks the report on
// standard error, the crash log and the abnormal exit.
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <optional>
#include <stdexcept>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

#include "../core/allocation_failure_injection.h"
#include "orrery/app/error_surface.h"

namespace {

// No dialog or error report on abort or a crash: the probe runs unattended.
void quiet_failures() {
#if defined(_WIN32)
    (void)SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX);
#if defined(_MSC_VER)
    (void)_set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#endif
#endif
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2 && argc != 3) {
        return 2;
    }
    quiet_failures();
    orrery::app::install_terminate_handler(
        argc == 3 ? std::filesystem::path(argv[2]) : std::filesystem::path(), "probe");
    const bool exhausted = std::strcmp(argv[1], "out_of_memory") == 0;
    try {
        throw std::runtime_error("probe failure with a message longer than any small string");
    } catch (...) {
        // Armed only for out_of_memory: no allocation succeeds from here on
        // (the handler aborts with the scope still alive).
        std::optional<orrery::test::AllocationFailureScope> scope;
        if (exhausted) {
            scope.emplace(0, true);
        }
        std::terminate();
    }
}
