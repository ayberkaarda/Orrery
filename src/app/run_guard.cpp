#include "orrery/app/run_guard.h"

#include <cstdio>
#include <optional>
#include <string>

#include "orrery/app/cli.h"
#include "orrery/core/trace_capture.h"

namespace orrery::app {

namespace {

// Stops the capture and writes it; false (reported on `err`) when it cannot.
bool write_trace(const std::filesystem::path& trace, std::ostream& err) {
    profile::stop_capture();
    if (const std::optional<std::string> error = profile::write_chrome_trace(trace, "orrery_app")) {
        err << "orrery_app: cannot write the trace " << trace.string() << ": " << *error << '\n';
        return false;
    }
    return true;
}

// The same while an exception is on its way out: nothing may replace it.
void write_trace_of_failed_run(const std::filesystem::path& trace, std::ostream& err) noexcept {
    try {
        (void)write_trace(trace, err);
    } catch (...) {
        std::fputs("orrery_app: the trace of the failed run could not be written\n", stderr);
    }
}

} // namespace

int run_traced(const std::filesystem::path& trace, const std::function<int()>& body,
               std::ostream& err) {
    profile::start_capture();
    int code = kExitFailure;
    try {
        code = body();
    } catch (...) {
        write_trace_of_failed_run(trace, err);
        throw;
    }
    if (!write_trace(trace, err) && code == kExitOk) {
        return kExitFailure;
    }
    return code;
}

} // namespace orrery::app
