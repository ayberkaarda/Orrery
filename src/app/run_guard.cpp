#include "orrery/app/run_guard.h"

#include <cstdio>
#include <exception>
#include <optional>
#include <string>

#include "orrery/app/cli.h"
#include "orrery/app/error_surface.h"
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

// Reports an exception that reached main: on `err`, then in the crash log,
// whose outcome is reported as well (it catches its own failures).
void report_unexpected(const char* what, const std::filesystem::path& crash_log,
                       std::string_view version, std::ostream& err) noexcept {
    try {
        err << "orrery_app: unexpected error" << (what != nullptr ? ": " : "")
            << (what != nullptr ? what : "") << '\n';
        std::string reason = "unexpected error: ";
        reason += what != nullptr ? what : "non-standard exception";
        if (append_crash_log(crash_log, crash_log_entry(reason, version))) {
            err << "orrery_app: crash report appended to " << crash_log.string() << '\n';
        } else {
            err << "orrery_app: cannot write the crash report to " << crash_log.string() << '\n';
        }
        err.flush();
    } catch (...) {
        std::fputs("orrery_app: the crash report could not be written\n", stderr);
    }
}

} // namespace

int run_reporting_errors(const std::function<int()>& body, const std::filesystem::path& crash_log,
                         std::string_view version, std::ostream& err) noexcept {
    try {
        return body();
    } catch (const std::exception& error) {
        report_unexpected(error.what(), crash_log, version, err);
    } catch (...) {
        report_unexpected(nullptr, crash_log, version, err);
    }
    return kExitFailure;
}

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
