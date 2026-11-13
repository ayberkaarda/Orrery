#pragma once

#include <filesystem>
#include <functional>
#include <ostream>
#include <string_view>

// How orrery_app's main runs a command (docs/app.md, "Error surface";
// docs/observability.md, "--trace"). Graphics-free and testable: main only
// passes its streams and the command.
namespace orrery::app {

// Runs `body` with the trace sink capturing, then writes the trace to
// `trace`, also when `body` throws: the trace of the failed run is written
// (a failure to write it is reported on `err`) and the exception propagates
// unchanged. Otherwise returns body's exit code, or kExitFailure instead of
// kExitOk when the trace cannot be written (reported on `err`).
[[nodiscard]] int run_traced(const std::filesystem::path& trace, const std::function<int()>& body,
                             std::ostream& err);

// main's last line of defence: returns body's exit code. An exception that
// escapes `body` is reported on `err` ("orrery_app: unexpected error:
// <what>") and appended to `crash_log` with `version` (crash_log_entry);
// the line after it says where it went, or that the crash log could not be
// written. The result is then kExitFailure. Never throws.
[[nodiscard]] int run_reporting_errors(const std::function<int()>& body,
                                       const std::filesystem::path& crash_log,
                                       std::string_view version, std::ostream& err) noexcept;

} // namespace orrery::app
