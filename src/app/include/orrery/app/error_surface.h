#pragma once

#include <exception>
#include <filesystem>
#include <string>
#include <string_view>

// Error surface of orrery_app (spec 8, docs/app.md "Error surface"): no
// silent crash. A std::terminate handler reports the reason (the active
// exception's message, if any) on stderr and appends it with a timestamp to
// a crash log, then aborts. Recoverable errors never get here: they are
// shown where they happen (import modal, status line, command-line message).
namespace orrery::app {

// "unhandled exception: <what>", "unhandled non-standard exception", or
// "std::terminate called without an active exception".
[[nodiscard]] std::string describe_termination(const std::exception_ptr& active);

// One crash-log entry: "<ISO 8601 UTC time> Orrery <version>: <reason>\n".
[[nodiscard]] std::string crash_log_entry(std::string_view reason, std::string_view version);

// Appends `entry` to the log (creating its directory); false when it cannot.
bool append_crash_log(const std::filesystem::path& log, std::string_view entry) noexcept;

// data_dir()/crash.log.
[[nodiscard]] std::filesystem::path default_crash_log();

// Installs the handler; `log` empty reports on stderr only.
void install_terminate_handler(std::filesystem::path log, std::string version);

} // namespace orrery::app
