#include "orrery/app/error_surface.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <string>
#include <system_error>
#include <utility>

#include "orrery/app/config.h"

namespace orrery::app {

namespace {

// Set once at start-up, before any thread exists; read by the handler.
struct TerminateSettings {
    std::filesystem::path log;
    std::string version;
};

TerminateSettings& settings() {
    static TerminateSettings instance;
    return instance;
}

// Appends the reason to the crash log, if one is set.
void write_crash_log(const char* reason) noexcept {
    const TerminateSettings& s = settings();
    if (s.log.empty()) {
        return;
    }
    try {
        if (append_crash_log(s.log, crash_log_entry(reason, s.version))) {
            std::fputs("orrery_app: crash report appended to ", stderr);
            std::fputs(s.log.string().c_str(), stderr);
            std::fputs("\n", stderr);
        }
    } catch (...) {
        std::fputs("orrery_app: the crash report could not be written\n", stderr);
    }
}

// Formatting the reason may fail (memory exhausted, for one); the fallback
// is a literal, so nothing in the handler can throw. A second termination
// while reporting (another thread, or the report itself) aborts at once.
[[noreturn]] void on_terminate() noexcept {
    static std::atomic<bool> entered{false};
    if (entered.exchange(true)) {
        std::fputs("orrery_app: std::terminate again while reporting; aborting\n", stderr);
        std::abort();
    }
    std::string reason;
    bool formatted = false;
    try {
        reason = describe_termination(std::current_exception());
        formatted = true;
    } catch (...) {
        formatted = false;
    }
    const char* text =
        formatted ? reason.c_str() : "std::terminate (the reason could not be formatted)";
    std::fputs("orrery_app: fatal: ", stderr);
    std::fputs(text, stderr);
    std::fputs("\n", stderr);
    write_crash_log(text);
    std::fflush(stderr);
    std::abort();
}

} // namespace

std::string describe_termination(const std::exception_ptr& active) {
    if (!active) {
        return "std::terminate called without an active exception";
    }
    try {
        std::rethrow_exception(active);
    } catch (const std::exception& error) {
        return std::string("unhandled exception: ") + error.what();
    } catch (...) {
        return "unhandled non-standard exception";
    }
}

std::string crash_log_entry(std::string_view reason, std::string_view version) {
    const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm utc{};
#if defined(_WIN32)
    (void)gmtime_s(&utc, &now);
#else
    (void)gmtime_r(&now, &utc);
#endif
    std::array<char, 32> stamp{};
    const std::size_t written =
        std::strftime(stamp.data(), stamp.size(), "%Y-%m-%dT%H:%M:%SZ", &utc);
    std::string entry(stamp.data(), written);
    entry += " Orrery ";
    entry += version;
    entry += ": ";
    entry += reason;
    entry += '\n';
    return entry;
}

bool append_crash_log(const std::filesystem::path& log, std::string_view entry) noexcept {
    try {
        std::error_code error;
        if (!log.parent_path().empty()) {
            std::filesystem::create_directories(log.parent_path(), error);
        }
        std::ofstream out(log, std::ios::binary | std::ios::app);
        out.write(entry.data(), static_cast<std::streamsize>(entry.size()));
        out.close();
        return static_cast<bool>(out);
    } catch (...) {
        return false;
    }
}

std::filesystem::path default_crash_log() {
    return data_dir() / "crash.log";
}

void install_terminate_handler(std::filesystem::path log, std::string version) {
    settings().log = std::move(log);
    settings().version = std::move(version);
    std::set_terminate(&on_terminate);
}

} // namespace orrery::app
