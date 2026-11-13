#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#include <nlohmann/json.hpp>

#include "orrery/app/cli.h"
#include "orrery/app/run_guard.h"
#include "orrery/core/profile.h"
#include "orrery/core/trace_capture.h"

// How main runs a command (docs/app.md, "Error surface"; docs/observability.md,
// "--trace"): the trace of a run that throws is still written, and an
// exception that reaches main is reported on stderr and in the crash log,
// or the crash log's own failure is.

using namespace orrery;
using namespace orrery::app;

namespace {

std::filesystem::path scratch(const char* name) {
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / "orrery_test_run_guard" / name;
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

std::string read_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

// Number of complete events named `name` in the Chrome trace at `path`.
std::int64_t zone_events(const std::filesystem::path& path, std::string_view name) {
    const nlohmann::json trace = nlohmann::json::parse(read_file(path));
    std::int64_t count = 0;
    for (const nlohmann::json& event : trace.at("traceEvents")) {
        if (event.value("ph", "") == "X" && event.value("name", "") == name) {
            ++count;
        }
    }
    return count;
}

int fail_after_a_zone() {
    {
        const profile::Zone zone("test_zone_before_failure");
    }
    throw std::runtime_error("injected failure");
}

int throw_a_number() {
    throw 42;
}

bool contains(const std::string& text, std::string_view part) {
    return text.find(part) != std::string::npos;
}

} // namespace

TEST_CASE("A run that throws still writes its trace", "[app][run_guard][trace]") {
    const std::filesystem::path dir = scratch("trace");
    std::ostringstream err;
    CHECK_THROWS_WITH(run_traced(dir / "failed.json", fail_after_a_zone, err), "injected failure");
    CHECK_FALSE(profile::capture_active());
    REQUIRE(std::filesystem::exists(dir / "failed.json"));
    CHECK(zone_events(dir / "failed.json", "test_zone_before_failure") == 1);
    CHECK(err.str().empty());

    // Through main's guard: the failure ends the run with exit code 1, and
    // the trace is there.
    const int code = run_reporting_errors(
        [&] { return run_traced(dir / "guarded.json", fail_after_a_zone, err); }, dir / "crash.log",
        "test", err);
    CHECK(code == kExitFailure);
    CHECK(zone_events(dir / "guarded.json", "test_zone_before_failure") == 1);
    CHECK(contains(err.str(), "orrery_app: unexpected error: injected failure"));

    // A trace that cannot be written does not hide the original failure.
    std::ostringstream unwritable;
    CHECK_THROWS_WITH(run_traced(dir / "no_such_dir" / "t.json", fail_after_a_zone, unwritable),
                      "injected failure");
    CHECK(contains(unwritable.str(), "orrery_app: cannot write the trace"));

    // A successful run keeps its code; an unwritable trace fails it.
    std::ostringstream quiet;
    CHECK(run_traced(dir / "ok.json", [] { return kExitOk; }, quiet) == kExitOk);
    CHECK(
        run_traced(dir / "no_such_dir" / "ok.json", [] { return kExitOk; }, quiet) == kExitFailure);
    CHECK(run_traced(dir / "no_such_dir" / "ok.json", [] { return 3; }, quiet) == 3);
}

TEST_CASE("An exception reaching main is reported and logged", "[app][run_guard][crash_log]") {
    const std::filesystem::path dir = scratch("crash_log");
    std::ostringstream err;
    CHECK(run_reporting_errors(fail_after_a_zone, dir / "logs" / "crash.log", "1.2.3", err) ==
          kExitFailure);
    CHECK(contains(err.str(), "orrery_app: unexpected error: injected failure\n"));
    CHECK(contains(err.str(), "orrery_app: crash report appended to "));
    const std::string log = read_file(dir / "logs" / "crash.log");
    CHECK(contains(log, "Orrery 1.2.3: unexpected error: injected failure\n"));

    std::ostringstream other;
    CHECK(run_reporting_errors(throw_a_number, dir / "logs" / "crash.log", "1.2.3", other) ==
          kExitFailure);
    CHECK(contains(other.str(), "orrery_app: unexpected error\n"));
    CHECK(contains(read_file(dir / "logs" / "crash.log"), "unexpected error: non-standard"));

    std::ostringstream fine;
    CHECK(run_reporting_errors([] { return 3; }, dir / "unused.log", "1.2.3", fine) == 3);
    CHECK(fine.str().empty());
    CHECK_FALSE(std::filesystem::exists(dir / "unused.log"));
}

TEST_CASE("A crash log that cannot be written is reported", "[app][run_guard][crash_log]") {
    const std::filesystem::path dir = scratch("crash_log_blocked");
    // The log's parent is a regular file: neither the directory nor the log
    // can be created.
    {
        std::ofstream(dir / "blocker") << "not a directory";
    }
    std::ostringstream err;
    CHECK(run_reporting_errors(fail_after_a_zone, dir / "blocker" / "crash.log", "1.2.3", err) ==
          kExitFailure);
    CHECK(contains(err.str(), "orrery_app: unexpected error: injected failure\n"));
    CHECK(contains(err.str(), "orrery_app: cannot write the crash report to "));
    CHECK_FALSE(contains(err.str(), "appended"));
}
