#include <catch2/catch_test_macros.hpp>

#include <exception>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include "orrery/app/config.h"
#include "orrery/app/error_surface.h"

// Phase 8b: user configuration (toml++) and the terminate handler's report.

using namespace orrery;

namespace {

std::filesystem::path scratch(const char* name) {
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / "orrery_test_config" / name;
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

} // namespace

TEST_CASE("Configuration defaults live in the platform directories", "[app][config]") {
    const app::AppConfig defaults = app::default_config();
    CHECK(defaults.window_width == 1600);
    CHECK(defaults.window_height == 900);
    CHECK(defaults.start_workspace == app::Workspace::space);
    CHECK(defaults.experiment_dir.filename() == "experiments");
    CHECK(defaults.experiment_dir.parent_path() == app::data_dir());
    CHECK(app::default_config_path() == app::config_dir() / "config.toml");
    CHECK_FALSE(defaults.model_cache_dir.empty());
    CHECK(app::config_dir().is_absolute());
}

TEST_CASE("Configuration files are read with warnings for what is ignored", "[app][config]") {
    const auto parsed = app::parse_config(R"(
[paths]
experiments = "lab/runs"

[window]
width = 1280
height = 99999
workspace = "aero"
colour = "blue"

[aero]
steps_per_frame = 250

[telemetry]
on = true
)",
                                          "config.toml");
    REQUIRE(parsed.has_value());
    const app::AppConfig& c = parsed.value().config;
    CHECK(c.experiment_dir == std::filesystem::path("lab/runs"));
    CHECK(c.window_width == 1280);
    CHECK(c.window_height == 900); // out of range: the default stays
    CHECK(c.start_workspace == app::Workspace::aero);
    CHECK(c.steps_per_frame == 250);
    const std::vector<std::string>& warnings = parsed.value().warnings;
    REQUIRE(warnings.size() == 3);
    CHECK(warnings[0].find("telemetry: unknown key, ignored") != std::string::npos);
    CHECK(warnings[1].find("window.colour: unknown key") != std::string::npos);
    CHECK(warnings[2].starts_with("config.toml:7:"));
    CHECK(warnings[2].find("window.height: expected an integer in [320, 8192]") !=
          std::string::npos);
}

TEST_CASE("A broken configuration file is an error with its position", "[app][config]") {
    const auto parsed = app::parse_config("[window]\nwidth = = 3\n", "config.toml");
    REQUIRE_FALSE(parsed.has_value());
    CHECK(parsed.error().starts_with("config.toml:2:"));
}

TEST_CASE("Configuration round-trips through its TOML text and file", "[app][config]") {
    app::AppConfig c = app::default_config();
    c.experiment_dir = "/data/orrery experiments";
    c.model_cache_dir = "/cache/models";
    c.window_width = 1920;
    c.window_height = 1080;
    c.start_workspace = app::Workspace::aero;
    c.steps_per_frame = 500;
    const auto parsed = app::parse_config(app::config_to_toml(c), "config.toml");
    REQUIRE(parsed.has_value());
    CHECK(parsed.value().warnings.empty());
    CHECK(parsed.value().config == c);

    const std::filesystem::path dir = scratch("roundtrip");
    // A missing file means the defaults, not an error.
    const auto missing = app::load_config(dir / "none.toml");
    REQUIRE(missing.has_value());
    CHECK(missing.value().config == app::default_config());
    REQUIRE(app::save_config(dir / "sub" / "config.toml", c).has_value());
    const auto loaded = app::load_config(dir / "sub" / "config.toml");
    REQUIRE(loaded.has_value());
    CHECK(loaded.value().config == c);
}

namespace {

// Runs in `dir` for its lifetime (the test process's working directory).
class WorkingDirectory {
public:
    explicit WorkingDirectory(const std::filesystem::path& dir)
        : previous_(std::filesystem::current_path()) {
        std::filesystem::current_path(dir);
    }
    ~WorkingDirectory() {
        std::error_code ignored;
        std::filesystem::current_path(previous_, ignored);
    }
    WorkingDirectory(const WorkingDirectory&) = delete;
    WorkingDirectory& operator=(const WorkingDirectory&) = delete;
    WorkingDirectory(WorkingDirectory&&) = delete;
    WorkingDirectory& operator=(WorkingDirectory&&) = delete;

private:
    std::filesystem::path previous_;
};

} // namespace

TEST_CASE("Configuration saves to a bare file name and to a nested path", "[app][config]") {
    app::AppConfig c = app::default_config();
    c.window_width = 1024;
    c.steps_per_frame = 200;
    const WorkingDirectory here(scratch("relative"));
    for (const std::filesystem::path path :
         {std::filesystem::path("config.toml"), std::filesystem::path("nested/dir/config.toml")}) {
        INFO(path.string());
        const auto saved = app::save_config(path, c);
        INFO((saved ? std::string() : saved.error()));
        REQUIRE(saved.has_value());
        const auto loaded = app::load_config(path);
        REQUIRE(loaded.has_value());
        CHECK(loaded.value().config == c);
    }
}

TEST_CASE("The terminate report names the active exception", "[app][error]") {
    CHECK(app::describe_termination(nullptr) ==
          "std::terminate called without an active exception");
    CHECK(app::describe_termination(std::make_exception_ptr(std::runtime_error("bad mesh"))) ==
          "unhandled exception: bad mesh");
    CHECK(app::describe_termination(std::make_exception_ptr(42)) ==
          "unhandled non-standard exception");

    const std::string entry = app::crash_log_entry("unhandled exception: x", "0.0.1");
    CHECK(entry.size() > 20);
    CHECK(entry[4] == '-');
    CHECK(entry[10] == 'T');
    CHECK(entry.find("Z Orrery 0.0.1: unhandled exception: x\n") != std::string::npos);

    const std::filesystem::path log = scratch("crash") / "logs" / "crash.log";
    REQUIRE(app::append_crash_log(log, "first\n"));
    REQUIRE(app::append_crash_log(log, "second\n"));
    std::ifstream in(log, std::ios::binary);
    std::ostringstream text;
    text << in.rdbuf();
    CHECK(text.str() == "first\nsecond\n");
    CHECK(app::default_crash_log().filename() == "crash.log");
}
