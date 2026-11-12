#include <catch2/catch_test_macros.hpp>

#include <initializer_list>
#include <string>
#include <string_view>
#include <vector>

#include "orrery/app/cli.h"

using namespace orrery::app;

namespace {

orrery::core::Result<CliOptions, std::string> parse(std::initializer_list<std::string_view> args) {
    const std::vector<std::string_view> list(args);
    return parse_command_line(list);
}

void expect_error(std::initializer_list<std::string_view> args, std::string_view part) {
    const auto parsed = parse(args);
    REQUIRE_FALSE(parsed.has_value());
    INFO(parsed.error());
    CHECK(parsed.error().find(part) != std::string::npos);
    CHECK(parsed.error().find('\n') == std::string::npos);
}

} // namespace

TEST_CASE("Command line: modes", "[app][cli]") {
    const auto none = parse({});
    REQUIRE(none.has_value());
    CHECK(none.value().mode == RunMode::window);
    CHECK_FALSE(none.value().experiment.has_value());

    CHECK(parse({"--version"}).value().mode == RunMode::version);
    CHECK(parse({"--help"}).value().mode == RunMode::help);
    CHECK(parse({"-h"}).value().mode == RunMode::help);
    CHECK(parse({"--modules"}).value().mode == RunMode::modules);
    // --help and --version answer even next to invalid arguments.
    CHECK(parse({"--bogus", "--help"}).value().mode == RunMode::help);
    CHECK(parse({"--frame", "x", "--version"}).value().mode == RunMode::version);

    const auto window = parse({"--experiment", "figure_eight", "--size", "1920x1080"});
    REQUIRE(window.has_value());
    CHECK(window.value().mode == RunMode::window);
    CHECK(window.value().experiment == "figure_eight");
    CHECK(window.value().width == 1920);
    CHECK(window.value().height == 1080);

    const auto bench = parse({"--benchmark=300"});
    REQUIRE(bench.has_value());
    CHECK(bench.value().benchmark_frames == 300);

    // Headless: time offscreen frames after writing the image.
    const auto headless_bench = parse({"--headless", "-e", "x", "-o", "a.png", "--benchmark", "5"});
    REQUIRE(headless_bench.has_value());
    CHECK(headless_bench.value().mode == RunMode::headless);
    CHECK(headless_bench.value().benchmark_frames == 5);
}

TEST_CASE("Command line: headless", "[app][cli]") {
    const auto parsed =
        parse({"--headless", "--experiment", "solar_system", "--frame", "120", "--out", "a.png"});
    REQUIRE(parsed.has_value());
    const CliOptions& options = parsed.value();
    CHECK(options.mode == RunMode::headless);
    CHECK(options.experiment == "solar_system");
    CHECK(options.frame == 120);
    CHECK(options.out == "a.png");
    CHECK_FALSE(options.width.has_value());

    // '=' form, short options, any order, upper-case extension.
    const auto compact =
        parse({"-o=", "x"}); // '=' is only split for long options: "-o=" is unknown
    CHECK_FALSE(compact.has_value());
    const auto other = parse({"--out=shot.PNG", "-e", "assets/experiments/solar_system.json",
                              "--headless", "--frame=0", "--size=640x360"});
    REQUIRE(other.has_value());
    CHECK(other.value().out == "shot.PNG");
    CHECK(other.value().experiment == "assets/experiments/solar_system.json");
    CHECK(other.value().frame == 0);
    CHECK(other.value().width == 640);
    CHECK(other.value().height == 360);

    // --frame defaults to 0.
    const auto no_frame = parse({"--headless", "-e", "x", "-o", "y.png"});
    REQUIRE(no_frame.has_value());
    CHECK(no_frame.value().frame == 0);
}

TEST_CASE("Command line: errors are one clear line", "[app][cli]") {
    expect_error({"--bogus"}, "unknown option '--bogus'");
    expect_error({"stray"}, "unexpected argument 'stray'");
    expect_error({"--headless", "--out", "a.png"}, "requires --experiment");
    expect_error({"--headless", "--experiment", "x"}, "requires --out");
    expect_error({"--headless", "-e", "x", "-o", "a.jpg"}, "use a .png file name");
    expect_error({"--headless", "-e", "x", "-o", "a.png", "--frame"}, "requires a value");
    expect_error({"--headless", "-e", "x", "-o", "a.png", "--frame", "-1"}, "--frame");
    expect_error({"--headless", "-e", "x", "-o", "a.png", "--frame", "12abc"}, "'12abc'");
    expect_error({"--headless", "-e", "x", "-o", "a.png", "--frame", "1000001"}, "[0, 1000000]");
    expect_error({"--headless", "-e", "x", "-o", "a.png", "--size", "10x10"}, "WIDTHxHEIGHT");
    expect_error({"--headless", "-e", "x", "-o", "a.png", "--size", "800"}, "WIDTHxHEIGHT");
    expect_error({"--frame", "10"}, "only valid with --headless");
    expect_error({"--out", "a.png"}, "only valid with --headless");
    expect_error({"-e", "a", "-e", "b"}, "more than once");
    expect_error({"--headless=yes"}, "takes no value");
    expect_error({"--experiment="}, "non-empty value");
    expect_error({"--modules", "-e", "x"}, "--modules");
    expect_error({"--benchmark", "0"}, "--benchmark");
}

TEST_CASE("Usage text lists every option", "[app][cli]") {
    const std::string usage = usage_text();
    for (const char* option : {"--headless", "--experiment", "--frame", "--out", "--size",
                               "--benchmark", "--version", "--help", "--modules"}) {
        CHECK(usage.find(option) != std::string::npos);
    }
}
