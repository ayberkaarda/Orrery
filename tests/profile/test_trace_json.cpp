#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <ios>
#include <string>
#include <string_view>

#include <nlohmann/json.hpp>

#include "orrery/core/trace_capture.h"

namespace profile = orrery::profile;
using nlohmann::json;

namespace {

profile::Capture sample_capture() {
    profile::Capture capture;
    capture.events = {
        profile::ZoneEvent{"physics_step", 1'234'567, 2'000, 1},
        profile::ZoneEvent{"tree_build", 0, 999, 2},
        profile::ZoneEvent{"physics_step", 2'000'000, 1'000, 1},
    };
    capture.totals = {
        profile::ZoneTotal{"physics_step", 2, 3'000, 1'000, 2'000},
        profile::ZoneTotal{"tree_build", 1, 999, 999, 999},
    };
    capture.dropped = 5;
    capture.duration_ns = 3'500'001;
    return capture;
}

std::string read_file(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    file.seekg(0, std::ios::end);
    const std::streamoff size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::string text;
    if (size > 0) {
        text.assign(static_cast<std::size_t>(size), '\0');
        file.read(text.data(), static_cast<std::streamsize>(size));
    }
    return text;
}

} // namespace

TEST_CASE("the trace is Chrome trace-event JSON with metadata and totals", "[profile][trace]") {
    const std::string text = profile::chrome_trace_json(sample_capture(), "orrery_test");
    const json trace = json::parse(text);
    REQUIRE(trace.is_object());
    CHECK(trace.at("displayTimeUnit") == "ms");

    const json& events = trace.at("traceEvents");
    REQUIRE(events.is_array());
    // process name, two thread names, three zones
    REQUIRE(events.size() == 6);
    CHECK(events[0].at("ph") == "M");
    CHECK(events[0].at("name") == "process_name");
    CHECK(events[0].at("args").at("name") == "orrery_test");
    CHECK(events[1].at("name") == "thread_name");
    CHECK(events[1].at("tid") == 1);
    CHECK(events[2].at("args").at("name") == "thread 2");

    const json& zone = events[3];
    CHECK(zone.at("name") == "physics_step");
    CHECK(zone.at("ph") == "X");
    CHECK(zone.at("cat") == "zone");
    CHECK(zone.at("pid") == 1);
    CHECK(zone.at("tid") == 1);
    REQUIRE(zone.at("ts").is_number());
    REQUIRE(zone.at("dur").is_number());
    // Microseconds with three exact decimals.
    CHECK(text.find(R"("ts":1234.567,"dur":2.000)") != std::string::npos);
    CHECK(text.find(R"("ts":0.000,"dur":0.999)") != std::string::npos);
    CHECK(events[4].at("tid") == 2);

    const json& other = trace.at("otherData");
    CHECK(other.at("format") == "orrery-trace");
    CHECK(other.at("version") == "1");
    CHECK(other.at("zones_compiled") == (profile::zones_compiled() ? "true" : "false"));
    CHECK(other.at("tracy") == (profile::tracy_enabled() ? "true" : "false"));
    CHECK(other.at("duration_us") == "3500.001");
    CHECK(other.at("events") == "3");
    CHECK(other.at("dropped") == "5");

    const json& totals = trace.at("zoneTotals");
    REQUIRE(totals.size() == 2);
    CHECK(totals[0].at("name") == "physics_step");
    CHECK(totals[0].at("count") == 2);
    CHECK(text.find(R"("total_us":3.000,"mean_us":1.500,"min_us":1.000,"max_us":2.000)") !=
          std::string::npos);
    CHECK(totals[1].at("name") == "tree_build");
}

TEST_CASE("an empty capture is a valid trace", "[profile][trace]") {
    const json trace = json::parse(profile::chrome_trace_json(profile::Capture{}, "empty"));
    REQUIRE(trace.at("traceEvents").size() == 1);
    CHECK(trace.at("traceEvents")[0].at("name") == "process_name");
    CHECK(trace.at("zoneTotals").empty());
    CHECK(trace.at("otherData").at("events") == "0");
}

TEST_CASE("names are escaped as JSON strings", "[profile][trace]") {
    profile::Capture capture;
    capture.events = {profile::ZoneEvent{"quote\" back\\ tab\t bell\x07", 0, 1, 1}};
    capture.totals = {profile::ZoneTotal{"quote\" back\\ tab\t bell\x07", 1, 1, 1, 1}};
    const json trace = json::parse(profile::chrome_trace_json(capture, R"(a "process")"));
    CHECK(trace.at("traceEvents")[0].at("args").at("name") == R"(a "process")");
    CHECK(trace.at("traceEvents")[2].at("name") == "quote\" back\\ tab\t bell\x07");
    CHECK(trace.at("zoneTotals")[0].at("name") == "quote\" back\\ tab\t bell\x07");
}

TEST_CASE("write_chrome_trace writes the current capture", "[profile][trace]") {
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / "orrery_test_profile_trace";
    std::filesystem::create_directories(dir);
    const std::filesystem::path path = dir / "trace.json";
    profile::start_capture();
    profile::record_zone("test_written", 10, 1'010);
    profile::stop_capture();
    REQUIRE_FALSE(profile::write_chrome_trace(path, "writer").has_value());
    const json trace = json::parse(read_file(path));
    const json& events = trace.at("traceEvents");
    REQUIRE(events.size() == 3);
    CHECK(events[2].at("name") == "test_written");
    CHECK(trace.at("zoneTotals")[0].at("count") == 1);
    std::filesystem::remove_all(dir);

    const auto failed = profile::write_chrome_trace(dir / "missing" / "trace.json", "writer");
    REQUIRE(failed.has_value());
    CHECK(failed->find("cannot open") != std::string::npos);
}
