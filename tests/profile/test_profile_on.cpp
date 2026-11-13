// The zone macros compiled in, whatever the build option: the macros are
// header-only and the trace sink is always built, so this translation unit
// turns them on for itself and exercises the sink in every build.
#if defined(ORRERY_PROFILING)
#define ORRERY_TEST_BUILD_PROFILING 1
#else
#define ORRERY_TEST_BUILD_PROFILING 0
#define ORRERY_PROFILING 1
#endif

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <set>
#include <string_view>
#include <thread>
#include <vector>

#include "orrery/core/profile.h"
#include "orrery/core/trace_capture.h"

namespace profile = orrery::profile;

namespace {

void busy_wait(std::chrono::microseconds duration) {
    const auto until = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < until) {
    }
}

std::vector<profile::ZoneEvent> events_named(const profile::Capture& capture,
                                             std::string_view name) {
    std::vector<profile::ZoneEvent> found;
    std::copy_if(capture.events.begin(), capture.events.end(), std::back_inserter(found),
                 [name](const profile::ZoneEvent& e) { return e.name == name; });
    return found;
}

const profile::ZoneTotal* total_named(const profile::Capture& capture, std::string_view name) {
    const auto found = std::find_if(capture.totals.begin(), capture.totals.end(),
                                    [name](const profile::ZoneTotal& t) { return t.name == name; });
    return found == capture.totals.end() ? nullptr : &*found;
}

void inner_work() {
    ORRERY_ZONE("test_inner");
    busy_wait(std::chrono::microseconds(200));
}

void zoned_work() {
    ORRERY_ZONE("test_thread_zone");
    busy_wait(std::chrono::microseconds(50));
}

} // namespace

TEST_CASE("zones_compiled reports the build option", "[profile]") {
    CHECK(profile::zones_compiled() == (ORRERY_TEST_BUILD_PROFILING == 1));
    if (profile::tracy_enabled()) {
        CHECK(profile::zones_compiled());
    }
}

TEST_CASE("a zone records nothing without a capture", "[profile]") {
    profile::stop_capture();
    {
        ORRERY_ZONE("test_idle");
    }
    profile::start_capture();
    profile::stop_capture();
    const profile::Capture capture = profile::capture_snapshot();
    CHECK(events_named(capture, "test_idle").empty());
    CHECK(total_named(capture, "test_idle") == nullptr);
    CHECK_FALSE(profile::capture_active());
}

TEST_CASE("zones record their name, nesting and duration", "[profile]") {
    profile::start_capture();
    CHECK(profile::capture_active());
    {
        ORRERY_ZONE("test_outer");
        busy_wait(std::chrono::microseconds(100));
        inner_work(); // a nested zone lives in its own function (profile.h)
    }
    profile::stop_capture();
    const profile::Capture capture = profile::capture_snapshot();
    const auto outer = events_named(capture, "test_outer");
    const auto inner = events_named(capture, "test_inner");
    REQUIRE(outer.size() == 1);
    REQUIRE(inner.size() == 1);
    CHECK(inner[0].duration_ns >= 200'000);
    CHECK(outer[0].duration_ns >= 300'000);
    CHECK(outer[0].start_ns >= 0);
    CHECK(outer[0].start_ns <= inner[0].start_ns);
    CHECK(inner[0].start_ns + inner[0].duration_ns <= outer[0].start_ns + outer[0].duration_ns);
    CHECK(inner[0].thread == outer[0].thread);
    CHECK(capture.duration_ns >= outer[0].start_ns + outer[0].duration_ns);
    // The inner zone completes first.
    const auto position = [&](std::string_view name) {
        return std::find_if(capture.events.begin(), capture.events.end(),
                            [name](const profile::ZoneEvent& e) { return e.name == name; });
    };
    CHECK(position("test_inner") < position("test_outer"));
    const profile::ZoneTotal* total = total_named(capture, "test_inner");
    REQUIRE(total != nullptr);
    CHECK(total->count == 1);
    CHECK(total->total_ns == inner[0].duration_ns);
    CHECK(total->min_ns == inner[0].duration_ns);
    CHECK(total->max_ns == inner[0].duration_ns);
}

TEST_CASE("every thread gets its own number", "[profile]") {
    profile::start_capture();
    std::thread first(zoned_work);
    first.join();
    std::thread second(zoned_work);
    second.join();
    zoned_work();
    profile::stop_capture();
    const auto events = events_named(profile::capture_snapshot(), "test_thread_zone");
    REQUIRE(events.size() == 3);
    std::set<std::uint32_t> threads;
    for (const profile::ZoneEvent& e : events) {
        threads.insert(e.thread);
    }
    CHECK(threads == std::set<std::uint32_t>{1, 2, 3});
}

TEST_CASE("the event cap drops events but keeps the totals", "[profile]") {
    profile::start_capture(4);
    for (int i = 0; i < 10; ++i) {
        ORRERY_ZONE("test_capped");
    }
    profile::stop_capture();
    const profile::Capture capture = profile::capture_snapshot();
    CHECK(capture.events.size() == 4);
    CHECK(capture.dropped == 6);
    const profile::ZoneTotal* total = total_named(capture, "test_capped");
    REQUIRE(total != nullptr);
    CHECK(total->count == 10);
    CHECK(total->min_ns <= total->max_ns);
}

TEST_CASE("a restart discards the previous capture and zones open across it", "[profile]") {
    profile::start_capture();
    {
        ORRERY_ZONE("test_before_restart");
    }
    {
        const profile::Zone straddle("test_straddle");
        profile::start_capture();
    }
    profile::stop_capture();
    const profile::Capture capture = profile::capture_snapshot();
    CHECK(events_named(capture, "test_before_restart").empty());
    CHECK(events_named(capture, "test_straddle").empty());
    CHECK(capture.totals.empty());
    CHECK(capture.dropped == 0);
}

TEST_CASE("zones still open when the capture stops are dropped", "[profile]") {
    profile::start_capture();
    {
        ORRERY_ZONE("test_open_at_stop");
        profile::stop_capture();
    }
    const profile::Capture capture = profile::capture_snapshot();
    CHECK(events_named(capture, "test_open_at_stop").empty());
    profile::record_zone("test_after_stop", 0, 10);
    CHECK(profile::capture_snapshot().events.empty());
}

TEST_CASE("record_zone clamps a negative duration and ignores a null name", "[profile]") {
    profile::start_capture();
    profile::record_zone("test_direct", 50, 20);
    profile::record_zone(nullptr, 0, 10);
    profile::stop_capture();
    const profile::Capture capture = profile::capture_snapshot();
    REQUIRE(capture.events.size() == 1);
    CHECK(capture.events[0].name == "test_direct");
    CHECK(capture.events[0].start_ns == 50);
    CHECK(capture.events[0].duration_ns == 0);
}

// Cost of one zone with the macros compiled in: idle (no capture) and
// recording. Hidden; run with orrery_test_profile "[benchmark]".
TEST_CASE("zone cost with the macros compiled in", "[.benchmark][profile]") {
    constexpr int kZones = 1'000'000;
    const auto time_zones = [] {
        const auto start = std::chrono::steady_clock::now();
        for (int i = 0; i < kZones; ++i) {
            ORRERY_ZONE("test_bench");
        }
        const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now() - start)
                            .count();
        return static_cast<double>(ns) / kZones;
    };
    profile::stop_capture();
    const double idle = time_zones();
    profile::start_capture(kZones);
    const double recording = time_zones();
    profile::stop_capture();
    std::cout << "zone cost: " << idle << " ns idle, " << recording << " ns recording\n";
    CHECK(profile::capture_snapshot().events.size() == static_cast<std::size_t>(kZones));
}
