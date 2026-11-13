// The zone macros compiled out, whatever the build option: this translation
// unit includes the header without ORRERY_PROFILING, as every default and CI
// build does, and shows that the macros leave no code behind.
#undef ORRERY_PROFILING
#undef ORRERY_PROFILING_TRACY

#include <catch2/catch_test_macros.hpp>

#include <string_view>

#include "orrery/core/profile.h"
#include "orrery/core/trace_capture.h"

namespace profile = orrery::profile;

#define ORRERY_TEST_STRINGIFY(...) #__VA_ARGS__
#define ORRERY_TEST_EXPANDED(...) ORRERY_TEST_STRINGIFY(__VA_ARGS__)

namespace {

int zoned_sum(int n) {
    ORRERY_ZONE("test_compiled_out");
    int sum = 0;
    for (int i = 1; i <= n; ++i) {
        sum += i;
    }
    return sum;
}

} // namespace

TEST_CASE("without ORRERY_PROFILING the zone macros expand to a void cast", "[profile]") {
    // The whole expansion: no object, no call, no clock read, no Tracy.
    CHECK(std::string_view(ORRERY_TEST_EXPANDED(ORRERY_ZONE("physics_step"))) ==
          "static_cast<void>(0)");
    CHECK(std::string_view(ORRERY_TEST_EXPANDED(ORRERY_FRAME_ZONE())) == "static_cast<void>(0)");
}

TEST_CASE("compiled-out zones record nothing during a capture", "[profile]") {
    profile::start_capture();
    CHECK(zoned_sum(10) == 55);
    profile::stop_capture();
    const profile::Capture capture = profile::capture_snapshot();
    for (const profile::ZoneEvent& event : capture.events) {
        CHECK(event.name != "test_compiled_out");
    }
    CHECK(capture.totals.empty());
}
