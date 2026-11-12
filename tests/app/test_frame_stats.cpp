#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <limits>

#include "orrery/app/frame_stats.h"

using namespace orrery::app;
using Catch::Approx;

TEST_CASE("Frame deltas clamp only the interactive part", "[app][frame_stats]") {
    const FrameDelta normal = frame_delta(0.016);
    CHECK(normal.raw_seconds == 0.016);
    CHECK(normal.interactive_seconds == 0.016);
    const FrameDelta stall = frame_delta(1.5);
    CHECK(stall.raw_seconds == 1.5);
    CHECK(stall.interactive_seconds == kMaxInteractiveFrameSeconds);
    CHECK(frame_delta(-1.0).raw_seconds == 0.0);
    CHECK(frame_delta(std::numeric_limits<double>::quiet_NaN()).interactive_seconds == 0.0);
}

// Review fix: a benchmark must report slow frames as slow, not as the 250 ms
// the interactive updates are clamped to.
TEST_CASE("Benchmark statistics use the measured frame time", "[app][frame_stats]") {
    BenchmarkStats stats(1);
    stats.add(frame_delta(5.0)); // warm-up, ignored
    CHECK(stats.frames() == 0);
    CHECK(stats.mean_ms() == 0.0);
    stats.add(frame_delta(1.0));
    stats.add(frame_delta(0.016));
    CHECK(stats.frames() == 2);
    CHECK(stats.mean_ms() == Approx(508.0));
    CHECK(stats.fps() == Approx(1000.0 / 508.0));
}
