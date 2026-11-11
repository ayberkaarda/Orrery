#pragma once

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>

#include "orrery/scene/scenario.h"

namespace orrery::scene::test {

// Directories baked in at compile time by tests/CMakeLists.txt.
inline std::filesystem::path fixture_path(const std::string& name) {
    return std::filesystem::path(ORRERY_SCENE_FIXTURE_DIR) / name;
}

inline std::filesystem::path scenario_path(const std::string& name) {
    return std::filesystem::path(ORRERY_SCENARIO_DIR) / name;
}

// Relative difference of two numbers. Any non-finite operand yields positive
// infinity, never NaN, so that no "< tolerance" or "== 0" assertion can pass
// on a NaN or infinite value (valid scenarios hold finite numbers only).
inline double relative_error(double expected, double actual) {
    if (!std::isfinite(expected) || !std::isfinite(actual)) {
        return std::numeric_limits<double>::infinity();
    }
    const double scale = std::max(std::abs(expected), std::abs(actual));
    return scale == 0.0 ? 0.0 : std::abs(expected - actual) / scale;
}

// Running maximum of relative_error over several pairs. NaN (which
// relative_error does not produce) would still propagate rather than vanish.
inline double worst_error(double worst, double expected, double actual) {
    const double e = relative_error(expected, actual);
    return e <= worst ? worst : e;
}

// Bit-for-bit equality; unlike ==, distinguishes 0.0 from -0.0.
inline bool same_bits(double a, double b) {
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

// A minimal valid document with the given bodies array body.
inline std::string document_with_bodies(const std::string& bodies) {
    return R"({"version": 1,
  "units": {"length": "AU", "mass": "Msun", "time": "day", "angle": "rad"},
  "metadata": {"source": "test", "epoch": "t = 0"},
  "bodies": [)" +
           bodies + "]}";
}

inline const std::string sun_json =
    R"({"name": "Sun", "kind": "star", "mass": 1.0,
        "state": {"position": [0.0, 0.0, 0.0], "velocity": [0.0, 0.0, 0.0]}})";

} // namespace orrery::scene::test
