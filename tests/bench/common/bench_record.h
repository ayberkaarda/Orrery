#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <initializer_list>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

// Measurements of the benchmark executables (docs/performance.md). Each
// benchmark records what it measured against its project target (spec
// section 8); bench_main.cpp writes every record of the run as JSON
// (--json <file>). A missed target is reported, never a failure of the run:
// only a broken result (a check inside the benchmark) fails it.
namespace orrery::bench {

// How the value is compared with the target.
enum class Goal : unsigned char {
    below,    // pass if value < target
    at_least, // pass if value >= target
    info,     // no target: recorded for context
};

struct Measurement {
    std::string id;    // stable key, e.g. "core.force.barnes_hut.n10000"
    std::string title; // row text of the report
    double value = 0.0;
    std::string unit;
    Goal goal = Goal::info;
    double target = 0.0;
    std::string notes;
    std::vector<std::pair<std::string, std::string>> context;
};

// Stores a measurement and prints it as one "bench:" line.
void record(Measurement measurement);

// Records that a measurement could not be taken (no OpenGL context, an
// unoptimised build) with the reason.
void record_skipped(const std::string& id, const std::string& title, const std::string& reason);

// The compute or render device the run used (driver renderer string).
void record_device(const std::string& device);

[[nodiscard]] inline std::string number(double value) {
    return std::to_string(value);
}

// Concatenation of the parts (one allocation pass, usable in loops).
[[nodiscard]] inline std::string cat(std::initializer_list<std::string_view> parts) {
    std::size_t size = 0;
    for (const std::string_view part : parts) {
        size += part.size();
    }
    std::string out;
    out.reserve(size);
    for (const std::string_view part : parts) {
        out.append(part);
    }
    return out;
}

// Median of samples (upper median for an even count); 0 for none.
[[nodiscard]] inline double median(std::vector<double> samples) {
    if (samples.empty()) {
        return 0.0;
    }
    std::sort(samples.begin(), samples.end());
    return samples[samples.size() / 2];
}

// Wall time of `work()` in milliseconds, repeated: `warmup` untimed calls,
// then `repeats` timed ones.
template <typename Work>
[[nodiscard]] std::vector<double> time_ms(Work&& work, int warmup, int repeats) {
    for (int i = 0; i < warmup; ++i) {
        work();
    }
    std::vector<double> samples;
    samples.reserve(static_cast<std::size_t>(std::max(repeats, 0)));
    for (int i = 0; i < repeats; ++i) {
        const auto start = std::chrono::steady_clock::now();
        work();
        const auto stop = std::chrono::steady_clock::now();
        samples.push_back(std::chrono::duration<double, std::milli>(stop - start).count());
    }
    return samples;
}

// True when the build defines NDEBUG (Release, RelWithDebInfo).
[[nodiscard]] constexpr bool optimised_build() noexcept {
#ifdef NDEBUG
    return true;
#else
    return false;
#endif
}

} // namespace orrery::bench
