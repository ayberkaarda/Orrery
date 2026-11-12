#pragma once

#include <cstdint>

// Frame timing of the window (docs/app.md, "Window"): the clamped delta that
// drives interactive updates, and the benchmark statistics.
namespace orrery::app {

// Longest frame delta fed to interactive updates (camera, UI, smoothing) [s].
inline constexpr double kMaxInteractiveFrameSeconds = 0.25;

struct FrameDelta {
    double raw_seconds = 0.0;         // measured, for statistics
    double interactive_seconds = 0.0; // clamped to [0, kMaxInteractiveFrameSeconds]
};

// Splits a measured frame duration; negative or NaN durations count as 0.
[[nodiscard]] FrameDelta frame_delta(double raw_seconds) noexcept;

// Mean frame time over the frames after a warm-up.
class BenchmarkStats {
public:
    explicit BenchmarkStats(std::int64_t warmup_frames) noexcept : warmup_(warmup_frames) {}

    void add(const FrameDelta& delta) noexcept;

    [[nodiscard]] std::int64_t frames() const noexcept { return measured_; }
    [[nodiscard]] double mean_ms() const noexcept;
    [[nodiscard]] double fps() const noexcept;

private:
    std::int64_t warmup_ = 0;
    std::int64_t seen_ = 0;
    std::int64_t measured_ = 0;
    double seconds_ = 0.0;
};

} // namespace orrery::app
