#include "orrery/app/frame_stats.h"

#include <algorithm>
#include <cmath>

namespace orrery::app {

FrameDelta frame_delta(double raw_seconds) noexcept {
    const double raw = std::isfinite(raw_seconds) ? std::max(raw_seconds, 0.0) : 0.0;
    return {raw, std::min(raw, kMaxInteractiveFrameSeconds)};
}

void BenchmarkStats::add(const FrameDelta& delta) noexcept {
    ++seen_;
    if (seen_ <= warmup_) {
        return;
    }
    seconds_ += delta.raw_seconds;
    ++measured_;
}

double BenchmarkStats::mean_ms() const noexcept {
    return measured_ > 0 ? 1000.0 * seconds_ / static_cast<double>(measured_) : 0.0;
}

double BenchmarkStats::fps() const noexcept {
    const double mean = mean_ms();
    return mean > 0.0 ? 1000.0 / mean : 0.0;
}

} // namespace orrery::app
