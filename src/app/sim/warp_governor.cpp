#include "orrery/sim/warp_governor.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace orrery::sim {

namespace {

// The highest level is 5e18, below the int64 limit (9.2e18).
constexpr std::int64_t kTopDecade = 1'000'000'000'000'000'000;
constexpr std::array<std::int64_t, 3> kMantissas{1, 2, 5};

// Exponential smoothing towards `sample` with weight wall / smoothing (at
// most 1), so the estimate's time constant is the same at any tick rate.
double smooth(std::optional<double> estimate, double sample, std::int64_t wall_ns,
              std::int64_t smoothing_ns) {
    if (!estimate) {
        return sample;
    }
    const double weight =
        std::min(1.0, static_cast<double>(wall_ns) / static_cast<double>(smoothing_ns));
    return *estimate + (sample - *estimate) * weight;
}

} // namespace

std::optional<std::string> validate(const WarpBudget& budget) {
    if (!(budget.load_limit >= kMinLoadLimit && budget.load_limit <= kMaxLoadLimit)) {
        return std::string("the physics load limit must be in [0.05, 0.95]");
    }
    if (!(budget.raise_margin >= 0.1 && budget.raise_margin < 1.0)) {
        return std::string("the raise margin must be in [0.1, 1)");
    }
    if (budget.lower_after_ns < 0 || budget.raise_after_ns < 0) {
        return std::string("the warp hysteresis times must not be negative");
    }
    if (budget.smoothing_ns <= 0) {
        return std::string("the warp smoothing time must be positive");
    }
    return std::nullopt;
}

std::int64_t warp_level_at_or_below(double warp) noexcept {
    if (!(warp >= 2.0)) { // also NaN
        return 1;
    }
    std::int64_t best = 1;
    for (std::int64_t decade = 1; decade <= kTopDecade; decade *= 10) {
        for (const std::int64_t mantissa : kMantissas) {
            const std::int64_t level = mantissa * decade;
            if (static_cast<double>(level) > warp) {
                return best;
            }
            best = level;
        }
        if (decade == kTopDecade) {
            break;
        }
    }
    return best;
}

std::int64_t warp_level_above(std::int64_t warp) noexcept {
    for (std::int64_t decade = 1; decade <= kTopDecade; decade *= 10) {
        for (const std::int64_t mantissa : kMantissas) {
            if (mantissa * decade > warp) {
                return mantissa * decade;
            }
        }
        if (decade == kTopDecade) {
            break;
        }
    }
    return warp;
}

WarpGovernor::WarpGovernor(const WarpBudget& budget, std::int64_t requested,
                           std::int64_t step_ns) noexcept
    : budget_(budget), step_ns_(std::max<std::int64_t>(step_ns, 1)),
      requested_(std::max<std::int64_t>(requested, 1)), effective_(requested_) {}

std::optional<double> WarpGovernor::sustainable_warp() const noexcept {
    if (!cost_ns_ || !(*cost_ns_ > 0.0)) {
        return std::nullopt;
    }
    return budget_.load_limit * static_cast<double>(step_ns_) / *cost_ns_;
}

void WarpGovernor::request(std::int64_t warp) noexcept {
    requested_ = std::max<std::int64_t>(warp, 1);
    over_ns_ = 0;
    under_ns_ = 0;
    if (const auto capacity = sustainable_warp()) {
        effective_ = std::min(requested_, warp_level_at_or_below(*capacity));
    } else {
        effective_ = requested_;
    }
}

void WarpGovernor::restart(std::int64_t step_ns) noexcept {
    step_ns_ = std::max<std::int64_t>(step_ns, 1);
    cost_ns_.reset();
    load_.reset();
    over_ns_ = 0;
    under_ns_ = 0;
    effective_ = requested_;
}

std::int64_t WarpGovernor::update(const WarpSample& sample) noexcept {
    if (sample.wall_ns <= 0) {
        return effective_;
    }
    const double busy = static_cast<double>(std::max<std::int64_t>(sample.busy_ns, 0));
    load_ = smooth(load_, busy / static_cast<double>(sample.wall_ns), sample.wall_ns,
                   budget_.smoothing_ns);
    if (sample.steps > 0) {
        cost_ns_ = smooth(cost_ns_, busy / static_cast<double>(sample.steps), sample.wall_ns,
                          budget_.smoothing_ns);
    }
    const auto capacity = sustainable_warp();
    if (!capacity) {
        return effective_;
    }
    if (*capacity < static_cast<double>(effective_)) {
        // Over the limit at the current warp: lower it, after the hold, to
        // the highest level that fits (several levels at once if need be).
        under_ns_ = 0;
        over_ns_ += sample.wall_ns;
        if (over_ns_ >= budget_.lower_after_ns) {
            effective_ = std::min(effective_, warp_level_at_or_below(*capacity));
            over_ns_ = 0;
        }
        return effective_;
    }
    over_ns_ = 0;
    if (effective_ >= requested_) {
        under_ns_ = 0;
        return effective_;
    }
    // Room for a higher level with the margin to spare: raise after the hold
    // to the highest level that keeps the margin.
    const std::int64_t target =
        std::min(requested_, warp_level_at_or_below(*capacity * budget_.raise_margin));
    if (target > effective_) {
        under_ns_ += sample.wall_ns;
        if (under_ns_ >= budget_.raise_after_ns) {
            effective_ = target;
            under_ns_ = 0;
        }
    } else {
        under_ns_ = 0;
    }
    return effective_;
}

} // namespace orrery::sim
