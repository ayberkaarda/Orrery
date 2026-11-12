#include "orrery/aero/lbm/force_history.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace orrery::aero::lbm {

Coefficients force_coefficients(double force_x, double force_y, double speed,
                                double length) noexcept {
    const double dynamic = 0.5 * speed * speed * length;
    return Coefficients{force_x / dynamic, force_y / dynamic};
}

void CoefficientHistory::push(double time, const Coefficients& sample) {
    time_.push_back(time);
    drag_.push_back(sample.drag);
    lift_.push_back(sample.lift);
}

Coefficients CoefficientHistory::moving_average() const noexcept {
    const std::size_t n = std::min(window_, time_.size());
    if (n == 0) {
        return {};
    }
    Coefficients mean;
    for (std::size_t k = time_.size() - n; k < time_.size(); ++k) {
        mean.drag += drag_[k];
        mean.lift += lift_[k];
    }
    mean.drag /= static_cast<double>(n);
    mean.lift /= static_cast<double>(n);
    return mean;
}

double mean_since(std::span<const double> time, std::span<const double> values,
                  double from) noexcept {
    double sum = 0.0;
    std::size_t count = 0;
    for (std::size_t k = 0; k < time.size() && k < values.size(); ++k) {
        if (time[k] >= from) {
            sum += values[k];
            ++count;
        }
    }
    return count == 0 ? 0.0 : sum / static_cast<double>(count);
}

std::optional<Oscillation> dominant_oscillation(std::span<const double> time,
                                                std::span<const double> values, double from) {
    const std::size_t n = std::min(time.size(), values.size());
    const double mean = mean_since(time, values, from);
    double lo = 0.0;
    double hi = 0.0;
    bool any = false;
    for (std::size_t k = 0; k < n; ++k) {
        if (time[k] >= from) {
            lo = any ? std::min(lo, values[k]) : values[k];
            hi = any ? std::max(hi, values[k]) : values[k];
            any = true;
        }
    }
    // Hysteresis: an upward mean crossing counts only after the signal was
    // below mean - band, so noise around the mean adds no crossings.
    const double band = 0.25 * (hi - lo);
    std::vector<double> crossings;
    bool armed = false;
    for (std::size_t k = 1; k < n; ++k) {
        if (time[k - 1] < from) {
            continue;
        }
        const double a = values[k - 1] - mean;
        const double b = values[k] - mean;
        if (a < -band) {
            armed = true;
        }
        if (armed && a < 0.0 && b >= 0.0) {
            const double fraction = a / (a - b);
            crossings.push_back(time[k - 1] + fraction * (time[k] - time[k - 1]));
            armed = false;
        }
    }
    if (crossings.size() < 3) {
        return std::nullopt;
    }
    double shortest = crossings[1] - crossings[0];
    double longest = shortest;
    for (std::size_t k = 2; k < crossings.size(); ++k) {
        const double p = crossings[k] - crossings[k - 1];
        shortest = std::min(shortest, p);
        longest = std::max(longest, p);
    }
    Oscillation result;
    result.cycles = crossings.size() - 1;
    result.period = (crossings.back() - crossings.front()) / static_cast<double>(result.cycles);
    result.frequency = 1.0 / result.period;
    result.amplitude = 0.5 * (hi - lo);
    result.period_spread = (longest - shortest) / result.period;
    return result;
}

namespace {

// Mean of `drag` and range (max - min) of `lift` over the samples with
// time in [from, to].
struct SpanStats {
    std::size_t count = 0;
    double drag_mean = 0.0;
    double lift_range = 0.0;
    double drag_range = 0.0;
};

SpanStats span_stats(std::span<const double> time, std::span<const double> drag,
                     std::span<const double> lift, double from, double to) noexcept {
    const std::size_t n = std::min({time.size(), drag.size(), lift.size()});
    SpanStats stats;
    double sum = 0.0;
    double lift_lo = 0.0;
    double lift_hi = 0.0;
    double drag_lo = 0.0;
    double drag_hi = 0.0;
    for (std::size_t k = 0; k < n; ++k) {
        if (time[k] < from || time[k] > to) {
            continue;
        }
        const bool first = stats.count == 0;
        lift_lo = first ? lift[k] : std::min(lift_lo, lift[k]);
        lift_hi = first ? lift[k] : std::max(lift_hi, lift[k]);
        drag_lo = first ? drag[k] : std::min(drag_lo, drag[k]);
        drag_hi = first ? drag[k] : std::max(drag_hi, drag[k]);
        sum += drag[k];
        ++stats.count;
    }
    if (stats.count > 0) {
        stats.drag_mean = sum / static_cast<double>(stats.count);
        stats.lift_range = lift_hi - lift_lo;
        stats.drag_range = drag_hi - drag_lo;
    }
    return stats;
}

// |a - b| at most `tolerance` times the larger magnitude.
bool agree(double a, double b, double tolerance) noexcept {
    return std::abs(a - b) <= tolerance * std::max(std::abs(a), std::abs(b));
}

} // namespace

bool steady_since(std::span<const double> time, std::span<const double> drag,
                  std::span<const double> lift, double from, const SettledCriteria& criteria) {
    const SpanStats stats = span_stats(time, drag, lift, from, std::numeric_limits<double>::max());
    if (stats.count < 2) {
        return false;
    }
    const double bound = criteria.steady_tolerance * std::abs(stats.drag_mean);
    return stats.drag_range <= bound && stats.lift_range <= bound;
}

std::optional<Oscillation> settled_oscillation(std::span<const double> time,
                                               std::span<const double> drag,
                                               std::span<const double> lift, double from,
                                               const SettledCriteria& criteria) {
    const auto shedding = dominant_oscillation(time, lift, from);
    if (!shedding || !(shedding->period_spread <= criteria.max_period_spread)) {
        return std::nullopt;
    }
    // First and last sample times of the analysed span.
    const std::size_t n = std::min({time.size(), drag.size(), lift.size()});
    double start = 0.0;
    double end = 0.0;
    bool any = false;
    for (std::size_t k = 0; k < n; ++k) {
        if (time[k] >= from) {
            start = any ? start : time[k];
            end = time[k];
            any = true;
        }
    }
    const double period = shedding->period;
    if (!any || !(end - start >= 2.0 * period)) {
        return std::nullopt;
    }
    // A full period averages the periodic part of the drag out, wherever it
    // starts, so the two means differ only through a drift.
    const SpanStats head = span_stats(time, drag, lift, start, start + period);
    const SpanStats tail = span_stats(time, drag, lift, end - period, end);
    if (head.count == 0 || tail.count == 0 ||
        !agree(head.drag_mean, tail.drag_mean, criteria.stationarity) ||
        !agree(head.lift_range, tail.lift_range, criteria.stationarity)) {
        return std::nullopt;
    }
    return shedding;
}

} // namespace orrery::aero::lbm
