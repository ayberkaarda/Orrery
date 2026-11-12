#include "orrery/aero/lbm/force_history.h"

#include <algorithm>

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

} // namespace orrery::aero::lbm
