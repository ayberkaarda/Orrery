#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

// Time series of force coefficients: instantaneous values, a moving average
// over the last `window` samples, and the dominant oscillation (vortex
// shedding) for the Strouhal number St = f L / U.
namespace orrery::aero::lbm {

// Drag and lift coefficients of a 2D force (per unit span), lattice units:
// C = F / (rho U^2 L / 2) with rho = 1.
struct Coefficients {
    double drag = 0.0;
    double lift = 0.0;
};
[[nodiscard]] Coefficients force_coefficients(double force_x, double force_y, double speed,
                                              double length) noexcept;

class CoefficientHistory {
public:
    // window >= 1 samples for the moving average.
    explicit CoefficientHistory(std::size_t window) noexcept : window_(window < 1 ? 1 : window) {}

    void push(double time, const Coefficients& sample);

    [[nodiscard]] std::size_t size() const noexcept { return time_.size(); }
    [[nodiscard]] std::size_t window() const noexcept { return window_; }
    [[nodiscard]] std::span<const double> time() const noexcept { return time_; }
    [[nodiscard]] std::span<const double> drag() const noexcept { return drag_; }
    [[nodiscard]] std::span<const double> lift() const noexcept { return lift_; }
    // Mean of the last min(window, size) samples; zero when empty.
    [[nodiscard]] Coefficients moving_average() const noexcept;

private:
    std::size_t window_ = 1;
    std::vector<double> time_;
    std::vector<double> drag_;
    std::vector<double> lift_;
};

// Mean of `values` over the samples with time >= from.
[[nodiscard]] double mean_since(std::span<const double> time, std::span<const double> values,
                                double from) noexcept;

struct Oscillation {
    double period = 0.0;
    double frequency = 0.0;
    std::size_t cycles = 0;     // complete periods between the first and last crossing
    double amplitude = 0.0;     // (max - min) / 2 over the analysed samples
    double period_spread = 0.0; // (longest - shortest period) / mean period
};

// Period of `values` from its upward crossings of its mean, located by linear
// interpolation, over the samples with time >= from. A crossing counts only
// after the signal was a quarter of its peak-to-peak range below the mean
// (hysteresis against noise). Empty when fewer than two complete periods are
// found.
[[nodiscard]] std::optional<Oscillation>
dominant_oscillation(std::span<const double> time, std::span<const double> values, double from);

} // namespace orrery::aero::lbm
