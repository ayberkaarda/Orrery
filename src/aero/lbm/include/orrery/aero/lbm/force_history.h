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

// When a drag and lift history counts as settled (converged).
struct SettledCriteria {
    // Steady: the ranges (max - min) of drag and of lift at most this
    // fraction of the magnitude of the mean drag.
    double steady_tolerance = 1e-3;
    // Periodic: the largest (longest - shortest period) / mean period of
    // the lift, the bound of the cylinder validation.
    double max_period_spread = 0.05;
    // Periodic: the largest relative change of the lift amplitude and of the
    // mean drag between the first and the last lift period.
    double stationarity = 0.02;
};

// True when the drag and lift samples with time >= from (at least two)
// are steady by `criteria.steady_tolerance`.
[[nodiscard]] bool steady_since(std::span<const double> time, std::span<const double> drag,
                                std::span<const double> lift, double from,
                                const SettledCriteria& criteria = {});

// The dominant lift oscillation over the samples with time >= from when it
// is settled periodic shedding: consistent periods (period_spread at most
// `max_period_spread`), and the same lift amplitude (max - min) and mean
// drag over the first and the last period of the analysed span (within
// `stationarity`, relative). Empty for a growing, decaying or irregular
// transient and for a drifting mean drag, even when dominant_oscillation
// finds a period.
[[nodiscard]] std::optional<Oscillation> settled_oscillation(std::span<const double> time,
                                                             std::span<const double> drag,
                                                             std::span<const double> lift,
                                                             double from,
                                                             const SettledCriteria& criteria = {});

} // namespace orrery::aero::lbm
