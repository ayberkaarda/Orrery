#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>
#include <vector>

#include "orrery/aero/lbm/force_history.h"

using namespace orrery::aero::lbm;
using Catch::Approx;

TEST_CASE("Force coefficients use the dynamic pressure per unit span", "[aero][lbm]") {
    const Coefficients c = force_coefficients(0.0135, -0.002, 0.05, 20.0);
    CHECK(c.drag == Approx(0.0135 / (0.5 * 0.0025 * 20.0)));
    CHECK(c.lift == Approx(-0.002 / (0.5 * 0.0025 * 20.0)));
}

TEST_CASE("The moving average covers the last window samples", "[aero][lbm]") {
    CoefficientHistory history(3);
    CHECK(history.moving_average().drag == 0.0);
    history.push(0.0, {1.0, 0.0});
    CHECK(history.moving_average().drag == 1.0);
    history.push(1.0, {2.0, 1.0});
    history.push(2.0, {3.0, -1.0});
    history.push(3.0, {7.0, 3.0});
    CHECK(history.size() == 4);
    CHECK(history.moving_average().drag == Approx(4.0));
    CHECK(history.moving_average().lift == Approx(1.0));
    CHECK(CoefficientHistory(0).window() == 1);
}

TEST_CASE("The dominant oscillation of a shedding-like signal", "[aero][lbm]") {
    // Lift-like sine with a decaying start-up transient and a drag offset.
    const double frequency = 1.0 / 2425.0;
    std::vector<double> t;
    std::vector<double> v;
    for (int k = 0; k < 40000; k += 10) {
        const auto time = static_cast<double>(k);
        t.push_back(time);
        v.push_back(1.3 + 0.3 * std::sin(2.0 * std::numbers::pi * frequency * time + 0.4) +
                    std::exp(-time / 500.0));
    }
    const auto found = dominant_oscillation(t, v, 10000.0);
    REQUIRE(found);
    CHECK(found->frequency == Approx(frequency).epsilon(2e-4));
    CHECK(found->cycles >= 10);
    CHECK(found->amplitude == Approx(0.3).epsilon(1e-3));
    CHECK(found->period_spread < 0.01);
    // 12.4 periods: the incomplete one biases the plain mean slightly.
    CHECK(mean_since(t, v, 10000.0) == Approx(1.3).epsilon(1e-2));

    const std::vector<double> flat(t.size(), 1.0);
    CHECK_FALSE(dominant_oscillation(t, flat, 0.0));
}

namespace {

// Lift sine of period 1000 with amplitude a(t) and drag d(t), sampled every
// 10 time units over [0, 20000).
struct Signal {
    std::vector<double> t;
    std::vector<double> drag;
    std::vector<double> lift;
};

template <typename Amplitude, typename Drag, typename Phase>
Signal shedding(const Amplitude& amplitude, const Drag& drag, const Phase& phase) {
    Signal s;
    for (int k = 0; k < 20000; k += 10) {
        const auto time = static_cast<double>(k);
        s.t.push_back(time);
        s.drag.push_back(drag(time));
        s.lift.push_back(amplitude(time) * std::sin(2.0 * std::numbers::pi * phase(time)));
    }
    return s;
}

} // namespace

TEST_CASE("Settled shedding needs consistent periods and stationary amplitude and drag",
          "[aero][lbm]") {
    const auto constant = [](double) { return 0.4; };
    const auto mean_drag = [](double time) {
        return 1.4 + 0.01 * std::sin(4.0 * std::numbers::pi * time / 1000.0);
    };
    const auto regular = [](double time) { return time / 1000.0; };

    // Locked-in shedding: accepted, at the right period.
    const Signal locked = shedding(constant, mean_drag, regular);
    const auto found = settled_oscillation(locked.t, locked.drag, locked.lift, 10000.0);
    REQUIRE(found);
    CHECK(found->period == Approx(1000.0).epsilon(1e-3));

    // Amplitude growing by 10 percent per period (a start-up transient):
    // the periods are regular, the amplitude is not.
    const Signal growing = shedding(
        [](double time) { return 0.4 * std::exp((time - 20000.0) / 10500.0); }, mean_drag, regular);
    REQUIRE(dominant_oscillation(growing.t, growing.lift, 10000.0));
    CHECK_FALSE(settled_oscillation(growing.t, growing.drag, growing.lift, 10000.0));

    // Phase jitter of 0.15 cycles: periods between about 700 and 1300.
    const auto irregular = [](double time) {
        return time / 1000.0 + 0.15 * std::sin(2.0 * std::numbers::pi * time / 2700.0);
    };
    const Signal uneven = shedding(constant, mean_drag, irregular);
    REQUIRE(dominant_oscillation(uneven.t, uneven.lift, 10000.0));
    CHECK_FALSE(settled_oscillation(uneven.t, uneven.drag, uneven.lift, 10000.0));

    // Regular lift over a mean drag still falling by 10 percent.
    const Signal drifting =
        shedding(constant, [](double time) { return 1.5 - 1.5e-5 * time; }, regular);
    REQUIRE(dominant_oscillation(drifting.t, drifting.lift, 10000.0));
    CHECK_FALSE(settled_oscillation(drifting.t, drifting.drag, drifting.lift, 10000.0));

    // Fewer than two periods analysed.
    CHECK_FALSE(settled_oscillation(locked.t, locked.drag, locked.lift, 18500.0));
}

TEST_CASE("A steady history has drag and lift ranges below the tolerance", "[aero][lbm]") {
    std::vector<double> t;
    std::vector<double> drag;
    std::vector<double> lift;
    for (int k = 0; k < 2000; ++k) {
        const auto time = static_cast<double>(k);
        t.push_back(time);
        drag.push_back(1.2 + 0.5 * std::exp(-time / 100.0));
        lift.push_back(1e-12 * std::sin(time));
    }
    CHECK(steady_since(t, drag, lift, 1500.0));       // decay below 1e-3 of the drag
    CHECK_FALSE(steady_since(t, drag, lift, 500.0));  // still decaying
    CHECK_FALSE(steady_since(t, drag, lift, 1999.5)); // fewer than two samples
    std::vector<double> waving = lift;
    for (std::size_t k = 0; k < waving.size(); ++k) {
        waving[k] = 0.01 * std::sin(static_cast<double>(k) / 50.0);
    }
    CHECK_FALSE(steady_since(t, drag, waving, 1500.0)); // lift oscillates
}