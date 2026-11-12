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
