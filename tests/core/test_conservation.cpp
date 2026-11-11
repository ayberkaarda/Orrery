#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

#include "gravity_test_support.h"
#include "orrery/core/conservation.h"
#include "orrery/core/integrator.h"

using namespace orrery::core;
using namespace orrery::core::test;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

const double k2 = constants::gauss_constant_value * constants::gauss_constant_value;

Vec3<space::Length> at(double x, double y, double z) {
    return {space::Length(x), space::Length(y), space::Length(z)};
}

Vec3<space::Velocity> vel(double x, double y, double z) {
    return {space::Velocity(x), space::Velocity(y), space::Velocity(z)};
}

} // namespace

TEST_CASE("conserved quantities of a hand-computed system", "[core][conservation]") {
    BodyStore store;
    add_or_throw(store, BodyInit{"A", Star{}, space::Mass(2.0), at(0, 0, 0), vel(0, -0.5, 0)});
    add_or_throw(store, BodyInit{"B", Planet{}, space::Mass(1.0), at(3, 0, 0), vel(0, 1.0, 0)});
    // The probe is excluded from every total.
    add_or_throw(store, BodyInit{"P", Probe{}, space::Mass(5.0), at(1, 1, 1), vel(0.3, 0.2, 0.1)});
    const SimState state(space::Time(0.0), store);
    const NewtonianGravity gravity;
    const ConservedQuantities q = ConservationDiagnostics::measure(state, gravity);

    CHECK(q.total_mass.value() == 3.0);
    CHECK_THAT(q.kinetic_energy.value(), WithinRel(0.5 * 2.0 * 0.25 + 0.5 * 1.0 * 1.0, 1e-15));
    CHECK_THAT(q.potential_energy.value(), WithinRel(-k2 * 2.0 * 1.0 / 3.0, 1e-15));
    CHECK_THAT(q.total_energy.value(),
               WithinRel(q.kinetic_energy.value() + q.potential_energy.value(), 1e-15));
    CHECK(q.linear_momentum.x.value() == 0.0);
    CHECK(q.linear_momentum.y.value() == 0.0);
    // L_z = m_B x_B v_B = 3.
    CHECK(q.angular_momentum.z.value() == 3.0);
    CHECK(q.center_of_mass.x.value() == 1.0);
    CHECK(q.center_of_mass_velocity == Vec3<space::Velocity>{});
    CHECK(q.momentum_scale.value() == 2.0);
    CHECK(q.angular_momentum_scale.value() == 3.0);

    // Softened potential matches the force model.
    const auto soft = NewtonianGravity::create(space::Length(4.0));
    REQUIRE(soft);
    CHECK_THAT(ConservationDiagnostics::measure(state, soft.value()).potential_energy.value(),
               WithinRel(-k2 * 2.0 / 5.0, 1e-15));
}

TEST_CASE("drift is zero for the baseline and normalised as documented", "[core][conservation]") {
    BodyStore store;
    add_or_throw(store, BodyInit{"A", Star{}, space::Mass(2.0), at(0, 0, 0), vel(0, -0.5, 0)});
    add_or_throw(store, BodyInit{"B", Planet{}, space::Mass(1.0), at(3, 0, 0), vel(0, 1.0, 0)});
    SimState state(space::Time(0.0), store);
    const NewtonianGravity gravity;
    const ConservationDiagnostics diagnostics(state, gravity);
    const ConservationDrift zero = diagnostics.drift(state, gravity);
    CHECK(zero.relative_energy == 0.0);
    CHECK(zero.linear_momentum == 0.0);
    CHECK(zero.angular_momentum == 0.0);
    CHECK(zero.center_of_mass_velocity == 0.0);

    // Kick B by +0.1 in y: dP = 0.1, scale sum m|v| = 2; dL_z = 0.3, scale 3.
    REQUIRE(state.bodies().set_velocity(1, vel(0, 1.1, 0)));
    const ConservationDrift d = diagnostics.drift(state, gravity);
    CHECK_THAT(d.linear_momentum, WithinRel(0.05, 1e-14));
    CHECK_THAT(d.angular_momentum, WithinRel(0.1, 1e-14));
    CHECK_THAT(d.center_of_mass_velocity, WithinRel(0.05, 1e-14));
    const double e0 = diagnostics.baseline().total_energy.value();
    CHECK_THAT(d.relative_energy, WithinRel(0.5 * (1.21 - 1.0) / std::abs(e0), 1e-12));

    // Zero baselines: drift is 0 when unchanged and infinite otherwise.
    SimState empty;
    const ConservationDiagnostics none(empty, gravity);
    CHECK(none.drift(empty, gravity).relative_energy == 0.0);
    CHECK(none.drift(state, gravity).relative_energy == std::numeric_limits<double>::infinity());
    CHECK(none.drift(state, gravity).linear_momentum == std::numeric_limits<double>::infinity());
}

TEST_CASE("momentum drifts are scale invariant for tiny and huge momenta", "[core][conservation]") {
    // One star at (0, 1, 0) AU; the velocity changes from (1, 0, 0) to
    // (2, 0, 0) AU/day: a 100 percent change of P, L and V_cm. With m = 1e-200
    // the squared components underflow, with m = 1e200 they overflow.
    for (const double mass : {1.0e-200, 1.0e200}) {
        BodyStore store;
        add_or_throw(store, BodyInit{"S", Star{}, space::Mass(mass), at(0, 1, 0), vel(1, 0, 0)});
        SimState state(space::Time(0.0), store);
        const NewtonianGravity gravity;
        const ConservationDiagnostics diagnostics(state, gravity);
        CHECK_THAT(diagnostics.baseline().momentum_scale.value(), WithinRel(mass, 1e-15));
        CHECK_THAT(diagnostics.baseline().angular_momentum_scale.value(), WithinRel(mass, 1e-15));
        REQUIRE(state.bodies().set_velocity(0, vel(2, 0, 0)));
        const ConservationDrift d = diagnostics.drift(state, gravity);
        CHECK_THAT(d.linear_momentum, WithinRel(1.0, 1e-14));
        CHECK_THAT(d.angular_momentum, WithinRel(1.0, 1e-14));
        CHECK_THAT(d.center_of_mass_velocity, WithinRel(1.0, 1e-14));
    }
}

TEST_CASE("running maximum of a sampled diagnostic does not discard NaN", "[core][conservation]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    RunningMax clean;
    clean.add(1.0e-12);
    clean.add(3.0e-12);
    CHECK(clean.value == 3.0e-12);
    CHECK(clean.all_finite());
    CHECK(clean.below(1.0e-9));
    CHECK_FALSE(clean.below(2.0e-12));
    // A sporadic NaN between finite samples, NaN only, and an infinite sample.
    for (const std::array<double, 3>& samples :
         {std::array<double, 3>{1.0e-12, nan, 1.0e-13}, std::array<double, 3>{nan, nan, nan},
          std::array<double, 3>{1.0e-12, inf, 1.0e-13}}) {
        RunningMax m;
        for (const double s : samples) {
            m.add(s);
        }
        CHECK_FALSE(m.below(1.0e-9));
    }
}

TEST_CASE("Sun and eight planets, 1000 years with Yoshida4: |dE/E0| below 1e-9",
          "[core][conservation][acceptance]") {
    SimState state = solar_system();
    const NewtonianGravity gravity;
    const ConservationDiagnostics diagnostics(state, gravity);
    Yoshida4 integrator;

    // dt = 1/4 day: Mercury's 88-day orbit (e = 0.21) is resolved with about
    // 352 steps per period. With dt = 1/2 day the maximum |dE/E0| is 2.2e-9,
    // dominated by Mercury's periapsis passages (docs/physics.md).
    // 1000 Julian years = 365250 days.
    const double dt_days = 0.25;
    const auto steps = static_cast<long long>(365250.0 / dt_days);
    const long long sample_every = 50;
    RunningMax max_energy;
    RunningMax max_momentum;
    RunningMax max_angular;
    const auto start = std::chrono::steady_clock::now();
    for (long long s = 1; s <= steps; ++s) {
        step_or_throw(integrator, state, space::Time(dt_days), gravity);
        if (s % sample_every == 0 || s == steps) {
            const ConservationDrift d = diagnostics.drift(state, gravity);
            max_energy.add(d.relative_energy);
            max_momentum.add(d.linear_momentum);
            max_angular.add(d.angular_momentum);
        }
    }
    const double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    const ConservationDrift final_drift = diagnostics.drift(state, gravity);
    report("solar system (Sun + 8 planets) Yoshida4 dt=" + sci(dt_days) + " d, " +
           std::to_string(steps) + " steps (1000 yr), " + std::to_string(seconds) +
           " s: max |dE/E0| " + sci(max_energy.value) + " (final " +
           sci(final_drift.relative_energy) + "), max momentum drift " + sci(max_momentum.value) +
           ", max angular momentum drift " + sci(max_angular.value));
    CHECK(state.time().value() == 365250.0);
    CHECK(max_energy.below(1.0e-9));
    CHECK(std::isfinite(final_drift.relative_energy));
    CHECK(std::isfinite(final_drift.linear_momentum));
    CHECK(std::isfinite(final_drift.angular_momentum));
    // Momentum and angular momentum: conserved to rounding. Rounding errors of
    // successive steps are uncorrelated, so they accumulate as a random walk,
    // about DBL_EPSILON * sqrt(steps) (1.3e-13 here); the bound allows ten
    // times that.
    const double rounding_bound =
        10.0 * std::numeric_limits<double>::epsilon() * std::sqrt(static_cast<double>(steps));
    report("rounding bound for momentum drifts " + sci(rounding_bound));
    CHECK(max_momentum.below(rounding_bound));
    CHECK(max_angular.below(rounding_bound));
}
