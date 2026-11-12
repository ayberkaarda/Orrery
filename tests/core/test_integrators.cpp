#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include "gravity_test_support.h"
#include "orrery/core/conservation.h"
#include "orrery/core/integrator.h"
#include "orrery/core/state_hash.h"

using namespace orrery::core;
using namespace orrery::core::test;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

constexpr double earth_mass = 3.0e-6;

space::Time period_of(const KeplerElements& el, space::GravitationalParameter mu) {
    const auto t = orbital_period(el.semi_major_axis, mu);
    REQUIRE(t);
    return t.value();
}

// Analytic relative position at time t: Kepler propagation of the mean
// anomaly, M(t) = M0 + n t.
Vec3<space::Length> analytic_position(const KeplerElements& el, space::GravitationalParameter mu,
                                      space::Time t) {
    const auto n = mean_motion(el.semi_major_axis, mu);
    REQUIRE(n);
    KeplerElements at_t = el;
    at_t.mean_anomaly = wrap_two_pi(el.mean_anomaly + Angle::from_radians((n.value() * t).value()));
    return to_cartesian_or_throw(at_t, mu).position;
}

// Position error (AU) after integrating one or more periods of a two-body
// orbit with `steps` fixed steps per period.
double one_orbit_error(IntegratorKind kind, const KeplerElements& el, long long steps,
                       int orbits = 1) {
    const auto mu = two_body_mu(1.0, earth_mass);
    const space::Time period = period_of(el, mu);
    const space::Time dt = period / static_cast<double>(steps);
    SimState state = two_body(1.0, earth_mass, el);
    const auto integrator = make_integrator(kind);
    advance(*integrator, state, dt, steps * orbits, NewtonianGravity{});
    return norm(relative_position(state) - analytic_position(el, mu, state.time())).value();
}

} // namespace

TEST_CASE("integrator properties and default", "[core][integrator]") {
    static_assert(default_integrator == IntegratorKind::leapfrog_kdk);
    const auto leapfrog = make_integrator(IntegratorKind::leapfrog_kdk);
    const auto yoshida = make_integrator(IntegratorKind::yoshida4);
    const auto rk4 = make_integrator(IntegratorKind::rk4);
    CHECK(leapfrog->name() == "leapfrog_kdk");
    CHECK(leapfrog->order() == 2);
    CHECK(leapfrog->is_symplectic());
    CHECK(leapfrog->force_evaluations_per_step() == 2);
    CHECK(yoshida->name() == "yoshida4");
    CHECK(yoshida->order() == 4);
    CHECK(yoshida->is_symplectic());
    CHECK(yoshida->force_evaluations_per_step() == 4);
    CHECK(rk4->name() == "rk4");
    CHECK(rk4->order() == 4);
    CHECK_FALSE(rk4->is_symplectic());
    CHECK(make_integrator(default_integrator)->name() == "leapfrog_kdk");

    // Yoshida coefficients: 2 w1 + w0 = 1 and the third-order condition
    // 2 w1^3 + w0^3 = 0.
    CHECK_THAT(Yoshida4::cube_root_two, WithinRel(std::cbrt(2.0), 2.3e-16));
    CHECK_THAT(2.0 * Yoshida4::w1 + Yoshida4::w0, WithinAbs(1.0, 1e-15));
    CHECK_THAT(2.0 * std::pow(Yoshida4::w1, 3) + std::pow(Yoshida4::w0, 3), WithinAbs(0.0, 1e-14));
    CHECK_THAT(Yoshida4::w1, WithinRel(1.3512071919596578, 1e-15));
    CHECK_THAT(Yoshida4::w0, WithinRel(-1.7024143839193153, 1e-15));
}

TEST_CASE("two-body orbit matches the analytic Kepler solution at several times",
          "[core][integrator][acceptance]") {
    const KeplerElements el = elements(1.3, 0.3, 12.0, 40.0, 70.0, 25.0);
    const auto mu = two_body_mu(1.0, earth_mass);
    const space::Time period = period_of(el, mu);
    const long long steps_per_orbit = 2000;
    const space::Time dt = period / static_cast<double>(steps_per_orbit);

    struct Case {
        IntegratorKind kind;
        double tolerance;
    };
    for (const Case c :
         {Case{IntegratorKind::yoshida4, 5.0e-8}, Case{IntegratorKind::leapfrog_kdk, 1.0e-3},
          Case{IntegratorKind::rk4, 2.0e-9}}) {
        SimState state = two_body(1.0, earth_mass, el);
        const auto integrator = make_integrator(c.kind);
        long long done = 0;
        // t = T/4, T/2, T, 10 T.
        for (const long long target :
             {steps_per_orbit / 4, steps_per_orbit / 2, steps_per_orbit, 10 * steps_per_orbit}) {
            advance(*integrator, state, dt, target - done, NewtonianGravity{});
            done = target;
            const Vec3<space::Length> expected = analytic_position(el, mu, state.time());
            const double error = norm(relative_position(state) - expected).value();
            report(
                std::string(integrator->name()) + " two-body a=1.3 e=0.3 dt=T/2000 t=" +
                std::to_string(static_cast<double>(target) / static_cast<double>(steps_per_orbit)) +
                "T position error " + sci(error) + " AU");
            CHECK(error < c.tolerance);
        }
    }
}

TEST_CASE("measured Kepler period matches 2 pi sqrt(a^3 / mu)", "[core][integrator][acceptance]") {
    // Periapsis on +x, motion counter-clockwise: the relative orbit crosses
    // y = 0 upwards (x > 0) once per period, at periapsis.
    const KeplerElements el = elements(1.0, 0.5, 0.0, 0.0, 0.0, 180.0);
    const auto mu = two_body_mu(1.0, earth_mass);
    const space::Time period = period_of(el, mu);
    const space::Time dt = period / 1000.0;
    SimState state = two_body(1.0, earth_mass, el);
    Yoshida4 integrator;
    std::vector<double> crossings;
    Vec3<space::Length> previous = relative_position(state);
    while (crossings.size() < 6) {
        const double t0 = state.time().value();
        step_or_throw(integrator, state, dt, NewtonianGravity{});
        const Vec3<space::Length> current = relative_position(state);
        if (previous.y.value() < 0.0 && current.y.value() >= 0.0 && current.x.value() > 0.0) {
            // Linear interpolation of the crossing time (y'' = 0 at y = 0 on
            // the apsidal line, so the interpolation error is third order).
            const double f = previous.y.value() / (previous.y.value() - current.y.value());
            crossings.push_back(t0 + f * dt.value());
        }
        previous = current;
    }
    for (std::size_t k = 1; k < crossings.size(); ++k) {
        const double measured = crossings[k] - crossings[k - 1];
        const double rel = std::abs(measured - period.value()) / period.value();
        report("yoshida4 Kepler period orbit " + std::to_string(k) + ": measured " +
               std::to_string(measured) + " d, analytic " + std::to_string(period.value()) +
               " d, relative error " + sci(rel));
        CHECK(rel < 1.0e-9);
    }
}

TEST_CASE("circular two-body orbit, 100 orbits with Leapfrog: position error below 1e-6 AU",
          "[core][integrator][acceptance]") {
    const KeplerElements el = elements(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const long long steps_per_orbit = 150000;
    const double error = one_orbit_error(IntegratorKind::leapfrog_kdk, el, steps_per_orbit, 100);
    report("leapfrog circular a=1 AU, 100 orbits, dt=T/" + std::to_string(steps_per_orbit) + " (" +
           sci(365.2568983 / static_cast<double>(steps_per_orbit)) + " d): position error " +
           sci(error) + " AU");
    CHECK(error < 1.0e-6);

    // For comparison: the fourth-order scheme meets the same target with
    // 125 times fewer steps.
    const double yoshida_error = one_orbit_error(IntegratorKind::yoshida4, el, 1200, 100);
    report("yoshida4 circular a=1 AU, 100 orbits, dt=T/1200: position error " + sci(yoshida_error) +
           " AU");
    CHECK(yoshida_error < 1.0e-6);
}

TEST_CASE("convergence order: Leapfrog 2, Yoshida4 4, RK4 4", "[core][integrator][acceptance]") {
    const KeplerElements el = elements(1.0, 0.5, 5.0, 30.0, 60.0, 0.0);
    struct Case {
        IntegratorKind kind;
        long long base_steps;
        double expected_order;
    };
    for (const Case c :
         {Case{IntegratorKind::leapfrog_kdk, 2000, 2.0}, Case{IntegratorKind::yoshida4, 400, 4.0},
          Case{IntegratorKind::rk4, 400, 4.0}}) {
        std::array<double, 3> errors{};
        for (std::size_t k = 0; k < errors.size(); ++k) {
            errors[k] = one_orbit_error(c.kind, el, c.base_steps << k);
        }
        const double order_1 = std::log2(errors[0] / errors[1]);
        const double order_2 = std::log2(errors[1] / errors[2]);
        report(std::string(make_integrator(c.kind)->name()) +
               " e=0.5 one orbit, N=" + std::to_string(c.base_steps) + "/" +
               std::to_string(2 * c.base_steps) + "/" + std::to_string(4 * c.base_steps) +
               " steps: errors " + sci(errors[0]) + " " + sci(errors[1]) + " " + sci(errors[2]) +
               " AU, observed order " + std::to_string(order_1) + " " + std::to_string(order_2));
        CHECK_THAT(order_1, WithinAbs(c.expected_order, 0.15));
        CHECK_THAT(order_2, WithinAbs(c.expected_order, 0.15));
    }
}

TEST_CASE("RK4 energy drifts secularly while symplectic energy error stays bounded",
          "[core][integrator][acceptance]") {
    const KeplerElements el = elements(1.0, 0.5, 0.0, 0.0, 0.0, 0.0);
    const auto mu = two_body_mu(1.0, earth_mass);
    const space::Time period = period_of(el, mu);
    const long long steps_per_orbit = 200;
    const space::Time dt = period / static_cast<double>(steps_per_orbit);
    const NewtonianGravity gravity;

    for (const IntegratorKind kind :
         {IntegratorKind::rk4, IntegratorKind::leapfrog_kdk, IntegratorKind::yoshida4}) {
        SimState state = two_body(1.0, earth_mass, el);
        const ConservationDiagnostics diagnostics(state, gravity);
        const auto integrator = make_integrator(kind);
        // Signed relative energy error at the end of orbits 100 and 1000, and
        // the largest error over the first and the last 100 orbits.
        RunningMax max_first;
        RunningMax max_last;
        double at_100 = 0.0;
        double at_1000 = 0.0;
        const double e0 = diagnostics.baseline().total_energy.value();
        for (long long orbit = 1; orbit <= 1000; ++orbit) {
            for (long long s = 0; s < steps_per_orbit; ++s) {
                step_or_throw(*integrator, state, dt, gravity);
                const double de =
                    std::abs(ConservationDiagnostics::measure(state, gravity).total_energy.value() -
                             e0) /
                    std::abs(e0);
                if (orbit <= 100) {
                    max_first.add(de);
                }
                if (orbit > 900) {
                    max_last.add(de);
                }
            }
            if (orbit == 100) {
                at_100 = diagnostics.drift(state, gravity).relative_energy;
            }
            if (orbit == 1000) {
                at_1000 = diagnostics.drift(state, gravity).relative_energy;
            }
        }
        report(std::string(integrator->name()) + " e=0.5 dt=T/200: |dE/E0| after 100 orbits " +
               sci(at_100) + ", after 1000 " + sci(at_1000) + "; max over orbits 1-100 " +
               sci(max_first.value) + ", 901-1000 " + sci(max_last.value));
        CHECK(max_first.all_finite());
        CHECK(max_last.all_finite());
        CHECK(std::isfinite(at_100));
        CHECK(std::isfinite(at_1000));
        if (kind == IntegratorKind::rk4) {
            // Secular: the error at the period boundary grows about linearly.
            CHECK(at_1000 > 8.0 * at_100);
            CHECK(at_1000 < 12.0 * at_100);
        } else {
            // Bounded: no growth between the first and the last 100 orbits.
            CHECK(max_last.below(1.05 * max_first.value));
        }
    }
}

TEST_CASE("symplectic integrators are time-reversible", "[core][integrator]") {
    const KeplerElements el = elements(1.0, 0.6, 10.0, 20.0, 30.0, 40.0);
    const auto mu = two_body_mu(1.0, earth_mass);
    const space::Time dt = period_of(el, mu) / 500.0;
    for (const IntegratorKind kind : {IntegratorKind::leapfrog_kdk, IntegratorKind::yoshida4}) {
        const SimState initial = two_body(1.0, earth_mass, el);
        SimState state = initial;
        const auto integrator = make_integrator(kind);
        advance(*integrator, state, dt, 1000, NewtonianGravity{});
        advance(*integrator, state, -dt, 1000, NewtonianGravity{});
        const double error = norm(relative_position(state) - relative_position(initial)).value();
        report(std::string(integrator->name()) + " 1000 steps forward and back: position error " +
               sci(error) + " AU");
        CHECK(error < 1.0e-12);
        CHECK_THAT(state.time().value(), WithinAbs(0.0, 1e-9));
    }
}

TEST_CASE("step reports errors and leaves the state unchanged", "[core][integrator]") {
    const NewtonianGravity gravity;
    for (const IntegratorKind kind :
         {IntegratorKind::leapfrog_kdk, IntegratorKind::yoshida4, IntegratorKind::rk4}) {
        const auto integrator = make_integrator(kind);

        SimState state = two_body(1.0, earth_mass, elements(1.0, 0.1, 0, 0, 0, 0));
        const std::uint64_t before = state_hash(state);
        for (const double bad :
             {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
              -std::numeric_limits<double>::infinity()}) {
            const auto r = integrator->step(state, space::Time(bad), gravity);
            REQUIRE_FALSE(r);
            CHECK(r.error() == IntegrationError::invalid_time_step);
            CHECK(state_hash(state) == before);
        }

        // Coincident sources without softening: non-finite accelerations.
        BodyStore store;
        add_or_throw(store, BodyInit{"A", Star{}, space::Mass(1.0), {}, {}});
        add_or_throw(store, BodyInit{"B", Planet{}, space::Mass(1.0e-3), {}, {}});
        SimState collapsed(space::Time(5.0), std::move(store));
        const std::uint64_t collapsed_before = state_hash(collapsed);
        const auto r = integrator->step(collapsed, space::Time(1.0), gravity);
        REQUIRE_FALSE(r);
        CHECK(r.error() == IntegrationError::non_finite_state);
        CHECK(state_hash(collapsed) == collapsed_before);

        // Success advances the time by dt and returns it.
        const auto ok = integrator->step(state, space::Time(0.25), gravity);
        REQUIRE(ok);
        CHECK(ok.value() == space::Time(0.25));
        CHECK(state.time() == space::Time(0.25));

        // An empty state is a valid no-op apart from the time.
        SimState empty;
        REQUIRE(integrator->step(empty, space::Time(2.0), gravity));
        CHECK(empty.time() == space::Time(2.0));
    }
    CHECK(to_string(IntegrationError::invalid_time_step).size() > 0);
    CHECK(to_string(IntegrationError::non_finite_state).size() > 0);
}

TEST_CASE("probes are test particles under integration", "[core][integrator]") {
    // The gravitating bodies evolve bit-identically with and without a probe.
    const SimState without = two_body(1.0, 1.0e-3, elements(5.2, 0.05, 1.3, 100.0, 14.0, 34.0));
    SimState with = without;
    add_or_throw(with.bodies(), BodyInit{"Probe",
                                         Probe{AeroRef{2}},
                                         space::Mass(0.5),
                                         {space::Length(1.0), {}, {}},
                                         {{}, space::Velocity(0.0172), {}}});
    for (const IntegratorKind kind :
         {IntegratorKind::leapfrog_kdk, IntegratorKind::yoshida4, IntegratorKind::rk4}) {
        SimState a = without;
        SimState b = with;
        const auto i1 = make_integrator(kind);
        const auto i2 = make_integrator(kind);
        advance(*i1, a, space::Time(2.0), 2000, NewtonianGravity{});
        advance(*i2, b, space::Time(2.0), 2000, NewtonianGravity{});
        for (std::size_t k = 0; k < 2; ++k) {
            CHECK(a.bodies().position(k) == b.bodies().position(k));
            CHECK(a.bodies().velocity(k) == b.bodies().velocity(k));
        }
        CHECK(norm(b.bodies().position(2) - with.bodies().position(2)).value() > 0.0);
    }
}

TEST_CASE("direct summation reference size: N = 256", "[core][integrator][gravity]") {
    // Deterministic pseudo-random cluster (64-bit LCG, Knuth MMIX constants).
    std::uint64_t seed = 12345;
    const auto uniform = [&seed]() {
        seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(seed >> 11U) * (1.0 / 9007199254740992.0) * 2.0 - 1.0;
    };
    BodyStore store;
    for (int i = 0; i < 256; ++i) {
        const Vec3<space::Length> r{space::Length(uniform()), space::Length(uniform()),
                                    space::Length(uniform())};
        const Vec3<space::Velocity> v{space::Velocity(1e-3 * uniform()),
                                      space::Velocity(1e-3 * uniform()),
                                      space::Velocity(1e-3 * uniform())};
        add_or_throw(store,
                     BodyInit{"b" + std::to_string(i), Asteroid{}, space::Mass(1.0 / 256.0), r, v});
    }
    SimState state(space::Time(0.0), std::move(store));
    const auto gravity = NewtonianGravity::create(space::Length(0.05));
    REQUIRE(gravity);
    const ConservationDiagnostics diagnostics(state, gravity.value());
    LeapfrogKdk integrator;
    const int steps = 200;
    const auto start = std::chrono::steady_clock::now();
    advance(integrator, state, space::Time(0.1), steps, gravity.value());
    const auto elapsed =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    const ConservationDrift drift = diagnostics.drift(state, gravity.value());
    report("N=256 softened cluster (eps 0.05 AU), leapfrog 200 steps of 0.1 d: " +
           sci(elapsed / (2.0 * steps)) + " ms per force evaluation, |dE/E0| " +
           sci(drift.relative_energy) + ", momentum drift " + sci(drift.linear_momentum) +
           ", angular momentum drift " + sci(drift.angular_momentum));
    CHECK(drift.relative_energy < 1.0e-3);
    CHECK(drift.linear_momentum < 1.0e-13);
    CHECK(drift.angular_momentum < 1.0e-13);
}
