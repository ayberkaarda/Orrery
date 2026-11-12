#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <span>
#include <string>
#include <vector>

#include "gravity_test_support.h"
#include "orrery/core/conservation.h"
#include "orrery/core/ias15.h"
#include "orrery/core/integrator.h"
#include "orrery/core/kepler.h"
#include "orrery/core/state_hash.h"

using namespace orrery::core;
using namespace orrery::core::test;

namespace {

double kepler_period(double a, double m_total) {
    return 2.0 * std::numbers::pi * std::sqrt(a * a * a / two_body_mu(m_total, 0.0).value());
}

// Relative position of the secondary predicted by the two-body solution after
// time t from the elements `el` (mean anomaly advanced by n t).
Vec3<space::Length> analytic_relative(KeplerElements el, double m1, double m2, double t) {
    const double a = el.semi_major_axis.value();
    const double n = 2.0 * std::numbers::pi / kepler_period(a, m1 + m2);
    el.mean_anomaly = Angle::from_radians(el.mean_anomaly.radians() + n * t);
    return to_cartesian_or_throw(el, two_body_mu(m1, m2)).position;
}

double energy(const SimState& state, const IForceModel& forces) {
    return ConservationDiagnostics::measure(state, forces).total_energy.value();
}

std::string hex(std::uint64_t value) {
    std::array<char, 24> buffer{};
    std::snprintf(buffer.data(), buffer.size(), "0x%016llx",
                  static_cast<unsigned long long>(value));
    return {buffer.data()};
}

} // namespace

TEST_CASE("IAS15 tolerance validation and metadata", "[core][ias15]") {
    CHECK(IAS15::create(1e-9));
    CHECK(IAS15::create(0.0).error() == Ias15Error::invalid_tolerance);
    CHECK(IAS15::create(-1.0).error() == Ias15Error::invalid_tolerance);
    CHECK(IAS15::create(std::nan("")).error() == Ias15Error::invalid_tolerance);
    const IAS15 ias;
    CHECK(ias.name() == "ias15");
    CHECK(ias.order() == 15);
    CHECK_FALSE(ias.is_symplectic());
    CHECK(ias.tolerance() == IAS15::default_tolerance);
    CHECK(to_string(IntegrationError::step_size_underflow).size() > 0);
}

TEST_CASE("IAS15 follows the analytic Kepler orbit", "[core][ias15]") {
    const KeplerElements el = elements(1.3, 0.5, 12.0, 40.0, 70.0, 10.0);
    const double m1 = 1.0;
    const double m2 = 3e-6;
    SimState state = two_body(m1, m2, el);
    const double period = kepler_period(1.3, m1 + m2);
    IAS15 ias;
    const NewtonianGravity gravity;
    double worst = 0.0;
    for (int orbit = 1; orbit <= 10; ++orbit) {
        step_or_throw(ias, state, space::Time(period), gravity);
        const Vec3<space::Length> expected = analytic_relative(el, m1, m2, orbit * period);
        worst = std::max(worst, norm(relative_position(state) - expected).value());
    }
    // The landing time is the sum of the requested intervals.
    CHECK(std::abs(state.time().value() - 10 * period) <= 1e-12 * period);
    report("ias15 e=0.5 a=1.3 AU: max position error over 10 orbits " + sci(worst) +
           " AU, accepted steps " + std::to_string(ias.accepted_steps()) + ", rejected " +
           std::to_string(ias.rejected_steps()) + ", force evaluations " +
           std::to_string(ias.force_evaluations()));
    CHECK(worst < 1e-10);
}

TEST_CASE("IAS15 beats fixed-step Leapfrog at equal cost on an e = 0.99 orbit", "[core][ias15]") {
    // Acceptance: high-eccentricity two-body, energy error compared at the
    // same number of force evaluations.
    const double a = 1.0;
    const double e = 0.99;
    const KeplerElements el = elements(a, e, 0.0, 0.0, 0.0, 180.0); // start at apocentre
    const double period = kepler_period(a, 1.0 + 1e-3);
    const int orbits = 10;
    const int samples_per_orbit = 50;
    const NewtonianGravity gravity;

    SimState ias_state = two_body(1.0, 1e-3, el);
    const double e0 = energy(ias_state, gravity);
    IAS15 ias;
    RunningMax ias_max;
    for (int s = 0; s < orbits * samples_per_orbit; ++s) {
        step_or_throw(ias, ias_state, space::Time(period / samples_per_orbit), gravity);
        ias_max.add(std::abs((energy(ias_state, gravity) - e0) / e0));
    }
    const double ias_final = std::abs((energy(ias_state, gravity) - e0) / e0);
    const long long evaluations = ias.force_evaluations();
    const Vec3<space::Length> expected = analytic_relative(el, 1.0, 1e-3, orbits * period);
    const double ias_pos = norm(relative_position(ias_state) - expected).value();

    // Leapfrog with the same number of force evaluations (2 per step).
    const long long steps = evaluations / 2;
    SimState lf_state = two_body(1.0, 1e-3, el);
    LeapfrogKdk leapfrog;
    const double dt = orbits * period / static_cast<double>(steps);
    RunningMax lf_max;
    const long long per_sample =
        std::max(1LL, steps / (static_cast<long long>(orbits) * samples_per_orbit));
    for (long long s = 0; s < steps; ++s) {
        step_or_throw(leapfrog, lf_state, space::Time(dt), gravity);
        if ((s + 1) % per_sample == 0) {
            lf_max.add(std::abs((energy(lf_state, gravity) - e0) / e0));
        }
    }
    const double lf_final = std::abs((energy(lf_state, gravity) - e0) / e0);
    const double lf_pos = norm(relative_position(lf_state) - expected).value();

    report("e=0.99, 10 orbits, " + std::to_string(evaluations) +
           " force evaluations each: IAS15 |dE/E| max " + sci(ias_max.value) + " final " +
           sci(ias_final) + " position error " + sci(ias_pos) + " AU (" +
           std::to_string(ias.accepted_steps()) + " steps, " +
           std::to_string(ias.rejected_steps()) + " rejected); Leapfrog dt=" + sci(dt) +
           " d |dE/E| max " + sci(lf_max.value) + " final " + sci(lf_final) + " position error " +
           sci(lf_pos) + " AU");
    CHECK(ias_max.below(1e-12));
    CHECK(lf_max.all_finite());
    CHECK(ias_max.value * 1e3 < lf_max.value);
    CHECK(ias_final < lf_final);
    CHECK(ias_pos < 1e-8);
}

TEST_CASE("IAS15 is deterministic and restarts on a foreign state", "[core][ias15][determinism]") {
    const auto run = [] {
        SimState state = golden_initial_state();
        const auto gravity = NewtonianGravity::create(space::Length(1.0e-4));
        IAS15 ias;
        for (int s = 0; s < 200; ++s) {
            step_or_throw(ias, state, space::Time(10.0), gravity.value());
        }
        return state_hash(state);
    };
    const std::uint64_t first = run();
    CHECK(first == run());

    // Restart semantics: a fresh integrator on a copy of a committed state
    // gives the same result as a restarted one.
    SimState a = two_body(1.0, 1e-3, elements(1.0, 0.3, 0, 0, 0, 0));
    const NewtonianGravity gravity;
    IAS15 warm;
    step_or_throw(warm, a, space::Time(30.0), gravity);
    SimState b = a;
    warm.restart();
    IAS15 fresh;
    step_or_throw(warm, a, space::Time(30.0), gravity);
    step_or_throw(fresh, b, space::Time(30.0), gravity);
    CHECK(state_hash(a) == state_hash(b));
}

TEST_CASE("IAS15 golden hash after 10000 steps", "[core][ias15][determinism][acceptance]") {
    // Golden scenario of the direct-force goldens (literal hexadecimal initial
    // state, Plummer softening 1e-4 AU), 10000 output steps of 10 days through
    // IAS15 with the default tolerance. The outputs are longer than the
    // controller's steps (about 1 day, set by Mercury), so the adaptive step
    // control, the b prediction and the warm restart between calls are all
    // part of the pinned value. They use only IEEE +, -, *, /, sqrt, frexp and
    // ldexp, so the value is exact.
    constexpr std::uint64_t golden_ias15 = 0xf9cc3e89244d1214ULL;
    SimState state = golden_initial_state();
    const auto gravity = NewtonianGravity::create(space::Length(1.0e-4));
    REQUIRE(gravity);
    IAS15 ias;
    advance(ias, state, space::Time(10.0), 10000, gravity.value());
    const std::uint64_t hash = state_hash(state);
    report("ias15 golden hash " + hex(hash) + " (" + std::to_string(ias.accepted_steps()) +
           " steps, " + std::to_string(ias.rejected_steps()) + " rejected)");
    CHECK(ias.accepted_steps() > 20000);
    CHECK(hash == golden_ias15);
}

TEST_CASE("IAS15 integrates backwards to the start", "[core][ias15]") {
    SimState state = two_body(1.0, 1e-3, elements(1.0, 0.6, 5.0, 0, 0, 0));
    const SimState start = state;
    const NewtonianGravity gravity;
    IAS15 ias;
    step_or_throw(ias, state, space::Time(400.0), gravity);
    step_or_throw(ias, state, space::Time(-400.0), gravity);
    const double back = norm(relative_position(state) - relative_position(start)).value();
    report("ias15 e=0.6 400 d forward and back: position error " + sci(back) + " AU");
    CHECK(back < 1e-12);
    CHECK(std::abs(state.time().value()) <= 1e-12);
}

TEST_CASE("IAS15 reports a singular force and leaves the state unchanged", "[core][ias15]") {
    BodyStore store;
    add_or_throw(store, BodyInit{"A", Star{}, space::Mass(1.0), {}, {}});
    add_or_throw(store, BodyInit{"B", Star{}, space::Mass(1.0), {}, {}});
    SimState state(space::Time(0.0), store);
    const std::uint64_t before = state_hash(state);
    IAS15 ias;
    const auto result = ias.step(state, space::Time(1.0), NewtonianGravity{});
    REQUIRE_FALSE(result);
    CHECK(result.error() == IntegrationError::non_finite_state);
    CHECK(state_hash(state) == before);
    CHECK(ias.step(state, space::Time(std::nan("")), NewtonianGravity{}).error() ==
          IntegrationError::invalid_time_step);
}

TEST_CASE("IAS15 on the solar system conserves energy to rounding level", "[core][ias15]") {
    SimState state = solar_system();
    const NewtonianGravity gravity;
    const ConservationDiagnostics diag(state, gravity);
    IAS15 ias;
    RunningMax de;
    RunningMax dp;
    // 100 years in 1-year output intervals.
    for (int year = 0; year < 100; ++year) {
        step_or_throw(ias, state, space::Time(365.25), gravity);
        const ConservationDrift d = diag.drift(state, gravity);
        de.add(d.relative_energy);
        dp.add(d.linear_momentum);
    }
    report("ias15 Sun + 8 planets 100 yr: max |dE/E| " + sci(de.value) + ", max momentum drift " +
           sci(dp.value) + ", steps " + std::to_string(ias.accepted_steps()) + ", mean step " +
           sci(100 * 365.25 / static_cast<double>(ias.accepted_steps())) + " d");
    CHECK(de.below(1e-12));
    CHECK(dp.below(1e-12));
}

namespace {

// Records the segments of step_observed and stops at segment `stop_at`
// (counted from 0; negative: never).
struct SegmentLog final : IStepObserver {
    struct Segment {
        std::vector<Vec3<space::Length>> start;
        std::vector<Vec3<space::Length>> end;
        double begin = 0.0;
        double end_offset = 0.0;
    };
    int stop_at = -1;
    std::vector<Segment> segments;

    bool on_segment(const BodyStore& /*bodies*/, std::span<const Vec3<space::Length>> start,
                    std::span<const Vec3<space::Length>> end, space::Time begin,
                    space::Time end_offset) override {
        segments.push_back({{start.begin(), start.end()},
                            {end.begin(), end.end()},
                            begin.value(),
                            end_offset.value()});
        return static_cast<int>(segments.size()) - 1 == stop_at;
    }
};

bool same_positions(const BodyStore& bodies, const std::vector<Vec3<space::Length>>& p) {
    if (p.size() != bodies.size()) {
        return false;
    }
    for (std::size_t i = 0; i < p.size(); ++i) {
        if (!(bodies.position(i) == p[i])) {
            return false;
        }
    }
    return true;
}

} // namespace

TEST_CASE("IAS15 reports its accepted internal steps to an observer", "[core][ias15]") {
    const KeplerElements el = elements(1.0, 0.9, 0.0, 0.0, 0.0, 180.0);
    const NewtonianGravity gravity;
    const space::Time dt(300.0);

    // Without a stop: the same result as step(), bit for bit, and contiguous
    // segments covering [0, dt].
    SimState plain = two_body(1.0, 1e-3, el);
    SimState observed = plain;
    IAS15 ias_plain;
    IAS15 ias_observed;
    step_or_throw(ias_plain, plain, dt, gravity);
    SegmentLog all;
    REQUIRE(ias_observed.step_observed(observed, dt, gravity, all));
    CHECK(state_hash(observed) == state_hash(plain));
    CHECK(observed.time() == plain.time());
    REQUIRE(all.segments.size() == static_cast<std::size_t>(ias_observed.accepted_steps()));
    REQUIRE(all.segments.size() > 10);
    CHECK(all.segments.front().begin == 0.0);
    CHECK(all.segments.back().end_offset == dt.value());
    for (std::size_t k = 1; k < all.segments.size(); ++k) {
        CHECK(all.segments[k].begin == all.segments[k - 1].end_offset);
        CHECK(all.segments[k].start == all.segments[k - 1].end);
    }
    CHECK(same_positions(observed.bodies(), all.segments.back().end));

    // Stop after the fifth segment: committed there, then finish the rest.
    SimState stopped = two_body(1.0, 1e-3, el);
    IAS15 ias_stop;
    SegmentLog five;
    five.stop_at = 4;
    const auto partial = ias_stop.step_observed(stopped, dt, gravity, five);
    REQUIRE(partial);
    REQUIRE(five.segments.size() == 5);
    const double reached = five.segments.back().end_offset;
    CHECK(reached < dt.value());
    CHECK(stopped.time().value() == reached);
    CHECK(same_positions(stopped.bodies(), five.segments.back().end));
    SegmentLog rest;
    REQUIRE(ias_stop.step_observed(stopped, dt - space::Time(reached), gravity, rest));
    CHECK(rest.segments.front().start == five.segments.back().end);
    const double gap = norm(relative_position(stopped) - relative_position(plain)).value();
    report("ias15 observer stop after 5 of " + std::to_string(all.segments.size()) +
           " segments, then resumed: position difference to one call " + sci(gap) + " AU");
    CHECK(gap < 1e-10);

    // A fixed-step integrator reports the whole step as its only segment.
    SimState lf_state = two_body(1.0, 1e-3, el);
    LeapfrogKdk leapfrog;
    SegmentLog one;
    one.stop_at = 0;
    REQUIRE(leapfrog.step_observed(lf_state, space::Time(0.5), gravity, one));
    REQUIRE(one.segments.size() == 1);
    CHECK(one.segments[0].begin == 0.0);
    CHECK(one.segments[0].end_offset == 0.5);
    CHECK(lf_state.time().value() == 0.5);
}
