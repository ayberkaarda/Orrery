#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "collision_test_support.h"
#include "gravity_test_support.h"
#include "orrery/core/black_hole.h"
#include "orrery/core/event.h"
#include "orrery/core/event_step.h"
#include "orrery/core/horizon.h"
#include "orrery/core/ias15.h"
#include "orrery/core/integrator.h"
#include "orrery/core/paczynski_wiita_gravity.h"
#include "orrery/core/state_hash.h"
#include "orrery/core/tidal.h"

using namespace orrery::core;
using namespace orrery::core::test;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

std::vector<Vec3<space::Length>> positions_of(const BodyStore& store) {
    std::vector<Vec3<space::Length>> out(store.size());
    for (std::size_t i = 0; i < store.size(); ++i) {
        out[i] = store.position(i);
    }
    return out;
}

// Moves body i by `shift` to emulate the end of a step whose start positions
// are `start`.
void move(BodyStore& store, std::size_t i, const Vec3<space::Length>& shift) {
    REQUIRE(store.set_position(i, store.position(i) + shift));
}

int phase_of(const Event& e) {
    if (std::holds_alternative<DisruptionEvent>(e)) {
        return 0;
    }
    if (std::holds_alternative<HorizonCrossingEvent>(e)) {
        return 1;
    }
    return 2;
}

double fraction_of(const Event& e) {
    return std::visit([](const auto& v) { return v.step_fraction; }, e);
}

constexpr double cluster_hole_mass = 1.0e6;

// A 1e6 Msun black hole (r_s = 0.0197 AU) and 300 bodies falling inwards from
// 0.1-1.7 AU with some tangential motion: every tenth is a probe, a third of
// the rest are large (R = 0.005-0.02 AU, they collide and start inside their
// tidal radius) and the others small (R = 5e-6 to 1e-5 AU, tidal radius
// 0.05-0.1 AU, so they are disrupted on the way in).
SimState cluster(std::uint64_t seed) {
    const double gm = constants::gravitational_constant_space.value() * cluster_hole_mass;
    SplitMix rng{seed};
    BodyStore store;
    add_or_throw(store, BodyInit{"BH", BlackHole{0.3}, space::Mass(cluster_hole_mass), {}, {}});
    for (int k = 0; k < 300; ++k) {
        Vec3<space::Length> r = at(rng.symmetric(), rng.symmetric(), 0.2 * rng.symmetric());
        double d = norm(r).value();
        if (d < 0.1) {
            r = r * (0.1 / d);
            d = 0.1;
        }
        const double v_ff = std::sqrt(2.0 * gm / d);
        const Vec3<space::Velocity> v =
            vel(-0.5 * v_ff * r.x.value() / d + 0.3 * v_ff * rng.symmetric(),
                -0.5 * v_ff * r.y.value() / d + 0.3 * v_ff * rng.symmetric(), 0.0);
        const bool probe = k % 10 == 0;
        add_or_throw(store, BodyInit{"Body" + std::to_string(k),
                                     probe ? BodyKind{Probe{}} : BodyKind{Asteroid{}},
                                     space::Mass(1.0e-6 * (1.0 + rng.uniform())), r, v});
        const double radius =
            k % 3 == 0 ? 0.005 + 0.015 * rng.uniform() : 5.0e-6 + 5.0e-6 * rng.uniform();
        set_radius_or_throw(store, store.size() - 1, radius);
    }
    return {space::Time(0.0), std::move(store)};
}

struct ClusterRun {
    std::vector<Event> events;
    std::uint64_t hash = 0;
    std::size_t bodies = 0;
    Totals before;
    Totals after;
};

ClusterRun run_cluster(std::uint64_t seed) {
    SimState state = cluster(seed);
    ClusterRun out;
    out.before = totals(state.bodies());
    const PaczynskiWiitaGravity gravity;
    LeapfrogKdk integrator;
    EventStepper stepper;
    EventLog log;
    int previous_phase = 0;
    for (int s = 0; s < 400; ++s) {
        REQUIRE(stepper.step(integrator, state, space::Time(0.002), gravity, log));
        // Within one step: phases in order, step fractions ascending within a
        // phase.
        previous_phase = 0;
        double previous_fraction = 0.0;
        for (const Event& e : log.events()) {
            const int phase = phase_of(e);
            REQUIRE(phase >= previous_phase);
            if (phase != previous_phase) {
                previous_fraction = 0.0;
            }
            REQUIRE(fraction_of(e) >= previous_fraction);
            REQUIRE(fraction_of(e) >= 0.0);
            REQUIRE(fraction_of(e) <= 1.0);
            previous_fraction = fraction_of(e);
            previous_phase = phase;
        }
        for (Event& e : log.drain()) {
            out.events.push_back(e);
        }
        REQUIRE(log.empty());
    }
    out.hash = state_hash(state);
    out.bodies = state.bodies().size();
    out.after = totals(state.bodies());
    return out;
}

} // namespace

TEST_CASE("Horizon crossing absorbs a body conserving mass and momentum", "[core][horizon]") {
    const space::Mass hole(1.0e6);
    const double rs = schwarzschild_radius(hole).value();
    BodyStore store;
    add_or_throw(store, BodyInit{"BH", BlackHole{0.7}, hole, at(0.3, -0.1, 0.02),
                                 vel(0.001, 0.002, -0.0005)});
    add_or_throw(store, BodyInit{"Star", Star{}, space::Mass(0.8), at(0.3 + 3.0 * rs, -0.1, 0.02),
                                 vel(-5.0, 1.5, 0.3)});
    add_or_throw(store, BodyInit{"Far", Planet{}, space::Mass(1.0e-3), at(30.0, 0.0, 0.0),
                                 vel(0.0, 0.01, 0.0)});
    std::vector<Vec3<space::Length>> start = positions_of(store);
    // End of step: the star is 0.5 r_s from the hole.
    move(store, 1, at(-2.5 * rs, 0.0, 0.0));
    const Totals before = totals(store);

    EventLog log;
    CHECK(absorb_horizon_crossings(store, start, space::Time(10.0), space::Time(0.5), log) == 1);
    REQUIRE(start.size() == store.size());
    REQUIRE(store.size() == 2);
    REQUIRE(log.size() == 1);
    const auto& e = std::get<HorizonCrossingEvent>(log.events()[0]);
    CHECK(e.black_hole == BodyId{0});
    CHECK(e.absorbed == BodyId{1});
    CHECK(e.absorbed_mass.value() == 0.8);
    CHECK(e.horizon_radius.value() == rs);
    CHECK_THAT(e.step_fraction, WithinRel(0.8, 1e-12));
    CHECK_THAT(e.time.value(), WithinRel(10.4, 1e-12));
    CHECK(std::holds_alternative<BlackHole>(store.kind(0)));
    CHECK(std::get<BlackHole>(store.kind(0)).spin == 0.7);
    CHECK(store.mass(0).value() == 1.0e6 + 0.8);
    CHECK(e.schwarzschild_radius_after == schwarzschild_radius(store.mass(0)));

    const Totals after = totals(store);
    const double dm = std::abs(after.mass - before.mass) / before.mass;
    const double dp = difference(after.momentum, before.momentum) / before.momentum_scale;
    const double dcm = difference(after.moment, before.moment) / before.moment_scale;
    const double dl_orbital = difference(after.angular, before.angular) / before.angular_scale;
    const double dl =
        difference(plus(after.angular, raw(e.internal_angular_momentum)), before.angular) /
        before.angular_scale;
    report("horizon absorption: relative mass " + sci(dm) + ", momentum " + sci(dp) +
           ", centre of mass " + sci(dcm) + ", angular momentum (orbital + internal) " + sci(dl) +
           " (orbital alone " + sci(dl_orbital) + ")");
    CHECK(dm < 1e-15);
    CHECK(dp < 4e-16);
    CHECK(dcm < 4e-16);
    CHECK(dl < 1e-15);
}

TEST_CASE("Radial plunge conserves orbital angular momentum exactly", "[core][horizon]") {
    const space::Mass hole(1.0e6);
    const double rs = schwarzschild_radius(hole).value();
    BodyStore store;
    add_or_throw(store, BodyInit{"BH", BlackHole{}, hole, at(0.0, 0.0, 0.0), vel(0.0, 0.0, 0.0)});
    add_or_throw(store, BodyInit{"Star", Star{}, space::Mass(1.0), at(2.0 * rs, 0.0, 0.0),
                                 vel(-10.0, 0.0, 0.0)});
    add_or_throw(store, BodyInit{"Moon", Moon{}, space::Mass(1.0e-8), at(0.0, 5.0, 0.0),
                                 vel(0.4, 0.0, 0.0)});
    std::vector<Vec3<space::Length>> start = positions_of(store);
    move(store, 1, at(-1.5 * rs, 0.0, 0.0));
    const Totals before = totals(store);
    EventLog log;
    REQUIRE(absorb_horizon_crossings(store, start, space::Time(0.0), space::Time(1.0), log) == 1);
    const auto& e = std::get<HorizonCrossingEvent>(log.events()[0]);
    CHECK(raw(e.internal_angular_momentum) == Triple{0.0, 0.0, 0.0});
    const Totals after = totals(store);
    const double dl = difference(after.angular, before.angular) / before.angular_scale;
    const double dp = difference(after.momentum, before.momentum) / before.momentum_scale;
    report("radial plunge: relative angular momentum change " + sci(dl) + ", momentum " + sci(dp));
    CHECK(dl < 1e-15);
    CHECK(dp < 4e-16);
}

TEST_CASE("Black hole merger and probe absorption", "[core][horizon]") {
    const space::Mass m1(3.0e6);
    const space::Mass m2(5.0e6);
    const double rs_sum = (schwarzschild_radius(m1) + schwarzschild_radius(m2)).value();
    BodyStore store;
    add_or_throw(store,
                 BodyInit{"Light", BlackHole{0.1}, m1, at(0.0, 0.0, 0.0), vel(1.0, 0.0, 0.0)});
    add_or_throw(store, BodyInit{"Heavy", BlackHole{0.9}, m2, at(0.9 * rs_sum, 0.0, 0.0),
                                 vel(-0.6, 0.0, 0.0)});
    add_or_throw(store, BodyInit{"Probe", Probe{}, space::Mass(1.0e-12), at(10.0, 0.0, 0.0),
                                 vel(0.0, 0.0, 0.0)});
    std::vector<Vec3<space::Length>> start = positions_of(store);
    // The probe falls onto the heavy hole's position within the step.
    move(store, 2, at(0.9 * rs_sum - 10.0, 0.0, 0.0));
    const Totals before = totals(store);
    EventLog log;
    // Merger (step fraction 0, already within r_s1 + r_s2) first, then the
    // probe crosses the merged horizon.
    CHECK(absorb_horizon_crossings(store, start, space::Time(0.0), space::Time(1.0), log) == 2);
    REQUIRE(store.size() == 1);
    REQUIRE(log.size() == 2);
    const auto& merger = std::get<HorizonCrossingEvent>(log.events()[0]);
    CHECK(merger.black_hole == BodyId{1});
    CHECK(merger.absorbed == BodyId{0});
    CHECK(merger.step_fraction == 0.0);
    CHECK(merger.horizon_radius.value() == rs_sum);
    const auto& probe = std::get<HorizonCrossingEvent>(log.events()[1]);
    CHECK(probe.absorbed == BodyId{2});
    CHECK(store.id(0) == BodyId{1});
    CHECK(std::get<BlackHole>(store.kind(0)).spin == 0.9);
    const Totals after = totals(store);
    CHECK(std::abs(after.mass - before.mass) / before.mass < 1e-15);
    CHECK(difference(after.momentum, before.momentum) / before.momentum_scale < 4e-16);
}

TEST_CASE("A merged black hole absorbs a body crossing only its enlarged horizon",
          "[core][horizon]") {
    // Two 1e6 Msun holes 1.5 s apart (s = r_s(1e6 Msun)) merge at once
    // (threshold 2 s). The merged hole sits at 0.75 s with r_s = 2 s, so a
    // probe moving from -3 s to -1.1 s crosses its horizon at x = -1.25 s,
    // step fraction 1.75 / 1.9, although it never reaches either original
    // horizon.
    const space::Mass m(1.0e6);
    const double s = schwarzschild_radius(m).value();
    BodyStore store;
    add_or_throw(store, BodyInit{"Left", BlackHole{}, m, at(0.0, 0.0, 0.0), {}});
    add_or_throw(store, BodyInit{"Right", BlackHole{}, m, at(1.5 * s, 0.0, 0.0), {}});
    add_or_throw(store,
                 BodyInit{"Probe", Probe{}, space::Mass(1.0e-12), at(-3.0 * s, 0.0, 0.0), {}});
    std::vector<Vec3<space::Length>> start = positions_of(store);
    move(store, 2, at(1.9 * s, 0.0, 0.0));
    EventLog log;
    CHECK(absorb_horizon_crossings(store, start, space::Time(5.0), space::Time(2.0), log) == 2);
    REQUIRE(store.size() == 1);
    REQUIRE(start.size() == 1);
    REQUIRE(log.size() == 2);
    const auto& merger = std::get<HorizonCrossingEvent>(log.events()[0]);
    CHECK(merger.step_fraction == 0.0);
    CHECK(merger.black_hole == BodyId{0});
    CHECK(merger.absorbed == BodyId{1});
    const auto& probe = std::get<HorizonCrossingEvent>(log.events()[1]);
    CHECK(probe.black_hole == BodyId{0});
    CHECK(probe.absorbed == BodyId{2});
    CHECK(probe.horizon_radius == schwarzschild_radius(m + m));
    CHECK_THAT(probe.step_fraction, WithinRel(1.75 / 1.9, 1e-12));
    CHECK_THAT(probe.time.value(), WithinRel(5.0 + 2.0 * 1.75 / 1.9, 1e-14));
}

namespace {

// Pericentre plunge (review finding P3-01): a body on an ellipse with
// apocentre 2 AU and pericentre 0.1 AU (a = 1.05 AU, e = 19/21) starts at
// apocentre around a fixed 1 Msun primary. Its path enters r = 0.2 AU near
// pericentre, but both ends of one full period lie at apocentre.
constexpr double plunge_a = 1.05;
constexpr double plunge_e = 19.0 / 21.0;
constexpr double plunge_reach = 0.2;

double plunge_mu() {
    return constants::gravitational_constant_space.value();
}

double plunge_period() {
    return 2.0 * std::numbers::pi * std::sqrt(plunge_a * plunge_a * plunge_a / plunge_mu());
}

// Kepler time from apocentre to the first r = plunge_reach.
double plunge_entry_time() {
    const double cos_e = (1.0 - plunge_reach / plunge_a) / plunge_e;
    const double ecc_anomaly = 2.0 * std::numbers::pi - std::acos(cos_e);
    const double mean_anomaly = ecc_anomaly - plunge_e * std::sin(ecc_anomaly);
    return (mean_anomaly - std::numbers::pi) * plunge_period() / (2.0 * std::numbers::pi);
}

// tidal = false: a 0.2 AU star and a point asteroid (collision).
// tidal = true: a 1 Msun black hole and a planet with r_t = 0.2 AU.
// The full period is requested in `requests` equal output intervals.
std::vector<Event> run_plunge(bool tidal, int requests, std::size_t& bodies_left) {
    BodyStore store;
    if (tidal) {
        add_or_throw(store, BodyInit{"BH", BlackHole{}, space::Mass(1.0), {}, {}});
    } else {
        add_or_throw(store, BodyInit{"Sun", Star{}, space::Mass(1.0), {}, {}});
        set_radius_or_throw(store, 0, plunge_reach);
    }
    const double apocentre = plunge_a * (1.0 + plunge_e);
    const double v_apo = std::sqrt(plunge_mu() * (2.0 / apocentre - 1.0 / plunge_a));
    if (tidal) {
        // r_t = 2e-4 (1 / 1e-9)^(1/3) AU = 0.2 AU.
        add_or_throw(store, BodyInit{"Planet", Planet{}, space::Mass(1.0e-9),
                                     at(apocentre, 0.0, 0.0), vel(0.0, v_apo, 0.0)});
        set_radius_or_throw(store, 1, 2.0e-4);
    } else {
        add_or_throw(store, BodyInit{"Rock", Asteroid{}, space::Mass(0.0), at(apocentre, 0.0, 0.0),
                                     vel(0.0, v_apo, 0.0)});
    }
    SimState state(space::Time(0.0), std::move(store));
    const NewtonianGravity gravity;
    IAS15 integrator;
    EventStepper stepper;
    EventLog log;
    const double interval = plunge_period() / requests;
    for (int k = 0; k < requests; ++k) {
        REQUIRE(stepper.step(integrator, state, space::Time(interval), gravity, log));
    }
    bodies_left = state.bodies().size();
    return log.drain();
}

} // namespace

TEST_CASE("Encounters inside one adaptive output interval are found", "[core][events]") {
    const double expected = plunge_entry_time();
    for (const bool tidal : {false, true}) {
        double first_time = 0.0;
        for (const int requests : {1, 7, 64}) {
            std::size_t bodies_left = 0;
            const std::vector<Event> events = run_plunge(tidal, requests, bodies_left);
            INFO("tidal " << tidal << ", output intervals " << requests);
            REQUIRE(events.size() == 1);
            double time = 0.0;
            if (tidal) {
                const auto* e = std::get_if<DisruptionEvent>(&events[0]);
                REQUIRE(e != nullptr);
                CHECK(e->body == BodyId{1});
                CHECK(bodies_left == 2);
                time = e->time.value();
            } else {
                const auto* e = std::get_if<CollisionEvent>(&events[0]);
                REQUIRE(e != nullptr);
                CHECK(e->survivor == BodyId{0});
                CHECK(e->absorbed == BodyId{1});
                CHECK(bodies_left == 1);
                time = e->time.value();
            }
            report(std::string(tidal ? "tidal entry" : "collision") + " with " +
                   std::to_string(requests) + " output intervals at t = " + sci(time) +
                   " d, Kepler entry " + sci(expected) + " d, difference " + sci(time - expected) +
                   " d");
            // The swept chord of one internal step is short near pericentre:
            // measured differences are about 1e-3 d.
            CHECK(std::abs(time - expected) < 0.005);
            if (requests == 1) {
                first_time = time;
            } else {
                CHECK(std::abs(time - first_time) < 0.005);
            }
        }
    }
}

TEST_CASE("Tidal disruption is detected on entry, not for whole swallowing", "[core][tidal]") {
    // Sun-like star (R = 0.00465 AU) and a 1e6 Msun hole: r_t = 0.465 AU.
    BodyStore store;
    add_or_throw(store, BodyInit{"BH", BlackHole{}, space::Mass(1.0e6), {}, {}});
    add_or_throw(store, BodyInit{"Star", Star{}, space::Mass(1.0), at(1.0, 0.1, 0.0), {}});
    add_or_throw(store, BodyInit{"Inside", Star{}, space::Mass(1.0), at(0.0, 0.2, 0.0), {}});
    add_or_throw(store, BodyInit{"Point", Planet{}, space::Mass(1.0e-3), at(0.9, 0.0, 0.0), {}});
    set_radius_or_throw(store, 1, 0.00465);
    set_radius_or_throw(store, 2, 0.00465);
    const auto start = positions_of(store);
    move(store, 1, at(-0.8, 0.0, 0.0));
    move(store, 2, at(0.0, -0.1, 0.0));
    move(store, 3, at(-0.8, 0.0, 0.0));
    EventLog log;
    detect_tidal_disruptions(store, start, space::Time(0.0), space::Time(1.0), log);
    REQUIRE(log.size() == 1);
    const auto& e = std::get<DisruptionEvent>(log.events()[0]);
    CHECK(e.body == BodyId{1});
    CHECK(e.black_hole == BodyId{0});
    CHECK_THAT(e.tidal_radius.value(), WithinRel(0.465, 1e-14));
    // |(1 - 0.8 tau, 0.1)| = 0.465 at tau = (1 - sqrt(0.465^2 - 0.01)) / 0.8.
    CHECK_THAT(e.step_fraction, WithinRel((1.0 - std::sqrt(0.465 * 0.465 - 0.01)) / 0.8, 1e-13));
    CHECK_THAT(e.closest_approach.value(), WithinRel(std::sqrt(0.04 + 0.01), 1e-14));
    CHECK(store.size() == 4);

    // A 1e9 Msun hole: r_t = 4.65 AU < r_s = 19.7 AU, swallowed whole.
    BodyStore big;
    add_or_throw(big, BodyInit{"BH", BlackHole{}, space::Mass(1.0e9), {}, {}});
    add_or_throw(big, BodyInit{"Star", Star{}, space::Mass(1.0), at(30.0, 0.0, 0.0), {}});
    set_radius_or_throw(big, 1, 0.00465);
    const auto big_start = positions_of(big);
    move(big, 1, at(-29.0, 0.0, 0.0));
    EventLog none;
    detect_tidal_disruptions(big, big_start, space::Time(0.0), space::Time(1.0), none);
    CHECK(none.empty());
}

TEST_CASE("Event log drains in order", "[core][events]") {
    EventLog log;
    log.append(DisruptionEvent{space::Time(1.0), 0.5, BodyId{1}, BodyId{0}, {}, {}});
    log.append(CollisionEvent{});
    CHECK(log.size() == 2);
    const std::vector<Event> drained = log.drain();
    CHECK(log.empty());
    REQUIRE(drained.size() == 2);
    CHECK(std::holds_alternative<DisruptionEvent>(drained[0]));
    CHECK(std::holds_alternative<CollisionEvent>(drained[1]));

    // resolve_encounters rejects misaligned start positions.
    SimState state;
    add_or_throw(state.bodies(), BodyInit{"A", Planet{}, space::Mass(1.0), {}, {}});
    CHECK_THROWS_AS(resolve_encounters(state, {}, space::Time(0.0), space::Time(1.0), {}, log),
                    std::invalid_argument);
}

TEST_CASE("Encounter events are deterministic and conserve the totals",
          "[core][events][determinism]") {
    const ClusterRun a = run_cluster(2024);
    const ClusterRun b = run_cluster(2024);
    std::size_t collisions = 0;
    std::size_t crossings = 0;
    std::size_t disruptions = 0;
    for (const Event& e : a.events) {
        collisions += std::holds_alternative<CollisionEvent>(e) ? 1U : 0U;
        crossings += std::holds_alternative<HorizonCrossingEvent>(e) ? 1U : 0U;
        disruptions += std::holds_alternative<DisruptionEvent>(e) ? 1U : 0U;
    }
    const double dm = std::abs(a.after.mass - a.before.mass) / a.before.mass;
    const double dp = difference(a.after.momentum, a.before.momentum) / a.before.momentum_scale;
    report("cluster (1 BH, 300 bodies, 400 steps): " + std::to_string(collisions) +
           " collisions, " + std::to_string(crossings) + " horizon crossings, " +
           std::to_string(disruptions) + " disruptions, " + std::to_string(a.bodies) +
           " bodies left; relative mass change " + sci(dm) +
           ", momentum change incl. probes (test particles, no reaction) " + sci(dp));
    CHECK(collisions > 0);
    CHECK(crossings > 0);
    CHECK(disruptions > 0);
    CHECK(a.events == b.events);
    CHECK(a.hash == b.hash);
    CHECK(a.bodies == b.bodies);
    CHECK(dm < 1e-14);
}
