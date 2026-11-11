#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "collision_test_support.h"
#include "gravity_test_support.h"
#include "orrery/core/collision.h"
#include "orrery/core/event_step.h"
#include "orrery/core/force_model.h"
#include "orrery/core/integrator.h"

using namespace orrery::core;
using namespace orrery::core::test;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

TEST_CASE("Swept sphere contact time", "[core][collision]") {
    // Head-on, passing through each other within the step (no tunnelling).
    const auto tau =
        swept_contact_fraction(at(10.0, 0.0, 0.0), at(-10.0, 0.0, 0.0), space::Length(2.0));
    REQUIRE(tau);
    CHECK_THAT(*tau, WithinAbs(0.4, 1e-15));
    // Overlapping at the start.
    CHECK(swept_contact_fraction(at(1.0, 0.0, 0.0), at(5.0, 0.0, 0.0), space::Length(2.0)) == 0.0);
    // Miss: impact parameter 3 > 2.
    CHECK_FALSE(
        swept_contact_fraction(at(10.0, 3.0, 0.0), at(-10.0, 3.0, 0.0), space::Length(2.0)));
    // Approaching but contact after the step.
    CHECK_FALSE(swept_contact_fraction(at(10.0, 0.0, 0.0), at(5.0, 0.0, 0.0), space::Length(2.0)));
    // Receding.
    CHECK_FALSE(swept_contact_fraction(at(3.0, 0.0, 0.0), at(10.0, 0.0, 0.0), space::Length(2.0)));
    // Oblique: p(tau) = (10 - 20 tau, 1, 0), |p| = 2 at 10 - 20 tau = sqrt(3).
    const auto oblique =
        swept_contact_fraction(at(10.0, 1.0, 0.0), at(-10.0, 1.0, 0.0), space::Length(2.0));
    REQUIRE(oblique);
    CHECK_THAT(*oblique, WithinRel((10.0 - std::sqrt(3.0)) / 20.0, 1e-15));
    // Scale invariance far outside the ordinary double range of squares.
    for (const double s : {0x1p-900, 0x1p+900}) {
        const auto scaled = swept_contact_fraction(
            at(10.0 * s, 1.0 * s, 0.0), at(-10.0 * s, 1.0 * s, 0.0), space::Length(2.0 * s));
        REQUIRE(scaled);
        CHECK(*scaled == *oblique);
    }
    const ClosestApproach c = swept_closest_approach(at(10.0, 1.0, 0.0), at(-10.0, 1.0, 0.0));
    CHECK_THAT(c.step_fraction, WithinAbs(0.5, 1e-15));
    CHECK_THAT(c.distance.value(), WithinAbs(1.0, 1e-15));
}

TEST_CASE("Swept contact survives cancellation in the discriminant", "[core][collision]") {
    // Long nearly straight passes: (a.d)^2 and |d|^2 (|a|^2 - R^2) agree to
    // ~17 digits, so subtracting them loses the perpendicular distance.
    // Every case is repeated under exact power-of-two scalings.
    for (const double s : {1.0, 0x1p-40, 0x1p+40, 0x1p-900, 0x1p+900}) {
        const space::Length reach(s);
        // Impact parameters 2 and 1.00000001 miss a reach of 1.
        CHECK_FALSE(swept_contact_fraction(at(1.0e9 * s, 2.0 * s, 0.0),
                                           at(-1.0e9 * s, 2.0 * s, 0.0), reach));
        CHECK_FALSE(swept_contact_fraction(at(1.0e8 * s, 1.00000001 * s, 0.0),
                                           at(-1.0e8 * s, 1.00000001 * s, 0.0), reach));
        // Impact parameter 0.5 hits where x = sqrt(1 - 0.25).
        const auto hit = swept_contact_fraction(at(1.0e9 * s, 0.5 * s, 0.0),
                                                at(-1.0e9 * s, 0.5 * s, 0.0), reach);
        REQUIRE(hit);
        CHECK_THAT(*hit, WithinRel((1.0e9 - std::sqrt(0.75)) / 2.0e9, 1e-15));
        // Grazing hit, impact parameter 0.99999999: x = sqrt(1 - b^2).
        const double b = 0.99999999;
        const auto graze =
            swept_contact_fraction(at(1.0e8 * s, b * s, 0.0), at(-1.0e8 * s, b * s, 0.0), reach);
        REQUIRE(graze);
        CHECK_THAT(*graze, WithinRel((1.0e8 - std::sqrt((1.0 - b) * (1.0 + b))) / 2.0e8, 1e-15));
        // Short oblique path: p(tau) = (1 - 2 tau, 0.5, 0) reaches 1 at
        // 1 - 2 tau = sqrt(0.75).
        const auto near = swept_contact_fraction(at(1.0 * s, 0.5 * s, 0.0),
                                                 at(-1.0 * s, 0.5 * s, 0.0), space::Length(s));
        REQUIRE(near);
        CHECK_THAT(*near, WithinRel((1.0 - std::sqrt(0.75)) / 2.0, 1e-15));
        const ClosestApproach far =
            swept_closest_approach(at(1.0e9 * s, 2.0 * s, 0.0), at(-1.0e9 * s, 2.0 * s, 0.0));
        CHECK_THAT(far.distance.value(), WithinRel(2.0 * s, 1e-15));
    }
}

TEST_CASE("Inelastic merge conserves mass, momentum and centre of mass", "[core][collision]") {
    BodyStore store;
    add_or_throw(store, BodyInit{"Moon", Moon{}, space::Mass(3.7e-8), at(1.3, -0.2, 0.07),
                                 vel(0.0123, 0.0071, -0.0004)});
    add_or_throw(store, BodyInit{"Planet", Planet{}, space::Mass(3.0e-6),
                                 at(1.30001, -0.20002, 0.07), vel(-0.0031, 0.0168, 0.0002)});
    add_or_throw(store, BodyInit{"Other", Asteroid{}, space::Mass(1.0e-12), at(5.0, 0.0, 0.0), {}});
    set_radius_or_throw(store, 0, 1.16e-5);
    set_radius_or_throw(store, 1, 4.26e-5);
    const Totals before = totals(store);
    const double r_moon = store.radius(0).value();
    const double r_planet = store.radius(1).value();

    const std::size_t survivor = heavier_of(store, 0, 1);
    REQUIRE(survivor == 1);
    const auto merged = merge_bodies(store, 1, 0, RadiusRule::combine_volumes);
    REQUIRE(merged);
    REQUIRE(store.size() == 2);
    const Totals after = totals(store);

    // Heavier wins kind, id and name.
    CHECK(store.id(0) == BodyId{1});
    CHECK(std::holds_alternative<Planet>(store.kind(0)));
    CHECK(store.name(0) == "Planet");
    CHECK(merged.value().survivor == BodyId{1});
    CHECK(merged.value().absorbed == BodyId{0});
    CHECK_THAT(
        store.radius(0).value(),
        WithinRel(std::cbrt(r_moon * r_moon * r_moon + r_planet * r_planet * r_planet), 1e-15));

    const double dm = std::abs(after.mass - before.mass) / before.mass;
    const double dp = difference(after.momentum, before.momentum) / before.momentum_scale;
    const double dcm = difference(after.moment, before.moment) / before.moment_scale;
    const double dl = difference(plus(after.angular, raw(merged.value().internal_angular_momentum)),
                                 before.angular) /
                      before.angular_scale;
    report("merge: relative mass " + sci(dm) + ", momentum " + sci(dp) + ", centre of mass " +
           sci(dcm) + ", angular momentum (orbital + internal) " + sci(dl));
    CHECK(dm < 1e-15);
    CHECK(dp < 4e-16);
    CHECK(dcm < 4e-16);
    CHECK(dl < 1e-15);

    // Dissipated energy = kinetic energy lost.
    CHECK(merged.value().dissipated_energy.value() > 0.0);

    // Equal masses: the lower id survives.
    BodyStore twins;
    add_or_throw(twins, BodyInit{"A", Asteroid{}, space::Mass(1.0e-10), at(0.0, 0.0, 0.0), {}});
    add_or_throw(twins, BodyInit{"B", Moon{}, space::Mass(1.0e-10), at(1.0, 0.0, 0.0), {}});
    CHECK(heavier_of(twins, 1, 0) == 0);
}

TEST_CASE("Broad phase matches exhaustive bounding-box test", "[core][collision]") {
    SplitMix rng{12345};
    std::vector<SweptSphere> spheres(600);
    for (SweptSphere& s : spheres) {
        s.start = at(rng.symmetric(), rng.symmetric(), rng.symmetric());
        s.end =
            s.start + at(0.05 * rng.symmetric(), 0.05 * rng.symmetric(), 0.05 * rng.symmetric());
        s.radius = space::Length(rng.uniform() < 0.2 ? 0.0 : 0.05 * rng.uniform());
        s.active = rng.uniform() < 0.95;
    }
    const auto pairs = broad_phase_pairs(spheres);
    std::vector<std::pair<std::size_t, std::size_t>> brute;
    const auto box_overlap = [](const SweptSphere& a, const SweptSphere& b) {
        const auto lo = [](const SweptSphere& s, int k) {
            const double p0 = k == 0   ? s.start.x.value()
                              : k == 1 ? s.start.y.value()
                                       : s.start.z.value();
            const double p1 = k == 0 ? s.end.x.value() : k == 1 ? s.end.y.value() : s.end.z.value();
            return std::pair{std::min(p0, p1) - s.radius.value(),
                             std::max(p0, p1) + s.radius.value()};
        };
        for (int k = 0; k < 3; ++k) {
            const auto [alo, ahi] = lo(a, k);
            const auto [blo, bhi] = lo(b, k);
            if (ahi < blo || bhi < alo) {
                return false;
            }
        }
        return true;
    };
    for (std::size_t i = 0; i < spheres.size(); ++i) {
        for (std::size_t j = i + 1; j < spheres.size(); ++j) {
            if (spheres[i].active && spheres[j].active && box_overlap(spheres[i], spheres[j])) {
                brute.emplace_back(i, j);
            }
        }
    }
    report("broad phase: " + std::to_string(pairs.size()) + " pairs of " +
           std::to_string(spheres.size()) +
           " swept spheres (exhaustive: " + std::to_string(brute.size()) + ")");
    CHECK(pairs == brute);
    CHECK(broad_phase_pairs(spheres) == pairs);
}

namespace {

bool boxes_overlap(const SweptSphere& a, const SweptSphere& b) {
    const auto range = [](const SweptSphere& s, int k) {
        const double p0 = k == 0   ? s.start.x.value()
                          : k == 1 ? s.start.y.value()
                                   : s.start.z.value();
        const double p1 = k == 0 ? s.end.x.value() : k == 1 ? s.end.y.value() : s.end.z.value();
        return std::pair{std::min(p0, p1) - s.radius.value(), std::max(p0, p1) + s.radius.value()};
    };
    for (int k = 0; k < 3; ++k) {
        const auto [alo, ahi] = range(a, k);
        const auto [blo, bhi] = range(b, k);
        if (ahi < blo || bhi < alo) {
            return false;
        }
    }
    return true;
}

std::vector<std::pair<std::size_t, std::size_t>>
exhaustive_pairs(const std::vector<SweptSphere>& spheres) {
    std::vector<std::pair<std::size_t, std::size_t>> out;
    for (std::size_t i = 0; i < spheres.size(); ++i) {
        for (std::size_t j = i + 1; j < spheres.size(); ++j) {
            if (spheres[i].active && spheres[j].active && boxes_overlap(spheres[i], spheres[j])) {
                out.emplace_back(i, j);
            }
        }
    }
    return out;
}

// k^3 stationary points of radius 1e-9 AU on a regular grid in [0, 1]^3
// plus one distant body sweeping 1e6 AU.
std::vector<SweptSphere> grid_with_outlier(int k) {
    std::vector<SweptSphere> spheres;
    for (int ix = 0; ix < k; ++ix) {
        for (int iy = 0; iy < k; ++iy) {
            for (int iz = 0; iz < k; ++iz) {
                const auto p = at(static_cast<double>(ix) / k, static_cast<double>(iy) / k,
                                  static_cast<double>(iz) / k);
                spheres.push_back({p, p, space::Length(1.0e-9), true});
            }
        }
    }
    spheres.push_back({at(0.0, 1.0e6, 0.0), at(1.0e6, 1.0e6, 0.0), space::Length(1.0e-9), true});
    return spheres;
}

} // namespace

TEST_CASE("Broad phase matches brute force on mixed scales", "[core][collision]") {
    SplitMix rng{777};
    std::vector<SweptSphere> spheres;
    for (int i = 0; i < 500; ++i) {
        SweptSphere s;
        // Clustered small bodies, a few big sweepers and far outliers.
        const double scale = i % 50 == 0 ? 1.0e3 : (i % 7 == 0 ? 1.0 : 1.0e-3);
        s.start = at(rng.symmetric(), rng.symmetric(), rng.symmetric());
        s.end =
            s.start + at(scale * rng.symmetric(), scale * rng.symmetric(), scale * rng.symmetric());
        s.radius = space::Length(i % 11 == 0 ? 0.0 : 0.02 * rng.uniform() * std::min(scale, 1.0));
        s.active = rng.uniform() < 0.97;
        spheres.push_back(s);
    }
    // Coincident points and touching boxes.
    spheres.push_back({at(0.25, 0.25, 0.25), at(0.25, 0.25, 0.25), space::Length(0.0), true});
    spheres.push_back({at(0.25, 0.25, 0.25), at(0.25, 0.25, 0.25), space::Length(0.0), true});
    spheres.push_back({at(0.5, 0.0, 0.0), at(0.5, 0.0, 0.0), space::Length(0.25), true});
    spheres.push_back({at(1.0, 0.0, 0.0), at(1.0, 0.0, 0.0), space::Length(0.25), true});
    CHECK(broad_phase_pairs(spheres) == exhaustive_pairs(spheres));

    const auto small = grid_with_outlier(5);
    CHECK(broad_phase_pairs(small) == exhaustive_pairs(small));
}

TEST_CASE("Broad phase candidate count scales near-linearly with a distant outlier",
          "[core][collision]") {
    std::size_t previous_tests = 0;
    std::size_t previous_n = 0;
    for (const int k : {8, 16, 32}) {
        const auto spheres = grid_with_outlier(k);
        BroadPhaseStats stats;
        const auto pairs = broad_phase_pairs(spheres, &stats);
        CHECK(pairs.empty());
        const std::size_t n = spheres.size();
        report("broad phase, " + std::to_string(n) +
               " bodies with a 1e6 AU outlier: " + std::to_string(stats.box_tests) + " box tests");
        // Bounded work per body, independent of N.
        CHECK(stats.box_tests <= 64 * n);
        if (previous_n > 0) {
            // N grows 8x: linear 8x, log-linear about 10x, quadratic 64x.
            CHECK(stats.box_tests <= 12 * previous_tests);
        }
        previous_tests = stats.box_tests;
        previous_n = n;
    }
}

TEST_CASE("Collision in a step: CCD catches a pass-through and conserves momentum",
          "[core][collision]") {
    // Two asteroids that would pass through each other within one step.
    BodyStore store;
    add_or_throw(store, BodyInit{"A", Asteroid{}, space::Mass(2.0e-12), at(-0.01, 0.0, 0.0),
                                 vel(0.5, 0.0001, 0.0)});
    add_or_throw(store, BodyInit{"B", Asteroid{}, space::Mass(1.0e-12), at(0.01, 0.00005, 0.0),
                                 vel(-0.5, 0.0, 0.0)});
    add_or_throw(store, BodyInit{"Probe", Probe{}, space::Mass(1.0e-15), at(0.0, 0.0, 0.0),
                                 vel(0.0, 0.0, 0.0)});
    set_radius_or_throw(store, 0, 1.0e-4);
    set_radius_or_throw(store, 1, 1.0e-4);
    set_radius_or_throw(store, 2, 1.0e-3);
    SimState state(space::Time(0.0), std::move(store));
    const Totals before = totals(state.bodies());

    const NewtonianGravity gravity;
    LeapfrogKdk integrator;
    EventStepper stepper;
    EventLog log;
    const space::Time dt(0.04);
    REQUIRE(stepper.step(integrator, state, dt, gravity, log));
    // After the step the asteroids are on opposite sides again: only CCD sees it.
    REQUIRE(log.size() == 1);
    const auto* hit = std::get_if<CollisionEvent>(&log.events()[0]);
    REQUIRE(hit != nullptr);
    CHECK(hit->survivor == BodyId{0});
    CHECK(hit->absorbed == BodyId{1});
    CHECK(hit->step_fraction > 0.45);
    CHECK(hit->step_fraction < 0.5);
    CHECK_THAT(hit->time.value(), WithinRel(hit->step_fraction * 0.04, 1e-15));
    // The probe overlapped both asteroids but takes no part in collisions.
    REQUIRE(state.bodies().size() == 2);
    CHECK(std::holds_alternative<Probe>(state.bodies().kind(1)));

    const Totals after = totals(state.bodies());
    const double dp = difference(after.momentum, before.momentum) / before.momentum_scale;
    const double dm = std::abs(after.mass - before.mass) / before.mass;
    report("CCD collision at step fraction " + sci(hit->step_fraction) +
           ": relative momentum change over step + merge " + sci(dp) + ", mass " + sci(dm));
    CHECK(dp < 1e-15);
    CHECK(dm < 1e-15);
}

namespace {

// No forces: every integrator moves the bodies on exact straight lines.
class NoForces final : public IForceModel {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "none"; }
    void accelerations(const BodyStore& /*bodies*/,
                       std::span<const Vec3<space::Length>> /*positions*/,
                       std::span<Vec3<space::Acceleration>> out) const override {
        for (Vec3<space::Acceleration>& a : out) {
            a = {};
        }
    }
    [[nodiscard]] space::Energy
    potential_energy(const BodyStore& /*bodies*/,
                     std::span<const Vec3<space::Length>> /*positions*/) const override {
        return {};
    }
};

} // namespace

TEST_CASE("A merged survivor keeps colliding within the same step", "[core][collision]") {
    // A (m 2, x 0) at rest, B (m 1, x -2, v 22) and C (m 1, x 3) at rest, all
    // R = 0.1, one step of 1 day: B hits A at tau = 1.8 / 22; the merged A
    // (R = cbrt(2) 0.1, v = 22/3, path from -2/3 to 20/3) then hits C.
    BodyStore store;
    add_or_throw(store, BodyInit{"A", Asteroid{}, space::Mass(2.0), at(0.0, 0.0, 0.0), {}});
    add_or_throw(store, BodyInit{"B", Asteroid{}, space::Mass(1.0), at(-2.0, 0.0, 0.0),
                                 vel(22.0, 0.0, 0.0)});
    add_or_throw(store, BodyInit{"C", Asteroid{}, space::Mass(1.0), at(3.0, 0.0, 0.0), {}});
    for (std::size_t i = 0; i < 3; ++i) {
        set_radius_or_throw(store, i, 0.1);
    }
    SimState state(space::Time(0.0), std::move(store));
    const Totals before = totals(state.bodies());
    const NoForces none;
    LeapfrogKdk integrator;
    EventStepper stepper;
    EventLog log;
    REQUIRE(stepper.step(integrator, state, space::Time(1.0), none, log));

    REQUIRE(log.size() == 2);
    const auto& first = std::get<CollisionEvent>(log.events()[0]);
    const auto& second = std::get<CollisionEvent>(log.events()[1]);
    CHECK(first.survivor == BodyId{0});
    CHECK(first.absorbed == BodyId{1});
    CHECK_THAT(first.step_fraction, WithinRel(1.8 / 22.0, 1e-14));
    CHECK(second.survivor == BodyId{0});
    CHECK(second.absorbed == BodyId{2});
    const double reach = std::cbrt(2.0) * 0.1 + 0.1;
    CHECK_THAT(second.step_fraction, WithinRel((3.0 - reach + 2.0 / 3.0) / (22.0 / 3.0), 1e-14));
    CHECK(second.step_fraction >= first.step_fraction);
    REQUIRE(state.bodies().size() == 1);
    CHECK(state.bodies().mass(0).value() == 4.0);
    CHECK_THAT(state.bodies().position(0).x.value(), WithinRel(23.0 / 4.0, 1e-15));
    CHECK_THAT(state.bodies().velocity(0).x.value(), WithinRel(22.0 / 4.0, 1e-15));
    const Totals after = totals(state.bodies());
    CHECK(std::abs(after.mass - before.mass) / before.mass < 1e-15);
    CHECK(difference(after.momentum, before.momentum) / before.momentum_scale < 1e-15);

    // Three bodies overlapping at the start all merge in one step.
    BodyStore pile;
    add_or_throw(pile, BodyInit{"P", Asteroid{}, space::Mass(1.0), at(0.0, 0.0, 0.0), {}});
    add_or_throw(pile, BodyInit{"Q", Asteroid{}, space::Mass(2.0), at(0.1, 0.0, 0.0), {}});
    add_or_throw(pile, BodyInit{"S", Asteroid{}, space::Mass(3.0), at(0.0, 0.15, 0.0), {}});
    for (std::size_t i = 0; i < 3; ++i) {
        set_radius_or_throw(pile, i, 0.1);
    }
    SimState piled(space::Time(0.0), std::move(pile));
    EventLog pile_log;
    REQUIRE(stepper.step(integrator, piled, space::Time(1.0), none, pile_log));
    REQUIRE(pile_log.size() == 2);
    const auto& pq = std::get<CollisionEvent>(pile_log.events()[0]);
    const auto& rest = std::get<CollisionEvent>(pile_log.events()[1]);
    CHECK(pq.step_fraction == 0.0);
    CHECK(pq.survivor == BodyId{1});
    CHECK(pq.absorbed == BodyId{0});
    CHECK(rest.step_fraction == 0.0);
    CHECK(rest.survivor == BodyId{1});
    CHECK(rest.absorbed == BodyId{2});
    REQUIRE(piled.bodies().size() == 1);
    CHECK(piled.bodies().mass(0).value() == 6.0);
}

TEST_CASE("Points never collide; black holes are left to the horizon phase", "[core][collision]") {
    BodyStore store;
    add_or_throw(store, BodyInit{"A", Asteroid{}, space::Mass(1.0e-12), at(0.0, 0.0, 0.0), {}});
    add_or_throw(store, BodyInit{"B", Asteroid{}, space::Mass(1.0e-12), at(0.0, 0.0, 0.0), {}});
    add_or_throw(store, BodyInit{"BH", BlackHole{}, space::Mass(1.0), at(1.0, 0.0, 0.0), {}});
    add_or_throw(store, BodyInit{"C", Asteroid{}, space::Mass(1.0e-12), at(1.0, 0.0, 0.0), {}});
    set_radius_or_throw(store, 2, 1.0);
    std::vector<Vec3<space::Length>> start;
    start.reserve(store.size());
    for (std::size_t i = 0; i < store.size(); ++i) {
        start.push_back(store.position(i));
    }
    EventLog log;
    CHECK(resolve_collisions(store, start, space::Time(0.0), space::Time(1.0), log) == 0);
    CHECK(log.empty());
    CHECK(store.size() == 4);

    CHECK_FALSE(store.set_radius(0, space::Length(-1.0)));
    CHECK_FALSE(store.set_radius(0, space::Length(std::nan(""))));
    CHECK(store.radius(0).value() == 0.0);
}
