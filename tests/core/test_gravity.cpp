#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

#include "gravity_test_support.h"
#include "orrery/core/force_model.h"
#include "orrery/core/units.h"

using namespace orrery::core;
using namespace orrery::core::test;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

const double k2 = constants::gauss_constant_value * constants::gauss_constant_value;

Vec3<space::Length> at(double x, double y, double z) {
    return {space::Length(x), space::Length(y), space::Length(z)};
}

std::vector<Vec3<space::Length>> positions_of(const BodyStore& store) {
    std::vector<Vec3<space::Length>> out(store.size());
    for (std::size_t i = 0; i < store.size(); ++i) {
        out[i] = store.position(i);
    }
    return out;
}

std::vector<Vec3<space::Acceleration>> accelerations_of(const IForceModel& model,
                                                        const BodyStore& store) {
    std::vector<Vec3<space::Acceleration>> out(store.size());
    model.accelerations(store, positions_of(store), out);
    return out;
}

BodyStore three_bodies() {
    BodyStore store;
    add_or_throw(store, BodyInit{"A", Star{}, space::Mass(1.0), at(0.1, -0.2, 0.05), {}});
    add_or_throw(store, BodyInit{"B", Planet{}, space::Mass(1.0e-3), at(5.2, 0.3, -0.1), {}});
    add_or_throw(store, BodyInit{"C", Moon{}, space::Mass(3.0e-6), at(-0.7, 0.8, 0.02), {}});
    return store;
}

} // namespace

TEST_CASE("gauss units: G = k^2 and the acceleration at 1 AU from 1 Msun is k^2",
          "[core][gravity]") {
    CHECK(constants::gravitational_constant_space.value() == k2);
    BodyStore store;
    add_or_throw(store, BodyInit{"Sun", Star{}, space::Mass(1.0), at(0, 0, 0), {}});
    add_or_throw(store, BodyInit{"Test", Probe{}, space::Mass(1.0), at(1.0, 0, 0), {}});
    const auto a = accelerations_of(NewtonianGravity{}, store);
    CHECK(a[1].x.value() == -k2);
    CHECK(a[1].y.value() == 0.0);
    CHECK(a[1].z.value() == 0.0);
    // The probe exerts nothing on the Sun.
    CHECK(a[0] == Vec3<space::Acceleration>{});
}

TEST_CASE("two-body accelerations match the inverse-square law", "[core][gravity]") {
    BodyStore store;
    add_or_throw(store, BodyInit{"A", Star{}, space::Mass(2.0), at(1.0, 2.0, 3.0), {}});
    add_or_throw(store, BodyInit{"B", Planet{}, space::Mass(0.5), at(4.0, 6.0, 3.0), {}});
    const auto a = accelerations_of(NewtonianGravity{}, store);
    // Separation (3, 4, 0), r = 5.
    const double r3 = 125.0;
    CHECK_THAT(a[0].x.value(), WithinRel(k2 * 0.5 * 3.0 / r3, 1e-15));
    CHECK_THAT(a[0].y.value(), WithinRel(k2 * 0.5 * 4.0 / r3, 1e-15));
    CHECK_THAT(a[1].x.value(), WithinRel(-k2 * 2.0 * 3.0 / r3, 1e-15));
    CHECK_THAT(a[1].y.value(), WithinRel(-k2 * 2.0 * 4.0 / r3, 1e-15));
    // Newton's third law: m_A a_A + m_B a_B = 0 to rounding.
    CHECK_THAT(2.0 * a[0].x.value() + 0.5 * a[1].x.value(), WithinAbs(0.0, 1e-20));
    CHECK_THAT(2.0 * a[0].y.value() + 0.5 * a[1].y.value(), WithinAbs(0.0, 1e-20));
}

TEST_CASE("plummer softening bounds the force and reduces to newton for eps = 0",
          "[core][gravity]") {
    REQUIRE_FALSE(NewtonianGravity::create(space::Length(-1.0e-3)));
    REQUIRE_FALSE(
        NewtonianGravity::create(space::Length(std::numeric_limits<double>::quiet_NaN())));
    REQUIRE_FALSE(NewtonianGravity::create(space::Length(std::numeric_limits<double>::infinity())));
    CHECK(NewtonianGravity::create(space::Length(-1.0e-3)).error() ==
          GravityError::invalid_softening);

    const auto exact = NewtonianGravity::create(space::Length(0.0));
    REQUIRE(exact);
    const BodyStore store = three_bodies();
    CHECK(accelerations_of(exact.value(), store) == accelerations_of(NewtonianGravity{}, store));

    const double eps = 0.01;
    const auto soft = NewtonianGravity::create(space::Length(eps));
    REQUIRE(soft);
    CHECK(soft.value().softening().value() == eps);
    BodyStore pair;
    add_or_throw(pair, BodyInit{"A", Star{}, space::Mass(1.0), at(0, 0, 0), {}});
    add_or_throw(pair, BodyInit{"B", Probe{}, space::Mass(1.0), at(0.02, 0, 0), {}});
    const double r = 0.02;
    const auto a = accelerations_of(soft.value(), pair);
    CHECK_THAT(a[1].x.value(), WithinRel(-k2 * r / std::pow(r * r + eps * eps, 1.5), 1e-14));

    // Coincident bodies: finite (zero) force with softening, non-finite without.
    BodyStore coincident;
    add_or_throw(coincident, BodyInit{"A", Star{}, space::Mass(1.0), at(1, 1, 1), {}});
    add_or_throw(coincident, BodyInit{"B", Planet{}, space::Mass(1.0), at(1, 1, 1), {}});
    const auto soft_a = accelerations_of(soft.value(), coincident);
    CHECK(soft_a[0].x.value() == 0.0);
    CHECK(
        std::isfinite(soft.value().potential_energy(coincident, positions_of(coincident)).value()));
    const auto hard_a = accelerations_of(NewtonianGravity{}, coincident);
    CHECK_FALSE(std::isfinite(hard_a[0].x.value()));
}

TEST_CASE("probes feel the field but exert no force", "[core][gravity]") {
    BodyStore with_probe = three_bodies();
    add_or_throw(with_probe,
                 BodyInit{"Probe", Probe{AeroRef{1}}, space::Mass(1.0e3), at(0.3, 0.3, 0.3), {}});
    const BodyStore without_probe = three_bodies();
    const auto a_with = accelerations_of(NewtonianGravity{}, with_probe);
    const auto a_without = accelerations_of(NewtonianGravity{}, without_probe);
    for (std::size_t i = 0; i < without_probe.size(); ++i) {
        CHECK(a_with[i] == a_without[i]);
    }
    CHECK(norm(a_with[3]).value() > 0.0);
    // Two probes do not attract each other.
    BodyStore probes;
    add_or_throw(probes, BodyInit{"P1", Probe{}, space::Mass(1.0), at(0, 0, 0), {}});
    add_or_throw(probes, BodyInit{"P2", Probe{}, space::Mass(1.0), at(1, 0, 0), {}});
    const auto a_probes = accelerations_of(NewtonianGravity{}, probes);
    CHECK(a_probes[0] == Vec3<space::Acceleration>{});
    CHECK(a_probes[1] == Vec3<space::Acceleration>{});
    CHECK(NewtonianGravity{}.potential_energy(probes, positions_of(probes)).value() == 0.0);
    // Probes do not enter the potential energy.
    CHECK(NewtonianGravity{}.potential_energy(with_probe, positions_of(with_probe)) ==
          NewtonianGravity{}.potential_energy(without_probe, positions_of(without_probe)));
}

TEST_CASE("accelerations are minus the gradient of the softened potential", "[core][gravity]") {
    const auto soft = NewtonianGravity::create(space::Length(0.05));
    REQUIRE(soft);
    const NewtonianGravity exact;
    const BodyStore store = three_bodies();
    for (const IForceModel* model : {static_cast<const IForceModel*>(&exact),
                                     static_cast<const IForceModel*>(&soft.value())}) {
        const auto a = accelerations_of(*model, store);
        for (std::size_t i = 0; i < store.size(); ++i) {
            for (int axis = 0; axis < 3; ++axis) {
                // Central difference of U with respect to body i's coordinate.
                const double h = 1.0e-5;
                auto plus = positions_of(store);
                auto minus = positions_of(store);
                space::Length& p = axis == 0 ? plus[i].x : (axis == 1 ? plus[i].y : plus[i].z);
                space::Length& m = axis == 0 ? minus[i].x : (axis == 1 ? minus[i].y : minus[i].z);
                p += space::Length(h);
                m -= space::Length(h);
                const double du = model->potential_energy(store, plus).value() -
                                  model->potential_energy(store, minus).value();
                const double force = -du / (2.0 * h);
                const double expected =
                    store.mass(i).value() *
                    (axis == 0 ? a[i].x : (axis == 1 ? a[i].y : a[i].z)).value();
                CHECK_THAT(force, WithinRel(expected, 1e-7));
            }
        }
    }
}

namespace {

BodyStore pair_of(const BodyKind& kind, double mass, const Vec3<space::Length>& a,
                  const Vec3<space::Length>& b) {
    BodyStore store;
    add_or_throw(store, BodyInit{"A", kind, space::Mass(mass), a, {}});
    add_or_throw(store, BodyInit{"B", kind, space::Mass(mass), b, {}});
    return store;
}

NewtonianGravity softened(double eps) {
    const auto model = NewtonianGravity::create(space::Length(eps));
    REQUIRE(model);
    return model.value();
}

// One step of every integrator leaves a force-free state at rest and advances
// the time.
void check_stationary_step(const BodyStore& store, const IForceModel& model) {
    for (const IntegratorKind kind :
         {IntegratorKind::leapfrog_kdk, IntegratorKind::yoshida4, IntegratorKind::rk4}) {
        SimState state(space::Time(0.0), store);
        const auto integrator = make_integrator(kind);
        const auto stepped = integrator->step(state, space::Time(1.0), model);
        CHECK(stepped);
        if (!stepped) {
            continue;
        }
        CHECK(state.time() == space::Time(1.0));
        for (std::size_t i = 0; i < store.size(); ++i) {
            CHECK(state.bodies().position(i) == store.position(i));
            CHECK(state.bodies().velocity(i) == Vec3<space::Velocity>{});
        }
    }
}

} // namespace

TEST_CASE("force and potential are accurate far outside the ordinary scale range",
          "[core][gravity]") {
    // Separation 1e110 AU without softening: r^3 overflows, the force does not.
    {
        const BodyStore store = pair_of(Star{}, 1.0, at(0, 0, 0), at(1.0e110, 0, 0));
        const auto a = accelerations_of(NewtonianGravity{}, store);
        const double expected = k2 / (1.0e110 * 1.0e110);
        CHECK_THAT(a[0].x.value(), WithinRel(expected, 1e-14));
        CHECK_THAT(a[1].x.value(), WithinRel(-expected, 1e-14));
        CHECK(a[0].y.value() == 0.0);
        CHECK(a[0].z.value() == 0.0);
        CHECK(a[1].y.value() == 0.0);
        CHECK(a[1].z.value() == 0.0);
        const double u = NewtonianGravity{}.potential_energy(store, positions_of(store)).value();
        CHECK_THAT(u, WithinRel(-k2 / 1.0e110, 1e-14));
        // The force is still minus the gradient of U at this scale.
        const double h = 1.0e104;
        auto plus = positions_of(store);
        auto minus = positions_of(store);
        plus[1].x += space::Length(h);
        minus[1].x -= space::Length(h);
        const double du = NewtonianGravity{}.potential_energy(store, plus).value() -
                          NewtonianGravity{}.potential_energy(store, minus).value();
        CHECK_THAT(-du / (2.0 * h), WithinRel(a[1].x.value(), 1e-7));
    }
    // Coordinates whose squared separation overflows: U = -G / r is representable.
    {
        const BodyStore store = pair_of(Star{}, 1.0, at(0, 0, 0), at(0, 1.0e200, 0));
        const double u = NewtonianGravity{}.potential_energy(store, positions_of(store)).value();
        CHECK_THAT(u, WithinRel(-k2 / 1.0e200, 1e-14));
        for (const auto& a : accelerations_of(NewtonianGravity{}, store)) {
            CHECK(isfinite(a));
        }
    }
    // Coincident sources with tiny softening: 1 / eps^3 overflows, the exact
    // softened force is zero and U = -G / eps.
    for (const double eps : {1.0e-110, 1.0e-200}) {
        const NewtonianGravity model = softened(eps);
        const BodyStore store = pair_of(Star{}, 1.0, at(0, 0, 0), at(0, 0, 0));
        for (const auto& a : accelerations_of(model, store)) {
            CHECK(a == Vec3<space::Acceleration>{});
        }
        const double u = model.potential_energy(store, positions_of(store)).value();
        CHECK_THAT(u, WithinRel(-k2 / eps, 1e-14));
        check_stationary_step(store, model);
    }
    // Very large softening: U is about -G / eps, not zero.
    {
        const NewtonianGravity model = softened(1.0e200);
        const BodyStore store = pair_of(Star{}, 1.0, at(0, 0, 0), at(1.0, 0, 0));
        const double u = model.potential_energy(store, positions_of(store)).value();
        CHECK_THAT(u, WithinRel(-k2 / 1.0e200, 1e-14));
        for (const auto& a : accelerations_of(model, store)) {
            CHECK(isfinite(a));
        }
    }
    // A small separation inside a small softening length (the force is
    // linear in r there): a = G m r / eps^3 with r = 1e-300, eps = 1e-150.
    {
        const NewtonianGravity model = softened(1.0e-150);
        const BodyStore store = pair_of(Star{}, 1.0, at(0, 0, 0), at(1.0e-300, 0, 0));
        const auto a = accelerations_of(model, store);
        // 1e-300 / 1e-150 / 1e-150 / 1e-150 = 1e150 without forming eps^3.
        CHECK_THAT(a[0].x.value(),
                   WithinRel(k2 * (1.0e-300 / 1.0e-150 / 1.0e-150 / 1.0e-150), 1e-14));
        CHECK_THAT(a[1].x.value(), WithinRel(-a[0].x.value(), 1e-15));
    }
}

TEST_CASE("zero-mass bodies are not sources but are accelerated", "[core][gravity]") {
    const NewtonianGravity gravity;
    // Coincident and separated zero-mass asteroids: no field, U = 0, at rest.
    for (const double x : {0.0, 1.0}) {
        const BodyStore store = pair_of(Asteroid{}, 0.0, at(0, 0, 0), at(x, 0, 0));
        for (const auto& a : accelerations_of(gravity, store)) {
            CHECK(a == Vec3<space::Acceleration>{});
        }
        CHECK(gravity.potential_energy(store, positions_of(store)).value() == 0.0);
        check_stationary_step(store, gravity);
    }
    // A zero-mass receiver feels a positive-mass source, in either order.
    for (const bool source_first : {true, false}) {
        BodyStore store;
        const BodyInit sun{"Sun", Star{}, space::Mass(1.0), at(0, 0, 0), {}};
        const BodyInit dust{"Dust", Asteroid{}, space::Mass(0.0), at(1.0, 0, 0), {}};
        add_or_throw(store, source_first ? sun : dust);
        add_or_throw(store, source_first ? dust : sun);
        const std::size_t s = source_first ? 0 : 1;
        const std::size_t d = 1 - s;
        const auto a = accelerations_of(gravity, store);
        CHECK(a[d].x.value() == -k2);
        CHECK(a[d].y.value() == 0.0);
        CHECK(a[d].z.value() == 0.0);
        CHECK(a[s] == Vec3<space::Acceleration>{});
        CHECK(gravity.potential_energy(store, positions_of(store)).value() == 0.0);
    }
    // A zero-mass receiver on top of a point source is genuinely singular.
    BodyStore singular;
    add_or_throw(singular, BodyInit{"Sun", Star{}, space::Mass(1.0), at(0, 0, 0), {}});
    add_or_throw(singular, BodyInit{"Dust", Asteroid{}, space::Mass(0.0), at(0, 0, 0), {}});
    const auto a = accelerations_of(gravity, singular);
    CHECK_FALSE(isfinite(a[1]));
    CHECK(a[0] == Vec3<space::Acceleration>{});
}

TEST_CASE("force model rejects mismatched spans and leaves the output unchanged",
          "[core][gravity]") {
    const BodyStore store = three_bodies();
    const NewtonianGravity model;
    std::vector<Vec3<space::Length>> short_positions(2);
    std::vector<Vec3<space::Acceleration>> out(
        3, Vec3<space::Acceleration>{space::Acceleration(7.0), {}, {}});
    CHECK_THROWS_AS(model.accelerations(store, short_positions, out), std::invalid_argument);
    CHECK(out[0].x.value() == 7.0);
    std::vector<Vec3<space::Acceleration>> short_out(2);
    CHECK_THROWS_AS(model.accelerations(store, positions_of(store), short_out),
                    std::invalid_argument);
    CHECK_THROWS_AS(static_cast<void>(model.potential_energy(store, short_positions)),
                    std::invalid_argument);
    CHECK(to_string(GravityError::invalid_softening).size() > 0);
    CHECK(model.name() == "newtonian");
}

TEST_CASE("force summation is deterministic and independent of earlier calls", "[core][gravity]") {
    const BodyStore store = three_bodies();
    const auto first = accelerations_of(NewtonianGravity{}, store);
    // Stale content in the output buffer must not leak into the result.
    std::vector<Vec3<space::Acceleration>> reused(
        store.size(),
        Vec3<space::Acceleration>{space::Acceleration(1.0e9), space::Acceleration(-3.0),
                                  space::Acceleration(42.0)});
    NewtonianGravity{}.accelerations(store, positions_of(store), reused);
    CHECK(reused == first);
}