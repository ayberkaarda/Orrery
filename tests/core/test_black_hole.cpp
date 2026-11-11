#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <variant>
#include <vector>

#include "gravity_test_support.h"
#include "orrery/core/black_hole.h"
#include "orrery/core/event_step.h"
#include "orrery/core/force_model.h"
#include "orrery/core/integrator.h"
#include "orrery/core/paczynski_wiita_gravity.h"
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

Vec3<space::Velocity> vel(double x, double y, double z) {
    return {space::Velocity(x), space::Velocity(y), space::Velocity(z)};
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

// Sgr A*-like mass: r_s = 0.0790 AU.
constexpr double sgr_mass = 4.0e6;

// Black hole at rest at the origin plus a probe (test particle) at radius r on
// the x axis with velocity (v_r, v_t, 0).
SimState hole_and_probe(double hole_mass, double r, double v_r, double v_t) {
    BodyStore store;
    add_or_throw(store, BodyInit{"BH", BlackHole{0.5}, space::Mass(hole_mass), {}, {}});
    add_or_throw(store, BodyInit{"Probe", Probe{}, space::Mass(1.0e-12), at(r, 0.0, 0.0),
                                 vel(v_r, v_t, 0.0)});
    return {space::Time(0.0), std::move(store)};
}

struct OrbitOutcome {
    double r_min = 0.0;
    double r_max = 0.0;
    bool absorbed = false;
    double absorbed_after_orbits = 0.0;
};

// Integrates a probe around a black hole with the PW force, Yoshida4 and the
// encounter phase, for `orbits` circular periods of radius r0.
OrbitOutcome run_orbit(double r0_over_rs, double tangential_factor, double radial_fraction,
                       double orbits) {
    const space::Mass m(sgr_mass);
    const double rs = schwarzschild_radius(m).value();
    const double r0 = r0_over_rs * rs;
    const double v = pw_circular_speed(m, space::Length(r0)).value().value();
    const double period = pw_circular_period(m, space::Length(r0)).value().value();
    SimState state = hole_and_probe(sgr_mass, r0, radial_fraction * v, tangential_factor * v);
    const PaczynskiWiitaGravity gravity;
    Yoshida4 integrator;
    EventStepper stepper;
    EventLog log;
    // A fixed step cannot follow the plunge, where the PW force diverges: the
    // step is limited to 1/50 of the time to cover the gap to the horizon
    // (never below the horizon floor), so the infall is resolved down to
    // r_s and the swept horizon test sees the crossing.
    const double dt_orbit = period / 2000.0;
    const double floor_gap = PaczynskiWiitaGravity::horizon_floor_fraction * rs;
    const double t_end = orbits * period;
    OrbitOutcome out{r0, r0, false, 0.0};
    double r = r0;
    for (long long s = 0; s < 10'000'000 && state.time().value() < t_end; ++s) {
        const double speed = norm(state.bodies().velocity(1)).value();
        const double dt = std::min(dt_orbit, 0.02 * std::max(r - rs, floor_gap) / speed);
        const auto result = stepper.step(integrator, state, space::Time(dt), gravity, log);
        REQUIRE(result);
        if (state.bodies().size() < 2) {
            out.absorbed = true;
            out.absorbed_after_orbits = state.time().value() / period;
            break;
        }
        r = norm(state.bodies().position(1) - state.bodies().position(0)).value();
        REQUIRE(std::isfinite(r));
        out.r_min = std::min(out.r_min, r);
        out.r_max = std::max(out.r_max, r);
    }
    if (out.absorbed) {
        REQUIRE(log.size() == 1);
        REQUIRE(std::holds_alternative<HorizonCrossingEvent>(log.events()[0]));
    } else {
        REQUIRE(log.empty());
    }
    return out;
}

} // namespace

TEST_CASE("Speed of light and characteristic black hole radii", "[core][black_hole]") {
    // c = 299792458 m/s * 86400 s / 149597870700 m.
    CHECK_THAT(constants::speed_of_light_space.value(), WithinRel(173.1446326742403, 1e-15));

    const space::Length rs = schwarzschild_radius(space::Mass(1.0));
    const double rs_m = rs.value() * constants::astronomical_unit.value();
    report("schwarzschild_radius(1 Msun) = " + sci(rs.value()) + " AU = " + sci(rs_m) + " m");
    // 2 GM_sun / c^2 with the exact k^2-derived GM_sun: 2953.25 m.
    CHECK_THAT(rs_m, WithinRel(2.0 * constants::gm_sun_si.value() /
                                   (constants::speed_of_light_si.value() *
                                    constants::speed_of_light_si.value()),
                               1e-14));
    CHECK_THAT(rs_m, WithinAbs(2953.25, 0.01));

    const space::Mass m(sgr_mass);
    CHECK(schwarzschild_radius(m).value() == sgr_mass * rs.value());
    CHECK(photon_sphere_radius(m).value() == 1.5 * schwarzschild_radius(m).value());
    CHECK(isco_radius(m).value() == 3.0 * schwarzschild_radius(m).value());
}

TEST_CASE("Tidal radius and PW circular orbit helpers", "[core][black_hole]") {
    const auto rt = tidal_radius(space::Length(0.00465), space::Mass(1.0), space::Mass(1.0e6));
    REQUIRE(rt);
    CHECK_THAT(rt->value(), WithinRel(0.465, 1e-14));
    CHECK_FALSE(tidal_radius(space::Length(0.0), space::Mass(1.0), space::Mass(1.0e6)));
    CHECK_FALSE(tidal_radius(space::Length(1.0), space::Mass(0.0), space::Mass(1.0e6)));

    const space::Mass m(sgr_mass);
    const double rs = schwarzschild_radius(m).value();
    const double r = 7.0 * rs;
    const double v = pw_circular_speed(m, space::Length(r)).value().value();
    CHECK_THAT(v * v, WithinRel(k2 * sgr_mass * r / ((r - rs) * (r - rs)), 1e-14));
    CHECK_THAT(
        pw_circular_period(m, space::Length(r)).value().value(),
        WithinRel(2.0 * std::numbers::pi * r * (r - rs) / std::sqrt(k2 * sgr_mass * r), 1e-14));
    CHECK_FALSE(pw_circular_speed(m, space::Length(rs)));
    CHECK_FALSE(pw_circular_speed(m, space::Length(0.5 * rs)));
    CHECK_FALSE(pw_circular_period(space::Mass(0.0), space::Length(1.0)));
}

TEST_CASE("PW gravity is bit-identical to Newtonian gravity without black holes",
          "[core][black_hole][paczynski]") {
    BodyStore store;
    add_or_throw(store, BodyInit{"A", Star{}, space::Mass(1.0), at(0.1, -0.2, 0.05), {}});
    add_or_throw(store, BodyInit{"B", Planet{}, space::Mass(1.0e-3), at(5.2, 0.3, -0.1), {}});
    add_or_throw(store, BodyInit{"C", Moon{}, space::Mass(3.0e-6), at(-0.7, 0.8, 0.02), {}});
    add_or_throw(store, BodyInit{"P", Probe{}, space::Mass(1.0e-12), at(1.0, 1.0, 0.0), {}});
    const auto pw = PaczynskiWiitaGravity::create(space::Length(1.0e-4));
    const auto newton = NewtonianGravity::create(space::Length(1.0e-4));
    REQUIRE(pw);
    REQUIRE(newton);
    CHECK(accelerations_of(pw.value(), store) == accelerations_of(newton.value(), store));
    CHECK(pw.value().potential_energy(store, positions_of(store)) ==
          newton.value().potential_energy(store, positions_of(store)));
    CHECK_FALSE(PaczynskiWiitaGravity::create(space::Length(-1.0)));
    CHECK(pw.value().name() == "paczynski_wiita");
}

TEST_CASE("PW force reduces to Newtonian far from the horizon", "[core][black_hole][paczynski]") {
    const space::Mass m(sgr_mass);
    const double rs = schwarzschild_radius(m).value();
    const PaczynskiWiitaGravity pw;
    const NewtonianGravity newton;
    for (const double k : {1.0e2, 1.0e4, 1.0e6}) {
        const SimState state = hole_and_probe(sgr_mass, k * rs, 0.0, 0.0);
        const double a_pw = norm(accelerations_of(pw, state.bodies())[1]).value();
        const double a_n = norm(accelerations_of(newton, state.bodies())[1]).value();
        const double rel = a_pw / a_n - 1.0;
        // Exact: (r / (r - r_s))^2 - 1 = 2 (r_s/r) + 3 (r_s/r)^2 + ...
        const double x = 1.0 / k;
        const double exact = 1.0 / ((1.0 - x) * (1.0 - x)) - 1.0;
        report("PW vs Newton at r = " + sci(k) + " r_s: relative difference " + sci(rel) + " = " +
               sci(rel / x) + " r_s/r");
        CHECK_THAT(rel, WithinRel(exact, 1e-6));
        CHECK(rel / x > 1.99);
        CHECK(rel / x < 2.04);
    }
}

TEST_CASE("PW pair forces conserve momentum and match the potential",
          "[core][black_hole][paczynski]") {
    BodyStore store;
    add_or_throw(store, BodyInit{"BH", BlackHole{}, space::Mass(1.0e6), at(0.01, 0.0, 0.0), {}});
    add_or_throw(store, BodyInit{"BH2", BlackHole{}, space::Mass(3.0e5), at(0.4, 0.2, 0.0), {}});
    add_or_throw(store, BodyInit{"Star", Star{}, space::Mass(1.0), at(-0.3, 0.1, 0.05), {}});
    add_or_throw(store, BodyInit{"Planet", Planet{}, space::Mass(1.0e-3), at(-0.3, 0.2, 0.0), {}});
    const PaczynskiWiitaGravity pw;
    const auto acc = accelerations_of(pw, store);
    Vec3<decltype(space::Mass{} * space::Acceleration{})> total{};
    double scale = 0.0;
    for (std::size_t i = 0; i < store.size(); ++i) {
        total += store.mass(i) * acc[i];
        scale += store.mass(i).value() * norm(acc[i]).value();
    }
    const double residual = norm(total).value() / scale;
    report("PW sum(m a) / sum(m |a|) = " + sci(residual));
    CHECK(residual < 1e-15);

    // -dU/dx of the star by central differences equals m a_x.
    const std::size_t star = 2;
    auto pos = positions_of(store);
    const double h = 1.0e-4;
    pos[star].x += space::Length(h);
    const double up = pw.potential_energy(store, pos).value();
    pos[star].x -= space::Length(2.0 * h);
    const double down = pw.potential_energy(store, pos).value();
    const double force_x = -(up - down) / (2.0 * h);
    CHECK_THAT(force_x, WithinRel(store.mass(star).value() * acc[star].x.value(), 1e-6));
}

TEST_CASE("PW force is finite at and inside the horizon", "[core][black_hole][paczynski]") {
    const space::Mass m(1.0);
    const double rs = schwarzschild_radius(m).value();
    const PaczynskiWiitaGravity pw;
    const double floor_gap = PaczynskiWiitaGravity::horizon_floor_fraction * rs;
    const double cap = k2 / (floor_gap * floor_gap);
    for (const double f : {0.0, 1.0e-9, 0.5, 1.0, 1.0005, 1.001, 1.01, 2.0}) {
        for (const BodyKind& kind : {BodyKind{Star{}}, BodyKind{Probe{}}}) {
            BodyStore store;
            add_or_throw(store, BodyInit{"BH", BlackHole{}, m, {}, {}});
            add_or_throw(store,
                         BodyInit{"Body", kind, space::Mass(1.0e-3), at(f * rs, 0.0, 0.0), {}});
            const auto acc = accelerations_of(pw, store);
            CHECK(isfinite(acc[0]));
            CHECK(isfinite(acc[1]));
            CHECK(std::isfinite(pw.potential_energy(store, positions_of(store)).value()));
            if (f == 0.0) {
                CHECK(acc[0] == Vec3<space::Acceleration>{});
                CHECK(acc[1] == Vec3<space::Acceleration>{});
                continue;
            }
            // Pulled towards the hole, never beyond the floor value.
            CHECK(acc[1].x.value() < 0.0);
            CHECK(-acc[1].x.value() <= cap * (1.0 + 1e-12));
            if (f <= 1.0) {
                CHECK_THAT(-acc[1].x.value(), WithinRel(cap, 1e-12));
            }
        }
    }
    // Two coincident black holes.
    BodyStore pair;
    add_or_throw(pair, BodyInit{"A", BlackHole{}, m, {}, {}});
    add_or_throw(pair, BodyInit{"B", BlackHole{}, m, {}, {}});
    const auto acc = accelerations_of(pw, pair);
    CHECK(acc[0] == Vec3<space::Acceleration>{});
    CHECK(acc[1] == Vec3<space::Acceleration>{});
    CHECK(std::isfinite(pw.potential_energy(pair, positions_of(pair)).value()));
    report("PW acceleration cap at the horizon floor (1 Msun) = " + sci(cap) + " AU/day^2");
}

TEST_CASE("PW force stays accurate at extreme separations", "[core][black_hole][paczynski]") {
    const PaczynskiWiitaGravity pw;
    const NewtonianGravity newton;

    // Black hole of 1e30 Msun and a 1 Msun star at 1e110 AU: r^2 = 1e220 and
    // (r - r_s)^2 r = 1e330 overflows, but the acceleration G M / (r - r_s)^2
    // = 2.959e-194 AU/day^2 is representable.
    {
        const double m_hole = 1.0e30;
        const double r = 1.0e110;
        const double rs = schwarzschild_radius(space::Mass(m_hole)).value();
        BodyStore store;
        add_or_throw(store, BodyInit{"BH", BlackHole{}, space::Mass(m_hole), {}, {}});
        add_or_throw(store, BodyInit{"Star", Star{}, space::Mass(1.0), at(r, 0.0, 0.0), {}});
        const auto acc = accelerations_of(pw, store);
        const double expected_star = -(k2 * m_hole / (r - rs)) / (r - rs);
        const double expected_hole = (k2 / (r - rs)) / (r - rs);
        report("PW 1e30 Msun hole, star at 1e110 AU: a = " + sci(acc[1].x.value()) +
               " AU/day^2 (analytic " + sci(expected_star) + ")");
        CHECK(acc[1].x.value() != 0.0);
        CHECK_THAT(acc[1].x.value(), WithinRel(expected_star, 1e-14));
        CHECK(acc[1].y.value() == 0.0);
        CHECK_THAT(acc[0].x.value(), WithinRel(expected_hole, 1e-14));
        // Force times gap equals minus the pair potential: F x = G M m.
        const double u = pw.potential_energy(store, positions_of(store)).value();
        CHECK_THAT(u, WithinRel(-k2 * m_hole / (r - rs), 1e-14));
        CHECK_THAT(-acc[1].x.value() * (r - rs), WithinRel(-u, 1e-14));
    }

    // Force/potential consistency far from the horizon floor at a scale where
    // (r - r_s)^2 r overflows (r^2 = 2^800 is inside the documented range): central
    // difference of U around r = 2^400 AU.
    {
        const double m_hole = 1.0e6;
        const double r = 0x1p+400;
        const double h = 0x1p+380;
        BodyStore store;
        add_or_throw(store, BodyInit{"BH", BlackHole{}, space::Mass(m_hole), {}, {}});
        add_or_throw(store, BodyInit{"Star", Star{}, space::Mass(1.0), at(0.0, r, 0.0), {}});
        const auto acc = accelerations_of(pw, store);
        auto pos = positions_of(store);
        pos[1].y = space::Length(r + h);
        const double up = pw.potential_energy(store, pos).value();
        pos[1].y = space::Length(r - h);
        const double down = pw.potential_energy(store, pos).value();
        const double force_y = -(up - down) / (2.0 * h);
        CHECK(acc[1].y.value() != 0.0);
        CHECK(up != 0.0);
        CHECK_THAT(force_y, WithinRel(acc[1].y.value(), 1e-8));
        CHECK_THAT(acc[1].y.value(), WithinRel(-(k2 * m_hole / r) / r, 1e-14));
    }

    // Without black holes the Newtonian pairs take NewtonianGravity's scaled
    // path: a 1 Msun star and a probe at 1e110 AU (inverse cube 1e-330 is
    // subnormal) and at 1e140 AU (inverse cube 1e-420 underflows to zero).
    for (const double r : {1.0e110, 1.0e140}) {
        BodyStore store;
        add_or_throw(store, BodyInit{"Star", Star{}, space::Mass(1.0), {}, {}});
        add_or_throw(store, BodyInit{"Probe", Probe{}, space::Mass(1.0e-12), at(r, 0.0, 0.0), {}});
        const auto acc = accelerations_of(pw, store);
        const double expected = -(k2 / r) / r;
        INFO("probe at r = " << r);
        CHECK(acc[1].x.value() != 0.0);
        CHECK_THAT(acc[1].x.value(), WithinRel(expected, 1e-14));
        CHECK(acc == accelerations_of(newton, store));

        BodyStore pair;
        add_or_throw(pair, BodyInit{"Star", Star{}, space::Mass(1.0), {}, {}});
        add_or_throw(pair, BodyInit{"Planet", Planet{}, space::Mass(1.0e-3), at(r, 0.0, 0.0), {}});
        const auto acc_pair = accelerations_of(pw, pair);
        CHECK_THAT(acc_pair[1].x.value(), WithinRel(expected, 1e-14));
        CHECK(acc_pair == accelerations_of(newton, pair));
        const double u = pw.potential_energy(pair, positions_of(pair)).value();
        CHECK_THAT(u, WithinRel(-k2 * 1.0e-3 / r, 1e-14));
        CHECK(u == newton.potential_energy(pair, positions_of(pair)).value());
    }
}

TEST_CASE("Circular PW orbit period matches the analytic value", "[core][black_hole][paczynski]") {
    const space::Mass m(sgr_mass);
    const double rs = schwarzschild_radius(m).value();
    const double r0 = 10.0 * rs;
    const double v = pw_circular_speed(m, space::Length(r0)).value().value();
    const double period = pw_circular_period(m, space::Length(r0)).value().value();
    const double kepler = 2.0 * std::numbers::pi * std::sqrt(r0 * r0 * r0 / (k2 * sgr_mass));

    SimState state = hole_and_probe(sgr_mass, r0, 0.0, v);
    const PaczynskiWiitaGravity gravity;
    Yoshida4 integrator;
    const int steps = 4000;
    const double dt = period / steps;
    // Unwrapped polar angle; the period is the time at which it reaches 2 pi
    // (linear interpolation between the two bracketing steps).
    double previous_angle = 0.0;
    double unwrapped = 0.0;
    double measured = 0.0;
    double radius_error = 0.0;
    for (int s = 1; s <= steps + 100 && measured == 0.0; ++s) {
        step_or_throw(integrator, state, space::Time(dt), gravity);
        const auto p = state.bodies().position(1);
        const double angle = std::atan2(p.y.value(), p.x.value());
        double delta = angle - previous_angle;
        if (delta < -std::numbers::pi) {
            delta += 2.0 * std::numbers::pi;
        }
        const double before = unwrapped;
        unwrapped += delta;
        previous_angle = angle;
        radius_error = std::max(radius_error, std::abs(norm(p).value() / r0 - 1.0));
        if (unwrapped >= 2.0 * std::numbers::pi) {
            const double fraction = (2.0 * std::numbers::pi - before) / (unwrapped - before);
            measured = (s - 1 + fraction) * dt;
        }
    }
    const double rel = std::abs(measured / period - 1.0);
    report("PW circular orbit r = 10 r_s: period measured " + sci(measured) + " d, analytic " +
           sci(period) + " d, relative error " + sci(rel) + "; Newtonian period " + sci(kepler) +
           " d; max |r/r0 - 1| = " + sci(radius_error));
    CHECK(rel < 1e-6);
    CHECK(radius_error < 1e-8);
    // PW and Newton differ by the factor (r - r_s)/r = 0.9 at 10 r_s.
    CHECK_THAT(period / kepler, WithinRel(0.9, 1e-12));
}

TEST_CASE("ISCO: perturbed orbits above 3 r_s stay bound, below 3 r_s they plunge",
          "[core][black_hole][paczynski][isco]") {
    // Small perturbation: tangential speed scaled by (1 - 1e-3) plus a radial
    // kick of 1e-3 of the circular speed.
    const double tangential = 1.0 - 1.0e-3;
    const double radial = -1.0e-3;
    const double rs = schwarzschild_radius(space::Mass(sgr_mass)).value();
    for (const double r0 : {3.5, 4.0, 6.0, 10.0}) {
        const OrbitOutcome o = run_orbit(r0, tangential, radial, 30.0);
        report("ISCO stable r0 = " + sci(r0) + " r_s, 30 orbits: r_min/r0 = " +
               sci(o.r_min / (r0 * rs)) + ", r_max/r0 = " + sci(o.r_max / (r0 * rs)) +
               ", absorbed = " + (o.absorbed ? "yes" : "no"));
        CHECK_FALSE(o.absorbed);
        CHECK(o.r_min / (r0 * rs) > 0.95);
        CHECK(o.r_max / (r0 * rs) < 1.05);
    }
    for (const double r0 : {2.5, 2.9}) {
        const OrbitOutcome o = run_orbit(r0, tangential, radial, 60.0);
        report("ISCO unstable r0 = " + sci(r0) + " r_s: absorbed = " + (o.absorbed ? "yes" : "no") +
               " after " + sci(o.absorbed_after_orbits) + " orbits, r_min/r0 = " +
               sci(o.r_min / (r0 * rs)) + ", r_max/r0 = " + sci(o.r_max / (r0 * rs)));
        CHECK(o.absorbed);
    }
}
