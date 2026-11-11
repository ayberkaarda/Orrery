#pragma once

// Shared fixtures for the gravity and integrator tests (space units).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>

#include "orrery/core/angle.h"
#include "orrery/core/body_store.h"
#include "orrery/core/force_model.h"
#include "orrery/core/integrator.h"
#include "orrery/core/kepler.h"
#include "orrery/core/sim_state.h"
#include "orrery/core/units.h"

namespace orrery::core::test {

inline KeplerElements elements(double a, double e, double i_deg, double node_deg, double arg_deg,
                               double mean_deg) {
    KeplerElements el;
    el.semi_major_axis = space::Length(a);
    el.eccentricity = e;
    el.inclination = Angle::from_degrees(i_deg);
    el.longitude_of_ascending_node = Angle::from_degrees(node_deg);
    el.argument_of_periapsis = Angle::from_degrees(arg_deg);
    el.mean_anomaly = Angle::from_degrees(mean_deg);
    return el;
}

inline space::GravitationalParameter two_body_mu(double m_primary, double m_secondary) {
    return space::GravitationalParameter(constants::gravitational_constant_space.value() *
                                         (m_primary + m_secondary));
}

inline CartesianState to_cartesian_or_throw(const KeplerElements& el,
                                            space::GravitationalParameter mu) {
    const auto state = to_cartesian(el, mu);
    if (!state) {
        throw std::runtime_error(std::string("to_cartesian failed: ") +
                                 std::string(to_string(state.error())));
    }
    return state.value();
}

inline void add_or_throw(BodyStore& store, BodyInit init) {
    if (!store.add(std::move(init))) {
        throw std::runtime_error("BodyStore::add failed");
    }
}

// Primary (index 0) and secondary (index 1) in the barycentric frame, with
// the secondary on the given orbit relative to the primary.
inline SimState two_body(double m_primary, double m_secondary, const KeplerElements& relative) {
    const CartesianState rel = to_cartesian_or_throw(relative, two_body_mu(m_primary, m_secondary));
    const double total = m_primary + m_secondary;
    const double f_primary = m_secondary / total;
    const double f_secondary = m_primary / total;
    BodyStore store;
    add_or_throw(store, BodyInit{"Primary", Star{}, space::Mass(m_primary),
                                 -rel.position * f_primary, -rel.velocity * f_primary});
    add_or_throw(store, BodyInit{"Secondary", Planet{}, space::Mass(m_secondary),
                                 rel.position * f_secondary, rel.velocity * f_secondary});
    return SimState(space::Time(0.0), std::move(store));
}

inline Vec3<space::Length> relative_position(const SimState& state) {
    return state.bodies().position(1) - state.bodies().position(0);
}

struct PlanetData {
    const char* name;
    double mass;      // Msun
    double a;         // AU
    double e;         //
    double i;         // deg
    double node;      // deg, longitude of ascending node
    double varpi;     // deg, longitude of perihelion
    double mean_long; // deg, mean longitude
};

// Approximate J2000 mean elements (Standish, "Keplerian Elements for
// Approximate Positions of the Major Planets", JPL, table 1, 1800-2050) and
// IAU 1976 reciprocal planetary masses (Earth+Moon 328900.56). Earth and Moon are merged into their
// barycentre; its inclination (-1.5e-5 deg) is set to 0. These are realistic initial conditions for
// conservation tests, not an ephemeris.
inline constexpr std::array<PlanetData, 8> planets = {{
    {"Mercury", 1.0 / 6023600.0, 0.38709927, 0.20563593, 7.00497902, 48.33076593, 77.45779628,
     252.25032350},
    {"Venus", 1.0 / 408523.71, 0.72333566, 0.00677672, 3.39467605, 76.67984255, 131.60246718,
     181.97909950},
    {"EarthMoon", 1.0 / 328900.56, 1.00000261, 0.01671123, 0.0, 0.0, 102.93768193, 100.46457166},
    {"Mars", 1.0 / 3098708.0, 1.52371034, 0.09339410, 1.84969142, 49.55953891, -23.94362959,
     -4.55343205},
    {"Jupiter", 1.0 / 1047.3486, 5.20288700, 0.04838624, 1.30439695, 100.47390909, 14.72847983,
     34.39644051},
    {"Saturn", 1.0 / 3497.898, 9.53667594, 0.05386179, 2.48599187, 113.66242448, 92.59887831,
     49.95424423},
    {"Uranus", 1.0 / 22902.98, 19.18916464, 0.04725744, 0.77263783, 74.01692503, 170.95427630,
     313.23810451},
    {"Neptune", 1.0 / 19412.24, 30.06992276, 0.00859048, 1.77004347, 131.78422574, 44.96476227,
     -55.12619052},
}};

// Sun followed by the eight planets, shifted to the barycentric frame.
inline SimState solar_system() {
    std::array<CartesianState, planets.size() + 1> states{};
    std::array<double, planets.size() + 1> masses{};
    masses[0] = 1.0;
    for (std::size_t k = 0; k < planets.size(); ++k) {
        const PlanetData& p = planets[k];
        const KeplerElements el =
            elements(p.a, p.e, p.i, p.node, p.varpi - p.node, p.mean_long - p.varpi);
        states[k + 1] = to_cartesian_or_throw(el, two_body_mu(1.0, p.mass));
        masses[k + 1] = p.mass;
    }
    double total = 0.0;
    Vec3<decltype(space::Mass{} * space::Length{})> moment{};
    Vec3<decltype(space::Mass{} * space::Velocity{})> momentum{};
    for (std::size_t k = 0; k < states.size(); ++k) {
        total += masses[k];
        moment += space::Mass(masses[k]) * states[k].position;
        momentum += space::Mass(masses[k]) * states[k].velocity;
    }
    const Vec3<space::Length> com = moment / space::Mass(total);
    const Vec3<space::Velocity> vcom = momentum / space::Mass(total);

    BodyStore store;
    add_or_throw(store, BodyInit{"Sun", Star{}, space::Mass(1.0), -com, -vcom});
    for (std::size_t k = 0; k < planets.size(); ++k) {
        add_or_throw(store, BodyInit{planets[k].name, Planet{}, space::Mass(planets[k].mass),
                                     states[k + 1].position - com, states[k + 1].velocity - vcom});
    }
    return SimState(space::Time(0.0), std::move(store));
}

inline void step_or_throw(IIntegrator& integrator, SimState& state, space::Time dt,
                          const IForceModel& forces) {
    const auto result = integrator.step(state, dt, forces);
    if (!result) {
        throw std::runtime_error(std::string("integrator step failed: ") +
                                 std::string(to_string(result.error())));
    }
}

// Advances `state` by `steps` fixed steps of `dt`.
inline void advance(IIntegrator& integrator, SimState& state, space::Time dt, long long steps,
                    const IForceModel& forces) {
    for (long long s = 0; s < steps; ++s) {
        step_or_throw(integrator, state, dt, forces);
    }
}

struct LiteralBody {
    const char* name;
    double mass;
    std::array<double, 3> position;
    std::array<double, 3> velocity;
};

// Golden-hash initial state: the barycentric state of solar_system() written
// as exact hexadecimal literals (as computed with glibc). solar_system() goes
// through sin, cos and atan2, whose last-bit results differ between C runtime
// libraries; literals make the determinism scenario identical on every
// platform, so the golden hash tests the integration loop alone.
inline constexpr std::array<LiteralBody, 9> golden_bodies = {{
    {"Sun",
     0x1p+0,
     {-0x1.d3a04fcd8d51cp-8, -0x1.6e697a48f1dbcp-9, 0x1.b041ecad990e4p-13},
     {0x1.68f3dde89f546p-18, -0x1.f09a6ce7660bbp-18, -0x1.951f0622bb863p-24}},
    {"Mercury",
     0x1.6482fc4a0e7b8p-23,
     {-0x1.1908e98ffcb48p-3, -0x1.cce3d38a66469p-2, -0x1.8fa669b322b3cp-6},
     {0x1.5e27368b8e1d3p-6, -0x1.a70ddefa23e74p-8, -0x1.46191b190bf2fp-9}},
    {"Venus",
     0x1.488b15bd302fcp-19,
     {-0x1.736e69eee695ap-1, -0x1.22d56ee763ec4p-5, 0x1.51b041931bc3dp-5},
     {0x1.a5a2fd0e5c1bbp-11, -0x1.4ca1cdbd2ac22p-6, -0x1.534714e14193p-12}},
    {"EarthMoon",
     0x1.9814786263d7dp-19,
     {-0x1.7975c503fd7c5p-3, 0x1.edc853300432ep-1, 0x1.b041ecad990e4p-13},
     {-0x1.19c4a77b607adp-6, -0x1.9fb73a15bdff4p-9, -0x1.951f0622bb863p-24}},
    {"Mars",
     0x1.5a8343722a44fp-22,
     {0x1.622f2ce0137e4p+0, -0x1.093360f0b6858p-6, -0x1.189e3dbe4ce3p-5},
     {0x1.6373a61aa658bp-11, 0x1.f16e9952fa837p-7, 0x1.3c2d1839ee5aap-12}},
    {"Jupiter",
     0x1.f49601333c10cp-11,
     {0x1.fedf2addbdb0dp+1, 0x1.78b173beaefe3p+1, -0x1.9fcabafccaaeap-4},
     {-0x1.2b481cbef7098p-8, 0x1.a54a70db602b8p-8, 0x1.3d3e7c11ec629p-14}},
    {"Saturn",
     0x1.2bc5eeff5e4f5p-12,
     {0x1.9a16ec27f620cp+2, 0x1.a2be6a0bc6e1ep+2, -0x1.79cb9455bfb2p-2},
     {-0x1.183fffc063e9dp-8, 0x1.fd60f105c200ep-9, 0x1.ad2c479f99deep-14}},
    {"Uranus",
     0x1.6e44607a13bd7p-15,
     {0x1.cd62f69664741p+3, -0x1.b7b1b1cd2441dp+3, -0x1.e711d81fdab79p-3},
     {0x1.6034fc21a2e69p-9, 0x1.5c28a95104244p-9, -0x1.a2f12c8f19603p-16}},
    {"Neptune",
     0x1.b0213ca2e9ee2p-15,
     {0x1.0cb82ce15116p+4, -0x1.8ff50e14c677p+4, 0x1.058be216ec096p-3},
     {0x1.53569e5082eefp-9, 0x1.cd90f758b4c98p-10, -0x1.92cdf8ce7f222p-14}},
}};

// golden_bodies plus a probe (1e-12 Msun) at (0, 1, 0) AU moving at
// (-0.01720209895, 0, 0) AU/day, at t = 0.
inline SimState golden_initial_state() {
    BodyStore store;
    for (std::size_t k = 0; k < golden_bodies.size(); ++k) {
        const LiteralBody& b = golden_bodies[k];
        const BodyKind kind = k == 0 ? BodyKind{Star{}} : BodyKind{Planet{}};
        add_or_throw(store,
                     BodyInit{b.name,
                              kind,
                              space::Mass(b.mass),
                              {space::Length(b.position[0]), space::Length(b.position[1]),
                               space::Length(b.position[2])},
                              {space::Velocity(b.velocity[0]), space::Velocity(b.velocity[1]),
                               space::Velocity(b.velocity[2])}});
    }
    add_or_throw(store, BodyInit{"Probe",
                                 Probe{AeroRef{1}},
                                 space::Mass(1.0e-12),
                                 {space::Length(0.0), space::Length(1.0), space::Length(0.0)},
                                 {space::Velocity(-0.01720209895), space::Velocity(0.0),
                                  space::Velocity(0.0)}});
    return SimState(space::Time(0.0), std::move(store));
}

// Golden scenario: golden_initial_state(), Plummer softening 1e-4 AU,
// 10000 steps of 1 day with the given integrator.
inline SimState golden_run(IntegratorKind kind) {
    SimState state = golden_initial_state();
    const auto gravity = NewtonianGravity::create(space::Length(1.0e-4));
    if (!gravity) {
        throw std::runtime_error("invalid softening");
    }
    const auto integrator = make_integrator(kind);
    for (int s = 0; s < 10000; ++s) {
        const auto result = integrator->step(state, space::Time(1.0), gravity.value());
        if (!result) {
            throw std::runtime_error("golden run step failed");
        }
    }
    return state;
}
// Running maximum of a sampled diagnostic. std::max(m, NaN) returns m, so a
// plain maximum silently drops NaN samples; every sample is therefore checked
// with std::isfinite before the reduction and non-finite ones are counted.
struct RunningMax {
    double value = 0.0;
    long long non_finite = 0;

    void add(double sample) {
        if (!std::isfinite(sample)) {
            ++non_finite;
            return;
        }
        value = std::max(value, sample);
    }
    [[nodiscard]] bool all_finite() const { return non_finite == 0; }
    // Every sample was finite and the largest is below `bound`.
    [[nodiscard]] bool below(double bound) const { return all_finite() && value < bound; }
};

// Measured values are printed so that they appear in the test log.
inline void report(const std::string& line) {
    std::printf("MEASURED %s\n", line.c_str());
    std::fflush(stdout);
}

inline std::string sci(double value) {
    std::array<char, 32> buffer{};
    std::snprintf(buffer.data(), buffer.size(), "%.3e", value);
    return std::string(buffer.data());
}

} // namespace orrery::core::test