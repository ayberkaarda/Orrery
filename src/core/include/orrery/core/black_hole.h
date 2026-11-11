#pragma once

#include <optional>

#include "orrery/core/quantity.h"
#include "orrery/core/units.h"

namespace orrery::core {

namespace constants {

// Speed of light in vacuum, exact by definition of the metre (SI 2019).
inline constexpr si::Velocity speed_of_light_si{299792458.0};

// Speed of light in space units: c * 86400 s / 149597870700 m
// = 173.144632674240... AU/day. Derived exactly from the defined constants
// (one rounding of the quotient), so it carries no measurement uncertainty.
inline constexpr space::Velocity speed_of_light_space{speed_of_light_si.value() * day.value() /
                                                      astronomical_unit.value()};

} // namespace constants

// Characteristic radii of a non-rotating (Schwarzschild) black hole of mass M,
// in space units (ADR-003, ADR-004). The stored BlackHole::spin does not enter
// any of them: Kerr geometry is not implemented.

// r_s = 2 G M / c^2, with G = k^2 AU^3 Msun^-1 day^-2 and c in AU/day:
// 1.97412e-8 AU (2953.25 m) per solar mass.
[[nodiscard]] constexpr space::Length schwarzschild_radius(space::Mass mass) noexcept {
    const double c = constants::speed_of_light_space.value();
    return space::Length(2.0 * constants::gravitational_constant_space.value() * mass.value() /
                         (c * c));
}

// Photon sphere, 1.5 r_s (exact Schwarzschild value).
[[nodiscard]] constexpr space::Length photon_sphere_radius(space::Mass mass) noexcept {
    return 1.5 * schwarzschild_radius(mass);
}

// Innermost stable circular orbit, 3 r_s = 6 G M / c^2. Schwarzschild and the
// Paczynski-Wiita potential place it at the same radius (ADR-004).
[[nodiscard]] constexpr space::Length isco_radius(space::Mass mass) noexcept {
    return 3.0 * schwarzschild_radius(mass);
}

// Tidal (Roche-type) disruption radius of a body of radius R and mass m near
// a black hole of mass M: r_t = R (M / m)^(1/3). Empty when R or m is not
// positive (a point or massless body has no tidal radius) or M is not
// positive.
//
// APPROXIMATION: order-of-magnitude tidal radius r_t = R (M/m)^(1/3) - the
// self-gravity of the body is compared with the Newtonian tidal acceleration
// and the order-unity prefactor (about 1 to 2.4 depending on structure and
// rotation) is set to 1 - valid as a disruption criterion to within a factor
// of about 2 in radius; it says nothing about the debris.
[[nodiscard]] std::optional<space::Length>
tidal_radius(space::Length body_radius, space::Mass body_mass, space::Mass black_hole_mass);

// Circular orbit of a test particle in the Paczynski-Wiita potential
// Phi = -G M / (r - r_s):
//   v^2 = G M r / (r - r_s)^2,   Omega = v / r,   T = 2 pi r (r - r_s) / sqrt(G M r).
// Circular orbits exist for every r > r_s; they are stable only for
// r > 3 r_s (ISCO). Empty for r <= r_s or a non-positive mass.
[[nodiscard]] std::optional<space::Velocity> pw_circular_speed(space::Mass black_hole_mass,
                                                               space::Length radius);
[[nodiscard]] std::optional<space::Time> pw_circular_period(space::Mass black_hole_mass,
                                                            space::Length radius);

} // namespace orrery::core
