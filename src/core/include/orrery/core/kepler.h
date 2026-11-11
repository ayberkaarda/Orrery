#pragma once

#include <cstdint>
#include <string_view>

#include "orrery/core/angle.h"
#include "orrery/core/quantity.h"
#include "orrery/core/result.h"
#include "orrery/core/vec3.h"

namespace orrery::core {

// Classical (osculating) orbital elements of a body relative to a central
// gravitational parameter mu = G (M + m), in space units.
//
// Conventions:
//   - Elliptic orbits: 0 <= e < 1, semi_major_axis > 0, mean_anomaly in [0, 2 pi)
//     on output.
//   - Hyperbolic orbits: e > 1, semi_major_axis < 0, mean_anomaly is the
//     hyperbolic mean anomaly M = e sinh(H) - H (not periodic, not wrapped).
//   - Parabolic orbits (e = 1) are not supported (KeplerError::parabolic_orbit).
//   - inclination in [0, pi]; node and periapsis angles in [0, 2 pi) on output.
//   - Circular orbits (e below kepler_tolerance::circular_eccentricity): the
//     argument of periapsis is undefined and reported as 0; the mean anomaly
//     is then measured from the ascending node (argument of latitude).
//   - Equatorial orbits (sin i below kepler_tolerance::equatorial_sine): the
//     node is undefined and reported as 0; angles are measured from the +x axis
//     in the direction of motion.
struct KeplerElements {
    space::Length semi_major_axis{};
    double eccentricity = 0.0;
    Angle inclination;
    Angle longitude_of_ascending_node;
    Angle argument_of_periapsis;
    Angle mean_anomaly;
};

// Position and velocity relative to the central body, in space units.
struct CartesianState {
    Vec3<space::Length> position{};
    Vec3<space::Velocity> velocity{};
};

enum class KeplerError : std::uint8_t {
    // mu is not finite or not positive.
    invalid_gravitational_parameter,
    // An element or state component is NaN or infinite.
    non_finite_input,
    // Negative eccentricity.
    invalid_eccentricity,
    // Inclination outside [0, pi].
    invalid_inclination,
    // |e - 1| below kepler_tolerance::parabolic_band.
    parabolic_orbit,
    // Sign of the semi-major axis does not match the eccentricity
    // (a > 0 requires e < 1, a < 0 requires e > 1), or a is zero.
    inconsistent_semi_major_axis,
    // Zero radius or (nearly) rectilinear motion: angular momentum vanishes.
    degenerate_state,
    // Hyperbolic true anomaly outside the asymptotes, 1 + e cos(nu) <= 0.
    beyond_asymptote,
    // The anomaly solver did not converge (overflow for extreme hyperbolic
    // mean anomalies), or a result is not representable as a finite double
    // (extreme input magnitudes). No function returns non-finite values.
    no_convergence,
};

[[nodiscard]] std::string_view to_string(KeplerError error) noexcept;

// The circular and equatorial thresholds sit just above rounding noise:
// treating an orbit as circular or equatorial misplaces the state by about
// 2 e or sin(i) relative, so the thresholds bound that error. Above them the
// undefined angle itself is ill-conditioned (absolute error of order
// 1e-16 / e or 1e-16 / sin(i) radians), but the elements still reproduce the
// state to rounding precision because the errors cancel in the longitudes.
namespace kepler_tolerance {
// Below this eccentricity the orbit is treated as circular.
inline constexpr double circular_eccentricity = 1.0e-13;
// Below this value of sin(i) the orbit is treated as equatorial.
inline constexpr double equatorial_sine = 1.0e-14;
// Orbits with |e - 1| below this value are rejected as parabolic. Near the
// band the semi-major axis loses precision as 1e-16 / |1 - e|.
inline constexpr double parabolic_band = 1.0e-9;
// Orbits whose specific angular momentum is below this fraction of |r| |v|
// are rejected as rectilinear.
inline constexpr double rectilinear_sine = 1.0e-12;
} // namespace kepler_tolerance

// Solves Kepler's equation M = E - e sin(E) for the eccentric anomaly,
// 0 <= e < 1. Safeguarded Newton iteration on a bracket that always contains
// the root, started from Danby's guess E0 = M + 0.85 e sign(sin M). The result
// lies in the same revolution as M.
[[nodiscard]] Result<Angle, KeplerError> eccentric_anomaly_from_mean(Angle mean_anomaly,
                                                                     double eccentricity);

// Solves M = e sinh(H) - H for the hyperbolic anomaly, e > 1. Safeguarded
// Newton iteration on the bracket [asinh(M / e), asinh(M / (e - 1))].
[[nodiscard]] Result<Angle, KeplerError> hyperbolic_anomaly_from_mean(Angle mean_anomaly,
                                                                      double eccentricity);

// Mean anomaly from the eccentric (e < 1) or hyperbolic (e > 1) anomaly.
[[nodiscard]] Angle mean_anomaly_from_eccentric(Angle eccentric_anomaly, double eccentricity);
[[nodiscard]] Angle mean_anomaly_from_hyperbolic(Angle hyperbolic_anomaly, double eccentricity);

// True anomaly <-> eccentric / hyperbolic anomaly.
[[nodiscard]] Angle true_anomaly_from_eccentric(Angle eccentric_anomaly, double eccentricity);
[[nodiscard]] Angle eccentric_anomaly_from_true(Angle true_anomaly, double eccentricity);
[[nodiscard]] Angle true_anomaly_from_hyperbolic(Angle hyperbolic_anomaly, double eccentricity);
// The true anomaly must lie strictly between the asymptotes,
// 1 + e cos(nu) > 0; otherwise KeplerError::beyond_asymptote is returned.
[[nodiscard]] Result<Angle, KeplerError> hyperbolic_anomaly_from_true(Angle true_anomaly,
                                                                      double eccentricity);

// Elements -> position and velocity.
[[nodiscard]] Result<CartesianState, KeplerError> to_cartesian(const KeplerElements& elements,
                                                               space::GravitationalParameter mu);

// Position and velocity -> osculating elements.
[[nodiscard]] Result<KeplerElements, KeplerError> to_elements(const CartesianState& state,
                                                              space::GravitationalParameter mu);

// Mean motion n = sqrt(mu / |a|^3), radians per day. Valid for elliptic
// (a > 0) and hyperbolic (a < 0) orbits.
[[nodiscard]] Result<space::Frequency, KeplerError> mean_motion(space::Length semi_major_axis,
                                                                space::GravitationalParameter mu);

// Orbital period T = 2 pi sqrt(a^3 / mu) of an elliptic orbit (a > 0).
[[nodiscard]] Result<space::Time, KeplerError> orbital_period(space::Length semi_major_axis,
                                                              space::GravitationalParameter mu);

} // namespace orrery::core
