#pragma once

#include "orrery/geom/vector3.h"

namespace orrery::aero {

// Free-stream state seen by a body, in SI units (ADR-001: aero works in SI).
//
// Body axes: x downstream along the body (chord direction of a wing, leading
// edge first), y along the span (to the right), z up. At zero angle of attack
// and sideslip the free stream blows along +x. A positive angle of attack
// tilts the free stream towards +z (nose up, positive lift on a symmetric
// wing); a positive sideslip tilts it towards +y.
struct FlowConditions {
    // Free-stream speed |V|, m/s.
    double speed = 0.0;
    // Free-stream density, kg/m^3.
    double density = 1.225;
    // Dynamic viscosity mu, Pa s.
    double viscosity = 1.789e-5;
    // Static temperature, K.
    double temperature = 288.15;
    // Angle of attack and sideslip, radians.
    double angle_of_attack = 0.0;
    double sideslip = 0.0;
    // Ratio of specific heats and specific gas constant of the medium (air:
    // 1.4 and 287.05 J/(kg K)); they set the speed of sound and therefore the
    // Mach number, which is derived, never given.
    double gamma = 1.4;
    double gas_constant = 287.05;

    // Speed of sound sqrt(gamma R T), m/s.
    [[nodiscard]] double speed_of_sound() const noexcept;
    [[nodiscard]] double mach() const noexcept;
    // Reynolds number rho V L / mu for the length `length` (m).
    [[nodiscard]] double reynolds(double length) const noexcept;
    // 1/2 rho V^2, Pa.
    [[nodiscard]] double dynamic_pressure() const noexcept;

    // Unit free-stream direction (cos a cos b, sin b, sin a cos b): the drag
    // direction.
    [[nodiscard]] geom::Vector3 drag_direction() const noexcept;
    // Unit lift direction: perpendicular to the free stream in the plane of
    // x and z, (-sin a, 0, cos a) at zero sideslip.
    [[nodiscard]] geom::Vector3 lift_direction() const noexcept;
    // Unit side-force direction, lift x drag (completes a right-handed set).
    [[nodiscard]] geom::Vector3 side_direction() const noexcept;
    // Free-stream velocity vector, speed * drag_direction(), m/s.
    [[nodiscard]] geom::Vector3 velocity() const noexcept;
};

// What the aerodynamic coefficients are normalised by. The caller chooses the
// area (frontal, planform, wetted: geom::reference_quantities). For a 2D
// solver the area and the moments are per unit span (area = chord * 1 m).
struct ReferenceValues {
    // Reference area, m^2.
    double area = 1.0;
    // Reference length for the pitching moment and the Reynolds number, m.
    double length = 1.0;
    // Point the moment is taken about, body axes, m.
    geom::Vector3 moment_centre;
};

} // namespace orrery::aero
