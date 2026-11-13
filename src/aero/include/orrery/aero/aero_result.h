#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "orrery/aero/flow_conditions.h"
#include "orrery/geom/vector3.h"

namespace orrery::aero {

// A result produced outside the solver's validity envelope, or under a
// condition the user should know about. Solvers never refuse to compute
// because of one (ADR-009): the warning travels with the result to the UI.
struct ValidityWarning {
    // Stable identifier for tests and the UI, "<solver>.<what>", for example
    // "newtonian.mach" or "panel.angle_of_attack".
    std::string code;
    // Human-readable explanation with the offending value and the envelope.
    std::string message;
};

// Surface pressure, one sample per surface element of the solver: a panel
// of the panel method, a triangle for Newtonian theory, a wall sample for a
// volume solver. All vectors have the same length.
struct SurfaceField {
    // Element centres and outward unit normals, body axes, m.
    std::vector<geom::Vector3> points;
    std::vector<geom::Vector3> normals;
    // Element areas, m^2 (m per unit span for a 2D solver).
    std::vector<double> areas;
    // Pressure coefficient (p - p_inf) / q_inf.
    std::vector<double> cp;
};

// Velocity on a regular grid (optional; volume solvers). values[i + nx (j +
// ny k)] is the velocity, m/s, at origin + spacing (i, j, k). A 2D field has
// dims[2] == 1.
struct VelocityField {
    std::array<std::size_t, 3> dims{};
    geom::Vector3 origin;
    double spacing = 0.0;
    std::vector<geom::Vector3> values;
};

struct AeroResult {
    // Force coefficients along FlowConditions::drag_direction, lift_direction
    // and side_direction, normalised by q_inf * reference area; pitching
    // moment coefficient about the y axis through the moment centre (nose up
    // positive) normalised by q_inf * area * length.
    double cd = 0.0;
    double cl = 0.0;
    double cy = 0.0;
    double cm = 0.0;
    // Breakdown of cd where a solver separates it; all are part of cd except
    // cd_friction. cd_pressure: integrated surface pressure. cd_induced:
    // induced drag from the wake (Trefftz plane), 3D lifting solvers only.
    double cd_pressure = 0.0;
    double cd_induced = 0.0;
    // Skin-friction estimate, reported separately and NOT included in cd or
    // in `force` (inviscid solvers have no friction of their own).
    double cd_friction = 0.0;

    // Total aerodynamic force (N) and moment about the moment centre (N m),
    // body axes; per unit span for a 2D solver.
    geom::Vector3 force;
    geom::Vector3 moment;

    SurfaceField surface;
    std::optional<VelocityField> velocity;
    // Stagnation-point convective heat flux, W/m^2, where a solver estimates
    // it (indicative only).
    std::optional<double> stagnation_heat_flux;

    // Residual of each iteration (iterative or time-marching solvers; a
    // direct solve reports the final relative residual of the linear system).
    std::vector<double> residuals;
    std::size_t iterations = 0;
    bool converged = false;

    std::vector<ValidityWarning> warnings;

    [[nodiscard]] bool has_warning(std::string_view code) const noexcept;
};

// Fills cd, cl, cy and cm of `result` from its force and moment.
// Throws std::invalid_argument if the dynamic pressure, the reference area or
// the reference length is not positive and finite.
void set_coefficients(AeroResult& result, const FlowConditions& conditions,
                      const ReferenceValues& reference);

} // namespace orrery::aero
