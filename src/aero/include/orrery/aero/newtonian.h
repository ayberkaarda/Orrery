#pragma once

#include "orrery/aero/aero_solver.h"

namespace orrery::aero {

// Stagnation pressure coefficient (p_0 - p_inf) / q_inf behind a normal
// shock (Rayleigh pitot formula) for Mach >= 1, isentropic stagnation below:
//   p02 / p1 = [(g+1)^2 M^2 / (4 g M^2 - 2 (g-1))]^(g/(g-1)) (1 - g + 2 g M^2) / (g+1)
//   Cp_max   = 2 / (g M^2) (p02 / p1 - 1)
// (Anderson, *Hypersonic and High-Temperature Gas Dynamics*, sec. 3.2).
// Tends to 1.839 for gamma = 1.4 as Mach -> infinity. Throws
// std::invalid_argument for Mach <= 0 or gamma <= 1.
[[nodiscard]] double newtonian_cp_max(double mach, double gamma);

// Sutton-Graves stagnation-point convective heat flux, W/m^2:
// q = k sqrt(rho / R_n) V^3 (Sutton and Graves, NASA TR R-376, 1971), with
// k = 1.7415e-4 kg^0.5 / m for Earth air.
// APPROXIMATION (A8): indicative only; no radiative heating, no wall
// catalysis or chemistry, cold wall.
inline constexpr double sutton_graves_earth = 1.7415e-4;
[[nodiscard]] double sutton_graves_heat_flux(double density, double speed, double nose_radius,
                                             double k = sutton_graves_earth);

struct NewtonianOptions {
    // Nose radius for the Sutton-Graves estimate, m; 0 skips the estimate.
    double nose_radius = 0.0;
    double sutton_graves_constant = sutton_graves_earth;
};

// Modified Newtonian impact theory on the triangles of AeroGeometry::mesh:
// Cp = Cp_max sin^2(theta) on windward triangles (sin theta = -n . V_hat),
// Cp = 0 in the shadow, Cp_max from newtonian_cp_max with the free-stream
// Mach number and gamma (Lees 1955).
// APPROXIMATION (A7): no shock-layer structure, centrifugal correction or
// shadow-side pressure; meant for Mach >= 5 (a warning is attached below).
// Throws from setup() for an empty or non-finite mesh.
class NewtonianSolver final : public IAeroSolver {
public:
    explicit NewtonianSolver(NewtonianOptions options = {}) : options_(options) {}

    [[nodiscard]] SolverKind kind() const noexcept override { return SolverKind::newtonian; }
    void setup(const AeroGeometry& geometry, const FlowConditions& conditions) override;
    bool step() override;
    [[nodiscard]] const AeroResult& result() const noexcept override { return result_; }

private:
    NewtonianOptions options_;
    geom::TriangleMesh mesh_;
    ReferenceValues reference_;
    FlowConditions conditions_;
    bool ready_ = false;
    AeroResult result_;
};

} // namespace orrery::aero
