#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "orrery/aero/compute/compute_backend.h"
#include "orrery/aero/lbm/d2q9.h"
#include "orrery/aero/lbm/solid_mask.h"
#include "orrery/core/result.h"

// Two-dimensional lattice Boltzmann solver, D2Q9 BGK (ADR-008), in lattice
// units (dx = dt = 1, reference density 1). One fused step per time step:
// every fluid node pulls its post-streaming populations from its neighbours
// (applying the boundary conditions while it gathers) and collides them.
//
//   Collision   BGK, f* = f - (f - f_eq) / tau_eff + F_i (Guo forcing), with
//               tau_eff = tau, or the Smagorinsky closure (APPROXIMATION A2)
//               tau_eff = (tau + sqrt(tau^2 + 18 sqrt(2) C^2 Q / rho)) / 2,
//               Q = |Pi_neq| (Hou et al. 1996).
//   Solids      halfway bounce-back on mask nodes (A4); the momentum
//               exchange method gives the force on them.
//   x           periodic, or a regularized Zou-He velocity inlet at x = 0
//               (Zou and He 1997; Latt et al. 2008) and an outlet at
//               x = nx - 1: convective (Yang 2013) or zero-gradient
//               extrapolation of the incoming populations.
//   y           per side: periodic, free slip (specular reflection) or a
//               no-slip wall halfway outside the domain (y = -1/2 or
//               ny - 1/2), optionally moving along x (Couette).
//
// The CPU solver (oneTBB, double precision) is the ground truth; the GPU
// solver runs the same step through an IComputeBackend in binary32 with
// shifted populations f - w (docs/lbm.md, "GPU precision").
namespace orrery::aero::lbm {

enum class XBoundary : std::uint8_t { periodic, inflow_outflow };

// Outlet treatment of the populations entering from x = nx (directions 3, 6
// and 7). extrapolation copies them from the outlet column (zero gradient);
// convective solves d f / dt + U d f / dx = 0 implicitly with U the inlet
// speed, f(t+1) = (f(t) + U f_upstream(t+1)) / (1 + U), which lets vortices
// leave with far less reflection.
enum class Outlet : std::uint8_t { convective, extrapolation };
enum class WallKind : std::uint8_t { periodic, slip, no_slip };

struct Wall {
    WallKind kind = WallKind::slip;
    double velocity_x = 0.0; // tangential wall speed, no_slip only
};

// APPROXIMATION A2: the constant C is not tuned per flow; 0.1 to 0.2 is the
// usual range for LBM large-eddy simulation.
struct Smagorinsky {
    bool enabled = false;
    double constant = 0.1;
};

struct Lbm2DConfig {
    std::size_t nx = 0;
    std::size_t ny = 0;
    double tau = 0.6; // BGK relaxation time, nu = (tau - 1/2) / 3
    XBoundary x_boundary = XBoundary::inflow_outflow;
    Outlet outlet = Outlet::convective;
    Wall bottom; // y = 0 side
    Wall top;    // y = ny - 1 side
    double inlet_ux = 0.0;
    double inlet_uy = 0.0;
    double body_force_x = 0.0; // force density, lattice units
    double body_force_y = 0.0;
    Smagorinsky smagorinsky;
};

// Empty when the configuration and the mask can run, otherwise the reason:
// sizes (at least 3 x 3, mask matching), tau > 1/2, finite parameters, a
// lattice Mach number of the inlet and walls below 0.1, periodic walls on
// both y sides or neither, a fluid inlet and outlet column, and a positive
// inlet speed for the convective outlet.
[[nodiscard]] std::optional<std::string> validate(const Lbm2DConfig& config,
                                                  const SolidMask2D& mask);

// Kinematic viscosity of a relaxation time, (tau - 1/2) / 3, and back.
[[nodiscard]] constexpr double viscosity_of_tau(double tau) noexcept {
    return (tau - 0.5) * d2q9::cs2;
}
[[nodiscard]] constexpr double tau_of_viscosity(double nu) noexcept {
    return nu / d2q9::cs2 + 0.5;
}

// A fluid node and a direction j whose source node x - c_j is solid: the
// population f_j of the node is reflected from the wall in the next step.
struct BoundaryLink {
    std::uint32_t cell = 0;
    std::uint32_t direction = 0;
};
[[nodiscard]] std::vector<BoundaryLink> boundary_links(const Lbm2DConfig& config,
                                                       const SolidMask2D& mask);

// Macroscopic state at the last completed step (moments of the
// post-collision populations, velocity with the Guo half-force correction).
// Solid nodes report density 1 and zero velocity.
struct MacroFields2D {
    std::size_t nx = 0;
    std::size_t ny = 0;
    std::vector<double> density;
    std::vector<double> velocity_x;
    std::vector<double> velocity_y;
    std::vector<std::uint8_t> solid;
};

// Gauge pressure c_s^2 (rho - 1), lattice units.
[[nodiscard]] std::vector<double> pressure(const MacroFields2D& fields);
// Vorticity d(uy)/dx - d(ux)/dy by central differences (one-sided next to a
// solid node or the domain edge; periodic directions wrap); 0 on solids.
[[nodiscard]] std::vector<double> vorticity(const MacroFields2D& fields, const Lbm2DConfig& config);
// Largest |u| over fluid nodes.
[[nodiscard]] double max_speed(const MacroFields2D& fields) noexcept;

struct Force2D {
    double x = 0.0;
    double y = 0.0;
};

// Initial macroscopic state of a fluid node; populations start at its
// equilibrium.
using InitialState = std::function<d2q9::Moments(std::size_t x, std::size_t y)>;

// Equilibrium populations of `initial` at every node (solids at rest), in
// the solver layout f[i * nx * ny + x + nx * y].
[[nodiscard]] std::vector<double> initial_populations(const Lbm2DConfig& config,
                                                      const SolidMask2D& mask,
                                                      const InitialState& initial);

// Uniform state: density 1 and the inlet velocity.
[[nodiscard]] InitialState uniform_inlet_state(const Lbm2DConfig& config);

using Status = core::Result<std::monostate, std::string>;

// A D2Q9 solver instance (CPU or GPU). Populations are reported in the
// layout of initial_populations().
class Lbm2DSolver {
public:
    Lbm2DSolver() = default;
    Lbm2DSolver(const Lbm2DSolver&) = delete;
    Lbm2DSolver& operator=(const Lbm2DSolver&) = delete;
    Lbm2DSolver(Lbm2DSolver&&) = delete;
    Lbm2DSolver& operator=(Lbm2DSolver&&) = delete;
    virtual ~Lbm2DSolver() = default;

    [[nodiscard]] virtual const Lbm2DConfig& config() const noexcept = 0;
    [[nodiscard]] virtual const SolidMask2D& mask() const noexcept = 0;
    [[nodiscard]] virtual std::uint64_t steps() const noexcept = 0;

    // Replaces the state by the equilibrium of `initial` and resets the step
    // count.
    [[nodiscard]] virtual Status initialize(const InitialState& initial) = 0;
    [[nodiscard]] virtual Status step(std::size_t count) = 0;

    [[nodiscard]] virtual core::Result<MacroFields2D, std::string> macroscopic() = 0;
    [[nodiscard]] virtual core::Result<std::vector<double>, std::string> populations() = 0;
    // Force on the solid nodes from the momentum exchange method (Ladd 1994):
    // F = sum over boundary links of 2 c_{opp(j)} f*_{opp(j)}(x), the
    // momentum the reflected populations hand to the wall in one step.
    [[nodiscard]] virtual core::Result<Force2D, std::string> force() = 0;
    // Total mass sum of populations over fluid nodes.
    [[nodiscard]] virtual core::Result<double, std::string> mass() = 0;
};

// Initialised with uniform_inlet_state(config).
[[nodiscard]] core::Result<std::unique_ptr<Lbm2DSolver>, std::string>
make_cpu_solver(const Lbm2DConfig& config, SolidMask2D mask);

// GPU solver on `backend`, which must provide the kernels lbm_d2q9_step,
// lbm_d2q9_moments and lbm_d2q9_force and outlive the solver.
[[nodiscard]] core::Result<std::unique_ptr<Lbm2DSolver>, std::string>
make_gpu_solver(compute::IComputeBackend& backend, const Lbm2DConfig& config, SolidMask2D mask);

// Names of the compute kernels the GPU solver dispatches.
inline constexpr std::string_view kernel_step = "lbm_d2q9_step";
inline constexpr std::string_view kernel_moments = "lbm_d2q9_moments";
inline constexpr std::string_view kernel_force = "lbm_d2q9_force";

} // namespace orrery::aero::lbm
