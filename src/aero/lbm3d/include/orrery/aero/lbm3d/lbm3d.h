#pragma once

#include <array>
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
#include "orrery/aero/lbm/lbm2d.h"
#include "orrery/aero/lbm3d/d3q19.h"
#include "orrery/aero/lbm3d/solid_mask3d.h"
#include "orrery/core/result.h"

// Three-dimensional lattice Boltzmann solver, D3Q19 BGK (ADR-008), in lattice
// units (dx = dt = 1, reference density 1). The same scheme as the D2Q9
// solver (lbm2d.h), extended by one axis: one fused step per time step in
// which every fluid node pulls its post-streaming populations from its
// neighbours (applying the boundary conditions while it gathers) and
// collides them.
//
//   Collision   BGK with Guo forcing; optional Smagorinsky closure
//               tau_eff = (tau + sqrt(tau^2 + 18 sqrt(2) C^2 Q / rho)) / 2,
//               Q = |Pi_neq| (APPROXIMATION A2, Hou et al. 1996).
//   Solids      halfway bounce-back on mask nodes (A4), or the linear
//               interpolated bounce-back of Bouzidi, Firdaouss and Lallemand
//               (2001) with the wall placed on each link from the mask's
//               signed distance; momentum exchange gives the force and
//               moment on them.
//   x           periodic, or a velocity inlet at x = 0 (Zou and He 1997:
//               density from the known populations, unknowns by bounce-back
//               of the non-equilibrium part; then regularized, Latt et al.
//               2008) and an outlet at x = nx - 1, convective (Yang 2013) or
//               zero-gradient extrapolation.
//   y, z        per side: periodic, free slip (specular reflection) or a
//               no-slip wall halfway outside the domain, optionally moving
//               along x (Couette); the moving-wall term is scaled by the
//               density of the wall node.
//
// The CPU solver (oneTBB, double precision) is the ground truth; the GPU
// solver runs the same step through an IComputeBackend in binary32 with
// populations shifted by their weights (docs/lbm.md, "GPU precision").
namespace orrery::aero::lbm {

// Absorbing layers (APPROXIMATION D3; Xu and Sagaut 2013): within
// `thickness` nodes of the inlet, of the outlet, and of every side that is
// not periodic, the post-collision populations relax towards the free-stream
// equilibrium (density 1, the inlet velocity),
//   f* <- f* - sigma (f* - f_eq(1, u_in)),
//   sigma = strength ((thickness - d) / thickness)^2,
// d the distance in nodes from the boundary (the largest sigma of the
// layers a node is in). Pressure waves of an impulsive start then leave the
// domain instead of ringing between the inlet and the walls. Off when the
// strength or every thickness is zero; only with the inflow/outflow x
// boundary.
struct Sponge {
    double strength = 0.0;
    std::size_t inlet = 0;  // nodes
    std::size_t outlet = 0; // nodes
    std::size_t sides = 0;  // nodes, non-periodic y and z sides
};

// Solid wall treatment. halfway: the wall sits halfway along every link
// to a solid node (staircase, first order in the resolution for curved
// walls, A4). interpolated: the wall sits at the fraction q of the link
// where the mask's signed distance changes sign (linear interpolation of the
// distance between the two nodes), and the reflected population is
// interpolated (Bouzidi et al. 2001):
//   q < 1/2:  f_j(x, t+1) = 2q f*_o(x) + (1 - 2q) f*_o(x + c_j),
//   q >= 1/2: f_j(x, t+1) = f*_o(x) / (2q) + (2q - 1) / (2q) f*_j(x),
// o = opposite(j); a link with q < 1/2 whose second node x + c_j is not a
// fluid node of the domain falls back to q = 1/2. Not mass conserving
// exactly (APPROXIMATION D2).
enum class SolidWall : std::uint8_t { halfway, interpolated };

struct Lbm3DConfig {
    std::size_t nx = 0;
    std::size_t ny = 0;
    std::size_t nz = 0;
    double tau = 0.6; // BGK relaxation time, nu = (tau - 1/2) / 3
    XBoundary x_boundary = XBoundary::inflow_outflow;
    Outlet outlet = Outlet::convective;
    Wall y_low;  // y = 0 side; Wall::velocity_x is the tangential wall speed
    Wall y_high; // y = ny - 1 side
    Wall z_low;  // z = 0 side
    Wall z_high; // z = nz - 1 side
    double inlet_ux = 0.0;
    double inlet_uy = 0.0;
    double inlet_uz = 0.0;
    double body_force_x = 0.0; // force density, lattice units
    double body_force_y = 0.0;
    double body_force_z = 0.0;
    Smagorinsky smagorinsky;
    Sponge sponge;
    SolidWall solid_wall = SolidWall::halfway; // interpolated needs mask distances
};

// Empty when the configuration and the mask can run, otherwise the reason:
// sizes (at least 3 nodes per axis, at most geom::max_grid_cells nodes with
// the product checked without wrapping, mask matching), tau > 1/2, finite
// parameters, a lattice Mach number of the inlet and walls below 0.1,
// periodic walls on both sides of an axis or neither, a fluid inlet and
// outlet face, a positive inlet speed for the convective outlet, absorbing
// layers that fit, and finite mask distances for the interpolated wall.
[[nodiscard]] std::optional<std::string> validate(const Lbm3DConfig& config,
                                                  const SolidMask3D& mask);

// A fluid node and a direction j whose source node x - c_j is solid: the
// population f_j of the node is reflected from the wall in the next step.
// The wall sits at x - q c_j (q = 1/2 for the halfway wall).
struct BoundaryLink3D {
    std::uint32_t cell = 0;
    std::uint32_t direction = 0;
    double q = 0.5;
};
[[nodiscard]] std::vector<BoundaryLink3D> boundary_links(const Lbm3DConfig& config,
                                                         const SolidMask3D& mask);

// Macroscopic state at the last completed step (moments of the
// post-collision populations, velocity with the Guo half-force correction).
// Solid nodes report density 1 and zero velocity.
struct MacroFields3D {
    std::size_t nx = 0;
    std::size_t ny = 0;
    std::size_t nz = 0;
    std::vector<double> density;
    std::vector<double> velocity_x;
    std::vector<double> velocity_y;
    std::vector<double> velocity_z;
    std::vector<std::uint8_t> solid;
};

// Largest |u| over fluid nodes.
[[nodiscard]] double max_speed(const MacroFields3D& fields) noexcept;

// Force on the solid nodes and its moment about the lattice origin, sum of
// r x F with each link force applied at the wall point of the link (halfway
// between the fluid node and its solid neighbour).
struct Force3D {
    std::array<double, 3> force{};
    std::array<double, 3> moment{};
};

// Wall point of a boundary link: node position minus q c_j.
[[nodiscard]] std::array<double, 3> link_wall_point(const Lbm3DConfig& config,
                                                    const BoundaryLink3D& link) noexcept;

// Adds the momentum-exchange force of one link and its moment to `total`:
// c_o (f*_o(x) + f_j(x, t+1)), o = opposite(j), the momentum the population
// leaving towards the wall and the one coming back from it hand over;
// `exchanged` is that sum of the two populations (2 f*_o for the halfway
// wall).
void add_link_force(Force3D& total, const Lbm3DConfig& config, const BoundaryLink3D& link,
                    double exchanged) noexcept;

// Initial macroscopic state of a fluid node; populations start at its
// equilibrium.
using InitialState3D = std::function<d3q19::Moments(std::size_t x, std::size_t y, std::size_t z)>;

// Equilibrium populations of `initial` at every node (solids at rest), in
// the solver layout f[i * cells + x + nx (y + ny z)].
[[nodiscard]] std::vector<double> initial_populations(const Lbm3DConfig& config,
                                                      const SolidMask3D& mask,
                                                      const InitialState3D& initial);

// Uniform state: density 1 and the inlet velocity (rest when x is periodic).
[[nodiscard]] InitialState3D uniform_inlet_state(const Lbm3DConfig& config);

// Memory the solvers allocate for a lattice (populations twice, node
// classes, moments, outlet history), checked without wrapping; empty when
// the count does not fit in std::size_t.
[[nodiscard]] std::optional<std::size_t> gpu_memory_bytes(const Lbm3DConfig& config) noexcept;
[[nodiscard]] std::optional<std::size_t> cpu_memory_bytes(const Lbm3DConfig& config) noexcept;
// Bytes of one population buffer on the device (19 binary32 values per node).
[[nodiscard]] std::optional<std::size_t>
gpu_population_buffer_bytes(const Lbm3DConfig& config) noexcept;

// A D3Q19 solver instance (CPU or GPU). Populations are reported in the
// layout of initial_populations().
class Lbm3DSolver {
public:
    Lbm3DSolver() = default;
    Lbm3DSolver(const Lbm3DSolver&) = delete;
    Lbm3DSolver& operator=(const Lbm3DSolver&) = delete;
    Lbm3DSolver(Lbm3DSolver&&) = delete;
    Lbm3DSolver& operator=(Lbm3DSolver&&) = delete;
    virtual ~Lbm3DSolver() = default;

    [[nodiscard]] virtual const Lbm3DConfig& config() const noexcept = 0;
    [[nodiscard]] virtual const SolidMask3D& mask() const noexcept = 0;
    [[nodiscard]] virtual std::uint64_t steps() const noexcept = 0;

    // Replaces the state by the equilibrium of `initial` and resets the step
    // count.
    [[nodiscard]] virtual Status initialize(const InitialState3D& initial) = 0;
    [[nodiscard]] virtual Status step(std::size_t count) = 0;
    // Velocity the inlet imposes from the next step on (and the target of
    // the absorbing layers), for a gradual start; the convective outlet
    // keeps config().inlet_ux as its convection speed. Fails for a
    // non-finite velocity, a lattice Mach number of 0.1 or more, or periodic
    // x. initialize() restores config()'s inlet velocity.
    [[nodiscard]] virtual Status set_inlet_velocity(const std::array<double, 3>& velocity) = 0;
    [[nodiscard]] virtual std::array<double, 3> inlet_velocity() const noexcept = 0;

    [[nodiscard]] virtual core::Result<MacroFields3D, std::string> macroscopic() = 0;
    [[nodiscard]] virtual core::Result<std::vector<double>, std::string> populations() = 0;
    // Force on the solid nodes from the momentum exchange method (Ladd 1994;
    // Mei, Yu, Shyy and Luo 2002 for the interpolated wall): F = sum over
    // boundary links of c_o (f*_o(x) + f_j(x, t+1)), 2 c_o f*_o(x) for the
    // halfway wall.
    [[nodiscard]] virtual core::Result<Force3D, std::string> force() = 0;
    // Total mass: sum of populations over fluid nodes.
    [[nodiscard]] virtual core::Result<double, std::string> mass() = 0;
    // The boundary links the force is summed over.
    [[nodiscard]] virtual const std::vector<BoundaryLink3D>& links() const noexcept = 0;
};

// Initialised with uniform_inlet_state(config).
[[nodiscard]] core::Result<std::unique_ptr<Lbm3DSolver>, std::string>
make_cpu_solver(const Lbm3DConfig& config, SolidMask3D mask);

// GPU solver on `backend`, which must provide the kernels lbm_d3q19_step,
// lbm_d3q19_moments and lbm_d3q19_force and outlive the solver. Besides
// validate(), refuses before allocating: a real parameter outside the
// binary32 range, a tau that rounds to 1/2 in binary32, a population buffer
// larger than the largest buffer the device can bind, and a lattice whose
// device memory exceeds what the backend reports as available.
[[nodiscard]] core::Result<std::unique_ptr<Lbm3DSolver>, std::string>
make_gpu_solver(compute::IComputeBackend& backend, const Lbm3DConfig& config, SolidMask3D mask);

// Names of the compute kernels the GPU solver dispatches.
inline constexpr std::string_view kernel3d_step = "lbm_d3q19_step";
inline constexpr std::string_view kernel3d_moments = "lbm_d3q19_moments";
inline constexpr std::string_view kernel3d_force = "lbm_d3q19_force";

} // namespace orrery::aero::lbm
