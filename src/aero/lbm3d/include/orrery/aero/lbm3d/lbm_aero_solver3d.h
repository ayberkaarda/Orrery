#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "orrery/aero/aero_solver.h"
#include "orrery/aero/compute/compute_backend.h"
#include "orrery/aero/lbm/force_history.h"
#include "orrery/aero/lbm/lattice_units.h"
#include "orrery/aero/lbm/lbm_aero_solver.h"
#include "orrery/aero/lbm3d/lbm3d.h"

// The 3D lattice Boltzmann solver behind the common IAeroSolver interface
// (ADR-009): a closed body mesh in SI units goes in; coefficients, forces,
// wall pressure, the velocity field and validity warnings come out.
//
// Geometry: AeroGeometry::mesh (closed, outward, body axes, metres) and its
// reference values. The lattice is wind-aligned: lattice x runs along the
// free stream (FlowConditions::drag_direction), y along the side direction
// and z along the lift direction, so angle of attack and sideslip rotate the
// body, not the channel. The body is voxelized with orrery_geom (generalized
// winding number at every node of its bounding box) and placed `upstream`
// reference lengths behind a velocity inlet, `downstream` lengths ahead of a
// convective outlet, in a square cross-section `width` lengths wide whose
// sides are free slip or periodic.
//
// Time-marching: every step() advances `steps_per_call` lattice steps; the
// force is sampled at the lattice steps that are multiples of `sample_every`
// (an absolute schedule, independent of the call size), and the result holds
// the moving average over the samples of the last `average_window`
// convective times L / U.
namespace orrery::aero::lbm {

// Largest lattice set up when the device does not report its memory (and
// for the CPU solver): 256^3 nodes, the top of the 64^3 .. 256^3 range of
// the specification (about 2.9 GB on a GPU, 5.1 GB for the CPU solver).
inline constexpr std::size_t default_max_cells_3d = std::size_t{1} << 24U;

struct LbmSolver3DOptions {
    LatticeResolution resolution{16.0, 0.05}; // cells per reference length, u_lat
    double upstream = 5.0;                    // reference lengths from the inlet to the body
    double downstream = 10.0;                 // reference lengths from the body to the outlet
    double width = 6.0;                       // side of the square cross-section, reference lengths
    WallKind sides = WallKind::slip;          // y and z sides: slip or periodic
    // Body wall: interpolated bounce-back with the link fractions from the
    // narrow-band signed distance of orrery_geom (default), or the halfway
    // staircase wall.
    SolidWall wall = SolidWall::interpolated;
    LesMode les = LesMode::automatic;
    double smagorinsky_constant = 0.1;
    double duration = 30.0;      // convective times L / U simulated
    double average_window = 8.0; // convective times in the moving average
    // Gradual start: the fluid starts at rest and the inlet speed rises as
    // U (1 - cos(pi t / T)) / 2 over T = `start_ramp` convective times, so
    // the start launches no pressure pulse that would ring between the
    // inlet and the sides (APPROXIMATION D4). 0 starts impulsively from the
    // uniform free stream.
    double start_ramp = 3.0;
    // Absorbing layers (Lbm3DConfig::sponge, APPROXIMATION D3): strength and
    // thicknesses in reference lengths; a zero strength turns them off.
    double sponge_strength = 0.1;
    double sponge_inlet = 0.0;
    double sponge_outlet = 2.0;
    double sponge_sides = 0.0;
    std::size_t steps_per_call = 200;
    std::size_t sample_every = 10; // lattice steps between force samples
    // Largest lattice accepted; 0 chooses default_max_cells_3d. On a GPU the
    // device memory the backend reports (free memory, largest binding) is
    // checked as well. A larger lattice is refused with the largest
    // resolution that would fit.
    std::size_t max_cells = 0;
    // Run on this backend's GPU solver (must outlive the solver), or on the
    // CPU when empty.
    compute::IComputeBackend* backend = nullptr;
};

// Drag, lift and side-force coefficients of one sample (wind axes).
struct Coefficients3D {
    double drag = 0.0;
    double lift = 0.0;
    double side = 0.0;
};

class LbmAeroSolver3D final : public IAeroSolver {
public:
    explicit LbmAeroSolver3D(LbmSolver3DOptions options = {}) noexcept
        : options_(options), history_(1) {}

    [[nodiscard]] SolverKind kind() const noexcept override {
        return SolverKind::lattice_boltzmann;
    }
    // Throws std::invalid_argument for a mesh that fails geom::validate_mesh
    // or covers no lattice node, non-finite or non-positive conditions or
    // reference values, invalid options (no_slip sides included), lattice
    // units that cannot be built (lattice Mach number of 0.1 or more), a
    // lattice above the cell budget (options.max_cells or
    // default_max_cells_3d) or the device memory, a duration or averaging
    // window of 2^53 lattice steps or more (all checked before the lattice is
    // allocated), or a backend that cannot run it.
    void setup(const AeroGeometry& geometry, const FlowConditions& conditions) override;
    bool step() override;
    [[nodiscard]] const AeroResult& result() const noexcept override { return result_; }

    // Diagnostics beyond AeroResult (valid after setup()). The history and
    // the instantaneous coefficients are normalised exactly like result():
    // set_coefficients with the reference values of the geometry. The
    // history's drag and lift are the drag and lift coefficients;
    // side_history() holds the side-force coefficient of the same samples.
    [[nodiscard]] const CoefficientHistory& history() const noexcept { return history_; }
    [[nodiscard]] const std::vector<double>& side_history() const noexcept { return side_; }
    [[nodiscard]] Coefficients3D instantaneous() const noexcept { return instantaneous_; }
    [[nodiscard]] const std::optional<LatticeUnits>& units() const noexcept { return units_; }
    [[nodiscard]] Lbm3DSolver* lattice() const noexcept { return solver_.get(); }
    [[nodiscard]] std::uint64_t total_steps() const noexcept { return total_steps_; }
    // Frontal area of the voxelized body over the channel cross-section.
    [[nodiscard]] double blockage() const noexcept { return blockage_; }

private:
    void update_result();
    // SI force and moment (body axes) and the coefficients of a lattice force
    // (wind axes, moment about the moment centre): the one normalisation of
    // the history and the result.
    void apply_force(AeroResult& result, const Force3D& lattice, const LatticeUnits& units) const;

    LbmSolver3DOptions options_;
    FlowConditions conditions_;
    ReferenceValues reference_;
    std::optional<LatticeUnits> units_;
    std::unique_ptr<Lbm3DSolver> solver_;
    std::vector<ValidityWarning> setup_warnings_;
    CoefficientHistory history_;
    std::vector<double> side_;
    std::vector<Force3D> samples_; // lattice force, moment about the centre
    Coefficients3D instantaneous_;
    AeroResult result_;
    // Wind axes in body coordinates: lattice x, y and z.
    std::array<geom::Vector3, 3> axes_{};
    std::array<double, 3> centre_{}; // moment centre, lattice coordinates
    std::array<double, 3> origin_{}; // wind-frame position (m) of lattice node (0, 0, 0)
    double surface_area_ = 0.0;      // mesh area shared among the wall samples
    double blockage_ = 0.0;
    std::uint64_t total_steps_ = 0;
    double ramp_steps_ = 0.0; // lattice steps of the gradual start
    std::size_t window_samples_ = 1;
    double previous_cd_ = 0.0;
};

} // namespace orrery::aero::lbm
