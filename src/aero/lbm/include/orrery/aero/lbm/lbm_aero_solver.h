#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "orrery/aero/aero_solver.h"
#include "orrery/aero/compute/compute_backend.h"
#include "orrery/aero/lbm/force_history.h"
#include "orrery/aero/lbm/lattice_units.h"
#include "orrery/aero/lbm/lbm2d.h"

// The 2D lattice Boltzmann solver behind the common IAeroSolver interface
// (ADR-009): a body section in SI units goes in, coefficients, forces, wall
// pressure, the velocity field and validity warnings come out.
//
// Geometry: AeroGeometry::section (x-z plane, metres) and its reference
// values; the mesh is not used. The section is rotated by the angle of
// attack so the free stream runs along the lattice x axis (the channel walls
// stay parallel to it), and placed `upstream` reference lengths behind the
// inlet in a channel `height` reference lengths high with free-slip sides.
// Time-marching: every step() advances `steps_per_call` lattice steps; the
// force is sampled at the lattice steps that are multiples of `sample_every`
// (an absolute schedule, independent of the call size), and the result holds
// the moving average of the force coefficients over the samples of the last
// `average_window` convective times L / U.
namespace orrery::aero::lbm {

enum class LesMode : std::uint8_t {
    automatic, // Smagorinsky above Re 2000 (APPROXIMATION A2)
    on,
    off,
};

struct LbmSolverOptions {
    LatticeResolution resolution{40.0, 0.05}; // cells per reference length, u_lat
    double upstream = 8.0;                    // reference lengths from the inlet to the section
    double downstream = 16.0;                 // reference lengths from the section to the outlet
    double height = 16.0;                     // channel height, reference lengths
    LesMode les = LesMode::automatic;
    double smagorinsky_constant = 0.1;
    double duration = 60.0;      // convective times L / U simulated
    double average_window = 6.0; // convective times in the moving average
    std::size_t steps_per_call = 500;
    std::size_t sample_every = 10; // lattice steps between force samples
    // Run on this backend's GPU solver (must outlive the solver), or on the
    // CPU when empty.
    compute::IComputeBackend* backend = nullptr;
};

class LbmAeroSolver2D final : public IAeroSolver {
public:
    explicit LbmAeroSolver2D(LbmSolverOptions options = {}) noexcept
        : options_(options), history_(1) {}

    [[nodiscard]] SolverKind kind() const noexcept override {
        return SolverKind::lattice_boltzmann;
    }
    // Throws std::invalid_argument for a section with fewer than 3 points,
    // non-finite or non-positive conditions or reference values, lattice
    // units that cannot be built (lattice Mach number of 0.1 or more), a
    // domain beyond geom::max_grid_cells, a duration or averaging window of
    // 2^53 lattice steps or more (checked before anything is allocated), or a
    // backend that cannot run it.
    void setup(const AeroGeometry& geometry, const FlowConditions& conditions) override;
    bool step() override;
    [[nodiscard]] const AeroResult& result() const noexcept override { return result_; }

    // Diagnostics beyond AeroResult (valid after setup()). The history and
    // the instantaneous coefficients are normalised exactly like result():
    // set_coefficients with the reference area of the geometry.
    [[nodiscard]] const CoefficientHistory& history() const noexcept { return history_; }
    [[nodiscard]] Coefficients instantaneous() const noexcept { return instantaneous_; }
    // Strouhal number f L / U of the lift oscillation over the averaging
    // window, when one is found.
    [[nodiscard]] std::optional<double> strouhal() const;
    [[nodiscard]] const std::optional<LatticeUnits>& units() const noexcept { return units_; }
    [[nodiscard]] Lbm2DSolver* lattice() const noexcept { return solver_.get(); }
    // Lattice position of the moment centre.
    [[nodiscard]] std::array<double, 2> moment_centre() const noexcept { return centre_; }

private:
    void update_result();
    // SI force and moment (body axes, per unit span) and the coefficients of
    // a lattice force: the one normalisation of the history and the result.
    void apply_force(AeroResult& result, const Force2D& lattice) const;

    LbmSolverOptions options_;
    FlowConditions conditions_;
    ReferenceValues reference_;
    std::optional<LatticeUnits> units_;
    std::unique_ptr<Lbm2DSolver> solver_;
    std::vector<ValidityWarning> setup_warnings_;
    CoefficientHistory history_;
    std::vector<Force2D> samples_; // lattice force, moment about the centre
    Coefficients instantaneous_;
    AeroResult result_;
    std::array<double, 2> centre_{}; // moment centre, lattice coordinates
    std::array<double, 2> origin_{}; // wind-frame position (m) of lattice node (0, 0)
    std::uint64_t total_steps_ = 0;
    std::size_t window_samples_ = 1;
    double previous_cd_ = 0.0;
};

} // namespace orrery::aero::lbm
