#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "orrery/aero/aero_solver.h"
#include "orrery/aero/flow_conditions.h"
#include "orrery/aero/regime.h"
#include "orrery/atmo/error.h"
#include "orrery/core/quantity.h"
#include "orrery/core/result.h"

namespace orrery::atmo {

// One point of a drag table.
struct CdSample {
    double mach = 0.0;
    double cd = 0.0;

    friend bool operator==(const CdSample&, const CdSample&) = default;
};

// Value of the table at one Mach number.
struct CdLookup {
    double cd = 0.0;
    // True when the Mach number lies outside [min_mach, max_mach] and the
    // value is the nearest end value.
    bool clamped = false;
};

// Drag coefficient versus Mach number of one vehicle, with the reference area
// the coefficient is normalised by: the typed value that carries the aero
// domain's result into the drag model (ADR-014).
//
// Interpolation: linear in Mach between neighbouring samples.
// Clamping policy: below the first and above the last sample the end value is
// held constant (no extrapolation) and the lookup reports clamped = true; the
// table never invents a trend outside the range that was sampled or measured.
// APPROXIMATION (E5): end values held outside the sampled range - safer than
// extrapolating a few samples - valid while Cd varies little beyond them.
// A single sample is a constant Cd. Deterministic, no allocation in lookup().
class AeroCoefficientTable {
public:
    // Constant Cd 1 over a 1 m^2 reference area.
    AeroCoefficientTable();

    // Validates and stores literal data: at least one sample, Mach numbers
    // finite, non-negative and strictly increasing, Cd finite and positive,
    // reference area finite and positive. Fails with invalid_table otherwise.
    [[nodiscard]] static core::Result<AeroCoefficientTable, AtmoError>
    create(std::vector<CdSample> samples, core::si::Area reference_area);

    // A single-sample table.
    [[nodiscard]] static core::Result<AeroCoefficientTable, AtmoError>
    constant(double cd, core::si::Area reference_area);

    [[nodiscard]] CdLookup lookup(double mach) const noexcept;
    [[nodiscard]] double cd(double mach) const noexcept { return lookup(mach).cd; }

    [[nodiscard]] std::span<const CdSample> samples() const noexcept { return samples_; }
    [[nodiscard]] core::si::Area reference_area() const noexcept { return reference_area_; }
    [[nodiscard]] double min_mach() const noexcept { return samples_.front().mach; }
    [[nodiscard]] double max_mach() const noexcept { return samples_.back().mach; }

    friend bool operator==(const AeroCoefficientTable&, const AeroCoefficientTable&) = default;

private:
    AeroCoefficientTable(std::vector<CdSample> samples, core::si::Area reference_area) noexcept;

    std::vector<CdSample> samples_;
    core::si::Area reference_area_{};
};

// Builds a solver for a regime. Returns nullptr when that solver is not
// available to the caller (for example the lattice Boltzmann solver without a
// compute backend).
using AeroSolverFactory = std::function<std::unique_ptr<aero::IAeroSolver>(aero::SolverKind)>;

// Newtonian (with `nose_radius` for its heat-flux estimate, 0 to skip it) and
// the 3D panel method; nullptr for the lattice Boltzmann solver.
[[nodiscard]] AeroSolverFactory default_aero_solver_factory(double nose_radius = 0.0);

// Default Mach numbers of a sampled table: the hypersonic range where
// modified Newtonian theory is valid and a capsule spends its heating pulse.
inline constexpr std::array<double, 5> default_sampling_mach{5.0, 10.0, 15.0, 20.0, 25.0};

struct CdSamplingRequest {
    // Body, its reference values (the table's reference area is
    // geometry.reference.area) and orientation in body axes.
    aero::AeroGeometry geometry;
    // Strictly increasing, positive Mach numbers.
    std::vector<double> mach_numbers{default_sampling_mach.begin(), default_sampling_mach.end()};
    // Free-stream template: density, temperature, viscosity, gas properties
    // and attitude; the speed is set to Mach * speed of sound per sample.
    aero::FlowConditions freestream;
    // Forces one solver for every sample; otherwise aero::select_solver
    // recommends one per Mach number.
    std::optional<aero::SolverKind> solver_override;
    // Upper bound on IAeroSolver::step() calls per sample.
    std::size_t max_steps = 100000;
};

// Provenance of one sampled point.
struct CdSamplingPoint {
    double mach = 0.0;
    double cd = 0.0;
    aero::SolverKind solver = aero::SolverKind::newtonian;
    bool within_envelope = false;
    bool converged = false;
    std::size_t steps = 0;
    // Validity warning codes of the solver's result.
    std::vector<std::string> warnings;
};

struct CdSamplingResult {
    AeroCoefficientTable table;
    std::vector<CdSamplingPoint> points;
};

// Samples Cd(Mach) from the aerodynamic solvers: for each Mach number the
// regime selector (or the override) picks a solver, `factory` builds it, and
// the solver steps until it reports a final result or max_steps is reached.
// Envelope warnings are recorded, never blocking (ADR-009). Fails with
// invalid_parameter for bad Mach numbers or reference values,
// solver_unavailable when the factory returns nullptr and solver_failed when
// a solver throws or returns a non-finite or non-positive Cd.
[[nodiscard]] core::Result<CdSamplingResult, AtmoError>
sample_cd_table(const CdSamplingRequest& request, const AeroSolverFactory& factory);

} // namespace orrery::atmo
