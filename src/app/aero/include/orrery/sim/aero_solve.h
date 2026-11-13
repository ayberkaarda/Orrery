#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "orrery/aero/aero_solver.h"
#include "orrery/core/result.h"
#include "orrery/scene/aero_experiment.h"
#include "orrery/sim/aero_flow.h"
#include "orrery/sim/aero_geometry.h"

// Solvers of the Aero workspace (docs/aero-workspace.md, "Solvers"): which
// IAeroSolver runs for a body and a regime choice, the diagnostics beyond
// AeroResult the panels show, and the record a finished run leaves in its
// experiment. Graphics-free: the lattice Boltzmann solvers run on the CPU
// (oneTBB) here.
namespace orrery::sim {

// Lattice steps of one IAeroSolver::step() call of the lattice Boltzmann
// solvers: the granularity of "steps per frame" (a frame runs
// ceil(steps_per_frame / granularity) calls).
inline constexpr std::size_t kLbmStepsPerCall2D = 50;
inline constexpr std::size_t kLbmStepsPerCall3D = 20;

struct SolverPlan {
    aero::SolverKind kind = aero::SolverKind::panel;
    bool two_dimensional = false;
    // Nose radius for the Newtonian heat-flux estimate (0: none), m.
    double nose_radius_m = 0.0;
};

[[nodiscard]] std::size_t lbm_steps_per_call(bool two_dimensional) noexcept;

// The solver for a plan, configured from the experiment's settings (lattice
// resolution and duration when given). Throws std::invalid_argument for a
// plan no solver covers (Newtonian in 2D).
[[nodiscard]] std::unique_ptr<aero::IAeroSolver>
make_solver(const SolverPlan& plan, const scene::AeroSolverSettings& settings);

// What a solver reports beyond AeroResult: the force history and lattice of
// the lattice Boltzmann solvers.
struct RunDiagnostics {
    std::vector<double> history_step; // lattice step of each force sample
    std::vector<double> history_cd;   // instantaneous coefficients
    std::vector<double> history_cl;
    std::optional<double> strouhal;  // 2D lattice Boltzmann
    std::uint64_t lattice_steps = 0; // advanced so far
    std::uint64_t planned_steps = 0; // of the whole run (0: not time-marching)
    std::string lattice;             // "640 x 320" or "160 x 60 x 60"
    double blockage = 0.0;           // 3D lattice Boltzmann
};

[[nodiscard]] RunDiagnostics diagnostics(const aero::IAeroSolver& solver,
                                         const scene::AeroSolverSettings& settings);

// Lattice steps a time-marching solver has advanced (0 for a direct solver).
[[nodiscard]] std::uint64_t lattice_steps(const aero::IAeroSolver& solver) noexcept;

// The result record of a run for its experiment.
[[nodiscard]] scene::AeroResultRecord record_result(const aero::AeroResult& result,
                                                    const RunDiagnostics& diagnostics,
                                                    aero::SolverKind solver,
                                                    const FlowSummary& flow,
                                                    const aero::ReferenceValues& reference);

// Everything needed to start a run: the body, the experiment it belongs to,
// and the solver chosen for it.
struct AeroRunSpec {
    std::shared_ptr<const PreparedGeometry> geometry;
    scene::AeroExperiment experiment;

    // Resolves the solver (regime recommendation or override, substituted
    // for availability) and the flow summary of this spec.
    [[nodiscard]] FlowSummary flow() const;
    [[nodiscard]] SolverPlan plan() const;
};

struct SolveOutcome {
    aero::AeroResult result;
    RunDiagnostics diagnostics;
    FlowSummary flow;
    aero::SolverKind solver = aero::SolverKind::panel;
    aero::ReferenceValues reference;
    // The solver reported completion (step() returned true).
    bool finished = false;
    // The solver itself, for its flow field (capture_flow).
    std::shared_ptr<aero::IAeroSolver> solver_instance;
};

// Runs a spec on the calling thread: setup, then step() until the solver
// finishes or, for a time-marching solver, `max_lattice_steps` have been
// advanced (nullopt: until it finishes). Fails with the solver's message for
// an unusable geometry or conditions (validity warnings never fail).
[[nodiscard]] core::Result<SolveOutcome, std::string>
solve(const AeroRunSpec& spec, std::optional<std::uint64_t> max_lattice_steps = std::nullopt);

} // namespace orrery::sim
