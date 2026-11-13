#include "orrery/sim/aero_solve.h"

#include <cmath>
#include <exception>
#include <span>
#include <stdexcept>
#include <utility>

#include "orrery/aero/lbm/lbm_aero_solver.h"
#include "orrery/aero/lbm3d/lbm_aero_solver3d.h"
#include "orrery/aero/newtonian.h"
#include "orrery/aero/panel_method.h"
#include "orrery/geom/primitives.h"

namespace orrery::sim {

namespace {

// Largest 3D lattice the workspace sets up on the CPU (about 1.2 GB of
// distributions): beyond it the solver refuses with the largest resolution
// that fits.
constexpr std::size_t kMaxCpuCells3D = std::size_t{1} << 22U;

std::vector<double> copy_of(std::span<const double> values) {
    return {values.begin(), values.end()};
}

double duration_of(const scene::AeroSolverSettings& settings, bool two_dimensional) {
    if (settings.lbm_duration > 0.0) {
        return settings.lbm_duration;
    }
    return two_dimensional ? aero::lbm::LbmSolverOptions{}.duration
                           : aero::lbm::LbmSolver3DOptions{}.duration;
}

double nose_radius(const scene::GeometryRef& ref) {
    if (ref.source != scene::GeometrySource::primitive) {
        return 0.0;
    }
    if (ref.shape == scene::AeroShape::sphere) {
        return 0.5 * ref.dimensions.diameter_m;
    }
    if (ref.shape == scene::AeroShape::capsule) {
        const geom::CapsuleShape shape =
            ref.capsule == "orion" ? geom::CapsuleShape::orion() : geom::CapsuleShape::apollo();
        return shape.nose_radius * ref.dimensions.diameter_m / shape.max_diameter;
    }
    return 0.0;
}

} // namespace

std::size_t lbm_steps_per_call(bool two_dimensional) noexcept {
    return two_dimensional ? kLbmStepsPerCall2D : kLbmStepsPerCall3D;
}

std::size_t lbm_steps_per_call_for(bool two_dimensional,
                                   std::optional<std::uint64_t> limit) noexcept {
    const std::size_t batch = lbm_steps_per_call(two_dimensional);
    const std::uint64_t steps = limit.value_or(0);
    if (steps == 0) {
        return batch;
    }
    for (std::size_t n = batch; n > 1; --n) {
        if (steps % n == 0) {
            return n;
        }
    }
    return 1;
}

std::unique_ptr<aero::IAeroSolver> make_solver(const SolverPlan& plan,
                                               const scene::AeroSolverSettings& settings,
                                               std::size_t steps_per_call) {
    const std::size_t per_call =
        steps_per_call > 0 ? steps_per_call : lbm_steps_per_call(plan.two_dimensional);
    switch (plan.kind) {
        case aero::SolverKind::panel:
            if (plan.two_dimensional) {
                return std::make_unique<aero::PanelSolver2D>();
            }
            return std::make_unique<aero::PanelSolver3D>();
        case aero::SolverKind::newtonian:
            if (plan.two_dimensional) {
                throw std::invalid_argument(
                    "Newtonian impact theory needs a 3D surface mesh, not a 2D section");
            }
            return std::make_unique<aero::NewtonianSolver>(
                aero::NewtonianOptions{plan.nose_radius_m, aero::sutton_graves_earth});
        case aero::SolverKind::lattice_boltzmann:
            if (plan.two_dimensional) {
                aero::lbm::LbmSolverOptions options;
                if (settings.lbm_cells_per_length > 0.0) {
                    options.resolution.cells_per_length = settings.lbm_cells_per_length;
                }
                options.duration = duration_of(settings, true);
                options.steps_per_call = per_call;
                return std::make_unique<aero::lbm::LbmAeroSolver2D>(options);
            } else {
                aero::lbm::LbmSolver3DOptions options;
                if (settings.lbm_cells_per_length > 0.0) {
                    options.resolution.cells_per_length = settings.lbm_cells_per_length;
                }
                options.duration = duration_of(settings, false);
                options.steps_per_call = per_call;
                options.max_cells = kMaxCpuCells3D;
                return std::make_unique<aero::lbm::LbmAeroSolver3D>(options);
            }
    }
    throw std::invalid_argument("unknown solver kind");
}

RunDiagnostics diagnostics(const aero::IAeroSolver& solver,
                           const scene::AeroSolverSettings& settings) {
    RunDiagnostics out;
    const auto planned = [&](const std::optional<aero::lbm::LatticeUnits>& units,
                             bool two_dimensional) -> std::uint64_t {
        if (!units) {
            return 0;
        }
        const double steps = std::ceil(duration_of(settings, two_dimensional) *
                                       units->cells_per_length() / units->lattice_velocity());
        return steps > 0.0 && steps < 9.0e15 ? static_cast<std::uint64_t>(steps) : 0;
    };
    if (const auto* lbm2 = dynamic_cast<const aero::lbm::LbmAeroSolver2D*>(&solver)) {
        const aero::lbm::CoefficientHistory& history = lbm2->history();
        out.history_step = copy_of(history.time());
        out.history_cd = copy_of(history.drag());
        out.history_cl = copy_of(history.lift());
        out.strouhal = lbm2->strouhal();
        out.planned_steps = planned(lbm2->units(), true);
        if (const aero::lbm::Lbm2DSolver* lattice = lbm2->lattice()) {
            out.lattice_steps = lattice->steps();
            out.lattice =
                std::to_string(lattice->config().nx) + " x " + std::to_string(lattice->config().ny);
        }
    } else if (const auto* lbm3 = dynamic_cast<const aero::lbm::LbmAeroSolver3D*>(&solver)) {
        const aero::lbm::CoefficientHistory& history = lbm3->history();
        out.history_step = copy_of(history.time());
        out.history_cd = copy_of(history.drag());
        out.history_cl = copy_of(history.lift());
        out.planned_steps = lbm3->total_steps();
        out.blockage = lbm3->blockage();
        if (const aero::lbm::Lbm3DSolver* lattice = lbm3->lattice()) {
            out.lattice_steps = lattice->steps();
            out.lattice = std::to_string(lattice->config().nx) + " x " +
                          std::to_string(lattice->config().ny) + " x " +
                          std::to_string(lattice->config().nz);
        }
    }
    return out;
}

std::uint64_t lattice_steps(const aero::IAeroSolver& solver) noexcept {
    if (const auto* lbm2 = dynamic_cast<const aero::lbm::LbmAeroSolver2D*>(&solver)) {
        const aero::lbm::Lbm2DSolver* lattice = lbm2->lattice();
        return lattice != nullptr ? lattice->steps() : 0;
    }
    if (const auto* lbm3 = dynamic_cast<const aero::lbm::LbmAeroSolver3D*>(&solver)) {
        const aero::lbm::Lbm3DSolver* lattice = lbm3->lattice();
        return lattice != nullptr ? lattice->steps() : 0;
    }
    return 0;
}

scene::AeroResultRecord record_result(const aero::AeroResult& result,
                                      const RunDiagnostics& diagnostics, aero::SolverKind solver,
                                      const FlowSummary& flow,
                                      const aero::ReferenceValues& reference) {
    scene::AeroResultRecord r;
    r.solver = solver;
    r.cd = result.cd;
    r.cl = result.cl;
    r.cy = result.cy;
    r.cm = result.cm;
    r.cd_pressure = result.cd_pressure;
    r.cd_induced = result.cd_induced;
    r.cd_friction = result.cd_friction;
    r.reynolds = flow.reynolds;
    r.mach = flow.mach;
    r.reference_area_m2 = reference.area;
    r.reference_length_m = reference.length;
    r.iterations = static_cast<std::int64_t>(result.iterations);
    r.converged = result.converged;
    r.strouhal = diagnostics.strouhal;
    for (const aero::ValidityWarning& warning : result.warnings) {
        r.warnings.push_back(warning.code);
    }
    for (const aero::ValidityWarning& warning : flow.warnings) {
        if (warning.code.starts_with("workspace.")) {
            r.warnings.push_back(warning.code);
        }
    }
    r.residuals = result.residuals;
    r.history_step = diagnostics.history_step;
    r.history_cd = diagnostics.history_cd;
    r.history_cl = diagnostics.history_cl;
    // The record keeps finite numbers only (JSON has no NaN or infinity).
    for (double* value :
         {&r.cd, &r.cl, &r.cy, &r.cm, &r.cd_pressure, &r.cd_induced, &r.cd_friction}) {
        if (!std::isfinite(*value)) {
            *value = 0.0;
            r.converged = false;
        }
    }
    for (std::vector<double>* series : {&r.residuals, &r.history_cd, &r.history_cl}) {
        for (double& value : *series) {
            if (!std::isfinite(value)) {
                value = 0.0;
            }
        }
    }
    return r;
}

FlowSummary AeroRunSpec::flow() const {
    const bool two_d = geometry && geometry->two_dimensional;
    const double length = geometry ? reference_values(*geometry, experiment.solver).length : 1.0;
    return summarize_flow(experiment.conditions, length, experiment.solver.solver, two_d);
}

SolverPlan AeroRunSpec::plan() const {
    SolverPlan plan;
    plan.kind = flow().solver;
    plan.two_dimensional = geometry && geometry->two_dimensional;
    plan.nose_radius_m = geometry ? nose_radius(geometry->ref) : 0.0;
    return plan;
}

core::Result<SolveOutcome, std::string> solve(const AeroRunSpec& spec,
                                              std::optional<std::uint64_t> max_lattice_steps) {
    if (!spec.geometry) {
        return core::failure(std::string("no geometry"));
    }
    SolveOutcome out;
    out.flow = spec.flow();
    if (!out.flow.runnable()) {
        return core::failure("flow conditions: " + out.flow.input_errors.front());
    }
    const SolverPlan plan = spec.plan();
    out.solver = plan.kind;
    out.reference = reference_values(*spec.geometry, spec.experiment.solver);
    try {
        std::unique_ptr<aero::IAeroSolver> solver =
            make_solver(plan, spec.experiment.solver,
                        lbm_steps_per_call_for(plan.two_dimensional, max_lattice_steps));
        solver->setup(to_aero_geometry(*spec.geometry, spec.experiment.solver), out.flow.flow);
        while (!out.finished) {
            out.finished = solver->step();
            if (max_lattice_steps && lattice_steps(*solver) >= *max_lattice_steps) {
                break;
            }
        }
        out.result = solver->result();
        out.diagnostics = diagnostics(*solver, spec.experiment.solver);
        out.solver_instance = std::move(solver);
    } catch (const std::exception& error) {
        return core::failure(std::string(aero::solver_name(plan.kind)) + ": " + error.what());
    }
    return out;
}

} // namespace orrery::sim
