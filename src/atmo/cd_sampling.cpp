#include <cmath>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "orrery/aero/aero_result.h"
#include "orrery/atmo/aero_table.h"

namespace orrery::atmo {

namespace {

AtmoError error(AtmoErrorCode code, std::string message) {
    return AtmoError{code, std::move(message)};
}

// Empty when the request can be sampled.
std::string request_problem(const CdSamplingRequest& request) {
    const std::vector<double>& mach = request.mach_numbers;
    if (mach.empty()) {
        return "at least one Mach number is needed";
    }
    for (std::size_t i = 0; i < mach.size(); ++i) {
        if (!(mach[i] > 0.0) || !std::isfinite(mach[i])) {
            return "Mach numbers must be positive and finite";
        }
        if (i > 0 && !(mach[i] > mach[i - 1])) {
            return "Mach numbers must be strictly increasing";
        }
    }
    const aero::ReferenceValues& ref = request.geometry.reference;
    if (!(ref.area > 0.0) || !std::isfinite(ref.area) || !(ref.length > 0.0) ||
        !std::isfinite(ref.length)) {
        return "reference area and length must be positive and finite";
    }
    if (request.max_steps == 0) {
        return "max_steps must be at least 1";
    }
    return {};
}

// Steps `solver` until it reports a final result (or max_steps); returns the
// number of steps taken.
std::size_t solve(aero::IAeroSolver& solver, const CdSamplingRequest& request,
                  const aero::FlowConditions& flow) {
    solver.setup(request.geometry, flow);
    std::size_t steps = 0;
    bool done = false;
    while (!done && steps < request.max_steps) {
        done = solver.step();
        ++steps;
    }
    return steps;
}

// Samples one Mach number into `point`; returns the error, if any.
std::optional<AtmoError> sample_point(const CdSamplingRequest& request,
                                      const AeroSolverFactory& factory, double mach,
                                      CdSamplingPoint& point) {
    aero::FlowConditions flow = request.freestream;
    flow.speed = mach * flow.speed_of_sound();
    const aero::RegimeChoice choice =
        aero::select_solver(flow, request.geometry.reference.length, request.solver_override);
    const std::string label = "Mach " + std::to_string(mach) + ": ";
    std::unique_ptr<aero::IAeroSolver> solver = factory ? factory(choice.solver) : nullptr;
    if (!solver) {
        return error(AtmoErrorCode::solver_unavailable,
                     label + "no " + std::string(aero::solver_name(choice.solver)) +
                         " solver available");
    }
    point.mach = mach;
    point.solver = choice.solver;
    point.within_envelope = choice.within_envelope;
    try {
        point.steps = solve(*solver, request, flow);
    } catch (const std::exception& e) {
        return error(AtmoErrorCode::solver_failed, label + e.what());
    }
    const aero::AeroResult& result = solver->result();
    point.cd = result.cd;
    point.converged = result.converged;
    point.warnings.reserve(result.warnings.size());
    for (const aero::ValidityWarning& w : result.warnings) {
        point.warnings.push_back(w.code);
    }
    if (!(point.cd > 0.0) || !std::isfinite(point.cd)) {
        return error(AtmoErrorCode::solver_failed,
                     label + "the solver returned Cd " + std::to_string(point.cd));
    }
    return std::nullopt;
}

} // namespace

core::Result<CdSamplingResult, AtmoError> sample_cd_table(const CdSamplingRequest& request,
                                                          const AeroSolverFactory& factory) {
    std::string problem = request_problem(request);
    if (!problem.empty()) {
        return core::failure(error(AtmoErrorCode::invalid_parameter, std::move(problem)));
    }
    CdSamplingResult out;
    std::vector<CdSample> samples;
    samples.reserve(request.mach_numbers.size());
    out.points.reserve(request.mach_numbers.size());
    for (const double mach : request.mach_numbers) {
        CdSamplingPoint point;
        if (std::optional<AtmoError> failed = sample_point(request, factory, mach, point)) {
            return core::failure(std::move(*failed));
        }
        samples.push_back(CdSample{mach, point.cd});
        out.points.push_back(std::move(point));
    }
    auto table = AeroCoefficientTable::create(std::move(samples),
                                              core::si::Area(request.geometry.reference.area));
    if (!table) {
        return core::failure(table.error());
    }
    out.table = std::move(table).value();
    return out;
}

} // namespace orrery::atmo
