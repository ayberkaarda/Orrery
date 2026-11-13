#include "orrery/atmo/aero_table.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>
#include <utility>

#include "orrery/aero/newtonian.h"
#include "orrery/aero/panel_method.h"

namespace orrery::atmo {

namespace {

AtmoError table_error(std::string message) {
    return AtmoError{AtmoErrorCode::invalid_table, std::move(message)};
}

bool positive_finite(double v) noexcept {
    return v > 0.0 && std::isfinite(v);
}

// Empty when the samples form a valid table.
std::string table_problem(const std::vector<CdSample>& samples) {
    if (samples.empty()) {
        return "the table needs at least one sample";
    }
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const CdSample& s = samples[i];
        const std::string where = "sample " + std::to_string(i) + ": ";
        if (!std::isfinite(s.mach) || s.mach < 0.0) {
            return where + "Mach must be finite and non-negative";
        }
        if (!positive_finite(s.cd)) {
            return where + "Cd must be positive and finite";
        }
        if (i > 0 && !(s.mach > samples[i - 1].mach)) {
            return where + "Mach numbers must be strictly increasing";
        }
    }
    return {};
}

} // namespace

AeroCoefficientTable::AeroCoefficientTable() : samples_{CdSample{0.0, 1.0}}, reference_area_(1.0) {}

AeroCoefficientTable::AeroCoefficientTable(std::vector<CdSample> samples,
                                           core::si::Area reference_area) noexcept
    : samples_(std::move(samples)), reference_area_(reference_area) {}

core::Result<AeroCoefficientTable, AtmoError>
AeroCoefficientTable::create(std::vector<CdSample> samples, core::si::Area reference_area) {
    std::string problem = table_problem(samples);
    if (!problem.empty()) {
        return core::failure(table_error(std::move(problem)));
    }
    if (!positive_finite(reference_area.value())) {
        return core::failure(table_error("reference area must be positive and finite"));
    }
    return AeroCoefficientTable(std::move(samples), reference_area);
}

core::Result<AeroCoefficientTable, AtmoError>
AeroCoefficientTable::constant(double cd, core::si::Area reference_area) {
    return create(std::vector<CdSample>{CdSample{0.0, cd}}, reference_area);
}

CdLookup AeroCoefficientTable::lookup(double mach) const noexcept {
    const CdSample& first = samples_.front();
    const CdSample& last = samples_.back();
    if (!(mach > first.mach)) {
        return CdLookup{first.cd, mach != first.mach};
    }
    if (mach >= last.mach) {
        return CdLookup{last.cd, mach != last.mach};
    }
    const auto upper =
        std::upper_bound(samples_.begin(), samples_.end(), mach,
                         [](double value, const CdSample& sample) { return value < sample.mach; });
    const CdSample& hi = *upper;
    const CdSample& lo = *(upper - 1);
    const double f = (mach - lo.mach) / (hi.mach - lo.mach);
    return CdLookup{lo.cd + f * (hi.cd - lo.cd), false};
}

AeroSolverFactory default_aero_solver_factory(double nose_radius) {
    return [nose_radius](aero::SolverKind kind) -> std::unique_ptr<aero::IAeroSolver> {
        switch (kind) {
            case aero::SolverKind::newtonian: {
                aero::NewtonianOptions options;
                options.nose_radius = nose_radius;
                return std::make_unique<aero::NewtonianSolver>(options);
            }
            case aero::SolverKind::panel:
                return std::make_unique<aero::PanelSolver3D>();
            case aero::SolverKind::lattice_boltzmann:
                return nullptr;
        }
        return nullptr;
    };
}

} // namespace orrery::atmo
