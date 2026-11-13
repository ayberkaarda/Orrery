#include "orrery/aero/regime.h"

#include <cmath>
#include <limits>
#include <numbers>
#include <sstream>
#include <utility>

namespace orrery::aero {

namespace {

constexpr double infinity = std::numeric_limits<double>::infinity();
constexpr double degree = std::numbers::pi / 180.0;

// Upper Mach number of incompressible models (density change below about 5
// percent) and lower Mach number of hypersonic impact theory.
constexpr double incompressible_mach = 0.3;
constexpr double hypersonic_mach = 5.0;
// Reynolds number separating thin attached boundary layers (panel method)
// from viscous, separated flow resolved by the lattice Boltzmann solver.
constexpr double panel_min_reynolds = 1e5;
// Attached flow on thin wings: stall begins at about 10 to 15 degrees.
constexpr double panel_max_angle = 10.0 * degree;

std::string format_value(double v) {
    std::ostringstream out;
    out.precision(4);
    out << v;
    return out.str();
}

void add_range_warning(std::vector<ValidityWarning>& out, std::string_view solver_code,
                       std::string_view what, std::string_view label, double value, double lo,
                       double hi) {
    if (value >= lo && value <= hi) {
        return;
    }
    std::string code(solver_code);
    code += '.';
    code += what;
    std::string message(solver_code);
    message += ": ";
    message += label;
    message += ' ';
    message += format_value(value);
    message += " is outside the validity envelope [";
    message += format_value(lo);
    message += ", ";
    message += format_value(hi);
    message += "]; the result is computed but not reliable";
    out.push_back(ValidityWarning{std::move(code), std::move(message)});
}

} // namespace

std::string_view solver_name(SolverKind kind) noexcept {
    switch (kind) {
        case SolverKind::lattice_boltzmann:
            return "lbm";
        case SolverKind::panel:
            return "panel";
        case SolverKind::newtonian:
            return "newtonian";
    }
    return "unknown";
}

ValidityEnvelope validity_envelope(SolverKind kind) noexcept {
    switch (kind) {
        case SolverKind::lattice_boltzmann:
            // APPROXIMATION (A3): weakly compressible; the physical Mach number
            // must stay in the incompressible range. Re above 1e5 needs grids far
            // beyond 256^3 even with the Smagorinsky closure (A2).
            return {0.0, incompressible_mach, 0.0, panel_min_reynolds, std::numbers::pi};
        case SolverKind::panel:
            // APPROXIMATION (A5): inviscid, incompressible, attached flow.
            return {0.0, incompressible_mach, panel_min_reynolds, infinity, panel_max_angle};
        case SolverKind::newtonian:
            // APPROXIMATION (A7): impact theory needs a strong, thin shock layer.
            return {hypersonic_mach, infinity, 0.0, infinity, std::numbers::pi};
    }
    return {};
}

std::vector<ValidityWarning> check_envelope(const ValidityEnvelope& envelope,
                                            std::string_view solver_code,
                                            const FlowConditions& conditions,
                                            double reference_length) {
    std::vector<ValidityWarning> out;
    add_range_warning(out, solver_code, "mach", "Mach number", conditions.mach(), envelope.min_mach,
                      envelope.max_mach);
    add_range_warning(out, solver_code, "reynolds", "Reynolds number",
                      conditions.reynolds(reference_length), envelope.min_reynolds,
                      envelope.max_reynolds);
    add_range_warning(out, solver_code, "angle_of_attack", "angle of attack (deg)",
                      conditions.angle_of_attack / degree, -envelope.max_angle / degree,
                      envelope.max_angle / degree);
    add_range_warning(out, solver_code, "sideslip", "sideslip (deg)", conditions.sideslip / degree,
                      -envelope.max_angle / degree, envelope.max_angle / degree);
    return out;
}

std::vector<ValidityWarning> check_validity(SolverKind kind, const FlowConditions& conditions,
                                            double reference_length) {
    return check_envelope(validity_envelope(kind), solver_name(kind), conditions, reference_length);
}

RegimeChoice select_solver(const FlowConditions& conditions, double reference_length,
                           std::optional<SolverKind> user_override) {
    const double mach = conditions.mach();
    const double re = conditions.reynolds(reference_length);
    RegimeChoice choice;
    if (mach >= hypersonic_mach) {
        choice.recommended = SolverKind::newtonian;
        choice.rationale = "hypersonic (Mach >= 5): modified Newtonian impact theory";
    } else if (mach < incompressible_mach) {
        if (re >= panel_min_reynolds) {
            choice.recommended = SolverKind::panel;
            choice.rationale = "incompressible, Re >= 1e5: thin attached boundary layer, "
                               "potential-flow panel method";
        } else {
            choice.recommended = SolverKind::lattice_boltzmann;
            choice.rationale = "incompressible, Re < 1e5: viscous flow resolved by lattice "
                               "Boltzmann";
        }
    } else if (mach < 1.0) {
        choice.recommended = SolverKind::panel;
        choice.rationale = "compressible subsonic (0.3 <= Mach < 1): no solver is valid; the "
                           "incompressible panel method is the closest";
    } else {
        choice.recommended = SolverKind::newtonian;
        choice.rationale = "supersonic (1 <= Mach < 5): no solver is valid; Newtonian impact "
                           "theory is the closest";
    }
    choice.solver = user_override.value_or(choice.recommended);
    choice.overridden = user_override.has_value() && *user_override != choice.recommended;
    choice.warnings = check_validity(choice.solver, conditions, reference_length);
    choice.within_envelope = choice.warnings.empty();
    return choice;
}

} // namespace orrery::aero
