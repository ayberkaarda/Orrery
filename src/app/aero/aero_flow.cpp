#include "orrery/sim/aero_flow.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <utility>

namespace orrery::sim {

namespace {

void check_positive(std::vector<std::string>& errors, double value, const char* what) {
    if (!(value > 0.0) || !std::isfinite(value)) {
        errors.push_back(std::string(what) + " must be positive");
    }
}

// The solver that runs on the body when `wanted` does not. Only Newtonian
// theory is ever unavailable (2D sections); its substitute is the panel
// method, which carries the warning of its own envelope (the 2D lattice
// Boltzmann solver is no more compressible).
aero::SolverKind available_substitute(aero::SolverKind wanted, bool two_dimensional) noexcept {
    return solver_available(wanted, two_dimensional) ? wanted : aero::SolverKind::panel;
}

} // namespace

bool solver_available(aero::SolverKind kind, bool two_dimensional) noexcept {
    return !(two_dimensional && kind == aero::SolverKind::newtonian);
}

std::string_view solver_display_name(aero::SolverKind kind) noexcept {
    switch (kind) {
        case aero::SolverKind::panel:
            return "Panel method";
        case aero::SolverKind::newtonian:
            return "Newtonian impact";
        case aero::SolverKind::lattice_boltzmann:
            return "Lattice Boltzmann";
    }
    return "Unknown";
}

FlowSummary summarize_flow(const scene::AeroConditions& conditions, double reference_length,
                           std::optional<aero::SolverKind> user_override, bool two_dimensional) {
    FlowSummary out;
    out.flow = conditions.flow();
    out.reference_length = reference_length;
    check_positive(out.input_errors, conditions.speed_m_s, "Speed");
    check_positive(out.input_errors, conditions.density_kg_m3, "Density");
    check_positive(out.input_errors, conditions.viscosity_pa_s, "Viscosity");
    check_positive(out.input_errors, conditions.temperature_k, "Temperature");
    check_positive(out.input_errors, reference_length, "Reference length");
    if (!(conditions.gamma > 1.0) || !std::isfinite(conditions.gamma)) {
        out.input_errors.emplace_back("Ratio of specific heats must be above 1");
    }
    if (!std::isfinite(conditions.angle_of_attack_deg) || !std::isfinite(conditions.sideslip_deg)) {
        out.input_errors.emplace_back("Angles must be finite");
    }
    if (!out.runnable()) {
        return out;
    }
    out.speed_of_sound = out.flow.speed_of_sound();
    out.mach = out.flow.mach();
    out.reynolds = out.flow.reynolds(reference_length);
    out.dynamic_pressure = out.flow.dynamic_pressure();
    out.kinematic_viscosity = conditions.viscosity_pa_s / conditions.density_kg_m3;
    out.regime = aero::select_solver(out.flow, reference_length, user_override);
    out.solver = available_substitute(out.regime.solver, two_dimensional);
    if (out.solver != out.regime.solver) {
        out.warnings.push_back({"workspace.solver_unavailable",
                                std::string(solver_display_name(out.regime.solver)) +
                                    " needs a 3D surface; this 2D section runs with the " +
                                    std::string(solver_display_name(out.solver)) + " instead"});
        auto substitute = aero::check_validity(out.solver, out.flow, reference_length);
        out.warnings.insert(out.warnings.end(), substitute.begin(), substitute.end());
    } else {
        out.warnings = out.regime.warnings;
    }
    if (two_dimensional && conditions.sideslip_deg != 0.0) {
        out.warnings.push_back(
            {"workspace.sideslip_2d", "a 2D section ignores the sideslip angle"});
    }
    return out;
}

scene::AeroConditions sea_level_air(const scene::AeroConditions& conditions) {
    scene::AeroConditions out = conditions;
    const scene::AeroConditions standard;
    out.density_kg_m3 = standard.density_kg_m3;
    out.viscosity_pa_s = standard.viscosity_pa_s;
    out.temperature_k = standard.temperature_k;
    out.gamma = standard.gamma;
    out.gas_constant_j_kg_k = standard.gas_constant_j_kg_k;
    return out;
}

std::string format_significant(double value, int digits) {
    if (!std::isfinite(value)) {
        return "-";
    }
    std::array<char, 64> buffer{};
    const double magnitude = std::abs(value);
    const bool scientific = magnitude != 0.0 && (magnitude < 1.0e-3 || magnitude >= 1.0e6);
    const int written =
        scientific
            ? std::snprintf(buffer.data(), buffer.size(), "%.*e", std::max(digits - 1, 0), value)
            : std::snprintf(buffer.data(), buffer.size(), "%.*g", std::max(digits, 1), value);
    if (written <= 0) {
        return "-";
    }
    return {buffer.data(), static_cast<std::size_t>(written)};
}

} // namespace orrery::sim
