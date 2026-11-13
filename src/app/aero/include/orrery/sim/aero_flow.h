#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "orrery/aero/aero_result.h"
#include "orrery/aero/flow_conditions.h"
#include "orrery/aero/regime.h"
#include "orrery/scene/aero_experiment.h"

// Flow conditions of the Aero workspace (docs/aero-workspace.md, "Flow
// conditions"): what the Flow Conditions panel derives from its inputs and
// which solver will run. Nothing here blocks a run: conditions outside a
// solver's envelope only add warnings (ADR-009).
namespace orrery::sim {

// Which solvers can run on a body: the 2D section solvers (panel, lattice
// Boltzmann) or the 3D ones (panel, Newtonian, lattice Boltzmann). Newtonian
// impact theory needs a surface mesh and has no 2D form.
[[nodiscard]] bool solver_available(aero::SolverKind kind, bool two_dimensional) noexcept;

// "Panel method", "Newtonian impact", "Lattice Boltzmann".
[[nodiscard]] std::string_view solver_display_name(aero::SolverKind kind) noexcept;

struct FlowSummary {
    aero::FlowConditions flow;
    double reference_length = 1.0; // m, for the Reynolds number
    double reynolds = 0.0;
    double mach = 0.0;
    double speed_of_sound = 0.0;    // m/s
    double dynamic_pressure = 0.0;  // Pa
    double kinematic_viscosity = 0.0; // m^2/s
    // aero::select_solver for these conditions with the user's override.
    aero::RegimeChoice regime;
    // The solver that will run: the regime's choice, or, when that solver is
    // not available for the body's dimension, the nearest available one (a
    // note says so).
    aero::SolverKind solver = aero::SolverKind::panel;
    // Envelope warnings of `solver` plus notes of the selection.
    std::vector<aero::ValidityWarning> warnings;
    // Input problems no solver can accept (non-positive density, ...); the
    // panel shows them next to the field and Run stays disabled until they
    // are fixed.
    std::vector<std::string> input_errors;

    [[nodiscard]] bool runnable() const noexcept { return input_errors.empty(); }
};

[[nodiscard]] FlowSummary summarize_flow(const scene::AeroConditions& conditions,
                                         double reference_length,
                                         std::optional<aero::SolverKind> user_override,
                                         bool two_dimensional);

// Standard sea-level air (ISA): 1.225 kg/m^3, 1.789e-5 Pa s, 288.15 K, keeping
// the speed and angles of `conditions`.
[[nodiscard]] scene::AeroConditions sea_level_air(const scene::AeroConditions& conditions);

// Formats a number with `digits` significant digits, switching to scientific
// notation outside [1e-3, 1e6) ("1.23e+06").
[[nodiscard]] std::string format_significant(double value, int digits);

} // namespace orrery::sim
