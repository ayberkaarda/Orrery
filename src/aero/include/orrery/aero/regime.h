#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "orrery/aero/aero_result.h"
#include "orrery/aero/flow_conditions.h"

namespace orrery::aero {

// The three aerodynamic solvers (ADR-009).
enum class SolverKind { lattice_boltzmann, panel, newtonian };

[[nodiscard]] std::string_view solver_name(SolverKind kind) noexcept;

// Range of free-stream conditions in which a solver's model holds. Bounds are
// inclusive; an unbounded side is +infinity (or 0 for the lower bounds).
struct ValidityEnvelope {
    double min_mach = 0.0;
    double max_mach = 0.0;
    // Reynolds number based on the reference length.
    double min_reynolds = 0.0;
    double max_reynolds = 0.0;
    // Largest |angle of attack| and |sideslip|, radians.
    double max_angle = 0.0;
};

// Documented envelopes (docs/aero.md):
//   panel method:      Mach <= 0.3, Re >= 1e5, |alpha|, |beta| <= 10 degrees
//   Newtonian:         Mach >= 5
//   lattice Boltzmann: Mach <= 0.3, Re <= 1e5
[[nodiscard]] ValidityEnvelope validity_envelope(SolverKind kind) noexcept;

// Warnings for every bound of `envelope` that `conditions` violate, coded
// "<solver_code>.mach", "<solver_code>.reynolds", "<solver_code>.angle_of_attack"
// and "<solver_code>.sideslip". Every solver runs this on its conditions and
// appends the output to its result.
[[nodiscard]] std::vector<ValidityWarning> check_envelope(const ValidityEnvelope& envelope,
                                                          std::string_view solver_code,
                                                          const FlowConditions& conditions,
                                                          double reference_length);

// check_envelope with the documented envelope and solver_name(kind).
[[nodiscard]] std::vector<ValidityWarning>
check_validity(SolverKind kind, const FlowConditions& conditions, double reference_length);

struct RegimeChoice {
    // Solver that will run: the override if one was given, else the
    // recommendation.
    SolverKind solver = SolverKind::panel;
    SolverKind recommended = SolverKind::panel;
    bool overridden = false;
    // True if `solver` is inside its envelope for these conditions.
    bool within_envelope = false;
    // Envelope warnings for `solver` (never blocking).
    std::vector<ValidityWarning> warnings;
    std::string rationale;
};

// Recommends a solver from the Mach and Reynolds numbers:
//   Mach >= 5                 -> Newtonian
//   Mach < 0.3, Re >= 1e5     -> panel method (thin attached boundary layer)
//   Mach < 0.3, Re < 1e5      -> lattice Boltzmann (viscous, separated flow)
//   0.3 <= Mach < 5           -> no solver is valid: panel below Mach 1,
//                                Newtonian above, both with a warning.
// `user_override` replaces the recommendation; the envelope warnings of the
// solver actually chosen are returned either way and nothing is blocked.
[[nodiscard]] RegimeChoice select_solver(const FlowConditions& conditions, double reference_length,
                                         std::optional<SolverKind> user_override = std::nullopt);

} // namespace orrery::aero
