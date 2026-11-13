#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace orrery::atmo {

enum class AtmoErrorCode : std::uint8_t {
    // A parameter is NaN, infinite or outside its allowed range.
    invalid_parameter,
    // A Cd(Mach) table is empty, unsorted or holds a non-finite value.
    invalid_table,
    // No aerodynamic solver is available for the requested regime.
    solver_unavailable,
    // An aerodynamic solver threw or produced a non-finite coefficient.
    solver_failed,
    // The orbit integrator rejected a step (see core::IntegrationError).
    integration_failed,
};

[[nodiscard]] std::string_view to_string(AtmoErrorCode code) noexcept;

// Error of the atmosphere and entry API: a code for programs and a message
// naming the offending value for people.
struct AtmoError {
    AtmoErrorCode code = AtmoErrorCode::invalid_parameter;
    std::string message;
};

} // namespace orrery::atmo
