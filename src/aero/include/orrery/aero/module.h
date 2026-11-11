#pragma once

#include "orrery/core/module.h"

// Aerodynamic solvers: lattice Boltzmann, panel method, Newtonian.
namespace orrery::aero {

[[nodiscard]] core::ModuleInfo module_info() noexcept;

} // namespace orrery::aero
