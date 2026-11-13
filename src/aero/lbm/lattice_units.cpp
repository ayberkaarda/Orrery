#include "orrery/aero/lbm/lattice_units.h"

#include <cmath>
#include <sstream>

#include "orrery/aero/lbm/d2q9.h"

namespace orrery::aero::lbm {

namespace {

bool positive_finite(double value) noexcept {
    return std::isfinite(value) && value > 0.0;
}

} // namespace

double lattice_mach(double lattice_velocity) noexcept {
    return std::abs(lattice_velocity) / std::sqrt(d2q9::cs2);
}

core::Result<LatticeUnits, std::string> LatticeUnits::create(const SiFlow& flow,
                                                             const LatticeResolution& resolution) {
    if (!positive_finite(flow.length) || !positive_finite(flow.velocity) ||
        !positive_finite(flow.kinematic_viscosity) || !positive_finite(flow.density)) {
        return core::failure(std::string(
            "lattice units: length, velocity, viscosity and density must be positive and finite"));
    }
    if (!positive_finite(resolution.cells_per_length) ||
        !positive_finite(resolution.lattice_velocity)) {
        return core::failure(std::string(
            "lattice units: cells per length and lattice velocity must be positive and finite"));
    }
    const double mach = lattice_mach(resolution.lattice_velocity);
    if (!(mach < max_lattice_mach)) {
        std::ostringstream message;
        message << "lattice units: lattice Mach number " << mach << " (u_lat "
                << resolution.lattice_velocity << ") is not below " << max_lattice_mach
                << "; lower the lattice velocity (weak compressibility, APPROXIMATION A3)";
        return core::failure(message.str());
    }
    const double dx = flow.length / resolution.cells_per_length;
    const double dt = resolution.lattice_velocity * dx / flow.velocity;
    const double nu_lat = flow.kinematic_viscosity * dt / (dx * dx);
    if (!positive_finite(dx) || !positive_finite(dt) || !positive_finite(nu_lat)) {
        return core::failure(std::string("lattice units: spacing, time step or viscosity "
                                         "underflows or overflows double precision"));
    }
    const double reynolds = flow.velocity * flow.length / flow.kinematic_viscosity;
    return LatticeUnits(dx, dt, flow.density, resolution.lattice_velocity, nu_lat, reynolds,
                        resolution.cells_per_length);
}

double LatticeUnits::pressure_to_si(double lattice_density) const noexcept {
    const double velocity_scale = dx_ / dt_;
    return d2q9::cs2 * (lattice_density - 1.0) * rho_ * velocity_scale * velocity_scale;
}

} // namespace orrery::aero::lbm
