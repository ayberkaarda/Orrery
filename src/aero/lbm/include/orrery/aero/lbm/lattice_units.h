#pragma once

#include <string>

#include "orrery/core/result.h"

// Conversion between SI and lattice units for the LBM solvers (ADR-001: the
// aero domain works in SI, the LBM internally in lattice units dx = dt = 1,
// rho_0 = 1). Fixing the resolution (cells per characteristic length) and the
// lattice velocity of the free stream fixes everything else:
//
//   dx = L / N,  dt = u_lat dx / U,  nu_lat = nu dt / dx^2,
//   tau = nu_lat / c_s^2 + 1/2 = 3 nu_lat + 1/2,
//
// so the Reynolds number U L / nu = u_lat N / nu_lat is the same in both
// systems. APPROXIMATION A3: the scheme is weakly compressible with a density
// error of order Ma^2; the conversion refuses a lattice Mach number
// u_lat / c_s of 0.1 or more.
namespace orrery::aero::lbm {

// Largest accepted lattice Mach number (exclusive).
inline constexpr double max_lattice_mach = 0.1;

// Lattice Mach number u / c_s of a lattice velocity, c_s = 1 / sqrt(3).
[[nodiscard]] double lattice_mach(double lattice_velocity) noexcept;

// Characteristic flow in SI units.
struct SiFlow {
    double length = 1.0;                 // m, characteristic length (cylinder diameter, chord)
    double velocity = 1.0;               // m/s, free-stream speed
    double kinematic_viscosity = 1.5e-5; // m^2/s
    double density = 1.225;              // kg/m^3, reference density (rho_lat = 1)
};

struct LatticeResolution {
    double cells_per_length = 20.0; // N: lattice cells across `length`
    double lattice_velocity = 0.05; // u_lat: free stream in lattice units
};

class LatticeUnits {
public:
    // Fails for non-finite or non-positive inputs, for a lattice Mach number
    // of 0.1 or more, and for tau <= 1/2 (zero or negative viscosity).
    [[nodiscard]] static core::Result<LatticeUnits, std::string>
    create(const SiFlow& flow, const LatticeResolution& resolution);

    [[nodiscard]] double dx() const noexcept { return dx_; }                 // m
    [[nodiscard]] double dt() const noexcept { return dt_; }                 // s
    [[nodiscard]] double reference_density() const noexcept { return rho_; } // kg/m^3
    [[nodiscard]] double lattice_velocity() const noexcept { return u_lat_; }
    [[nodiscard]] double lattice_viscosity() const noexcept { return nu_lat_; }
    [[nodiscard]] double tau() const noexcept { return 3.0 * nu_lat_ + 0.5; }
    [[nodiscard]] double mach() const noexcept { return lattice_mach(u_lat_); }
    [[nodiscard]] double reynolds() const noexcept { return reynolds_; }
    [[nodiscard]] double cells_per_length() const noexcept { return cells_; }

    [[nodiscard]] double length_to_lattice(double metres) const noexcept { return metres / dx_; }
    [[nodiscard]] double length_to_si(double cells) const noexcept { return cells * dx_; }
    [[nodiscard]] double time_to_si(double steps) const noexcept { return steps * dt_; }
    [[nodiscard]] double time_to_lattice(double seconds) const noexcept { return seconds / dt_; }
    [[nodiscard]] double velocity_to_lattice(double metres_per_second) const noexcept {
        return metres_per_second * dt_ / dx_;
    }
    [[nodiscard]] double velocity_to_si(double lattice) const noexcept {
        return lattice * dx_ / dt_;
    }
    [[nodiscard]] double viscosity_to_si(double lattice) const noexcept {
        return lattice * dx_ * dx_ / dt_;
    }
    // Gauge pressure p - p_ref from the lattice density: c_s^2 (rho - 1) in
    // lattice units, times rho_ref (dx / dt)^2.
    [[nodiscard]] double pressure_to_si(double lattice_density) const noexcept;
    // Vorticity (1/s) from lattice vorticity (1/step).
    [[nodiscard]] double vorticity_to_si(double lattice) const noexcept { return lattice / dt_; }
    // Force per unit span (N/m) of a 2D lattice force (momentum per step of a
    // lattice of unit depth): one lattice unit is rho_ref dx^4 / dt^2 newtons
    // on a slab of depth dx, so rho_ref dx^3 / dt^2 per metre of span. Keeps
    // the force coefficient F / (rho U^2 L / 2) unchanged.
    [[nodiscard]] double force_per_span_to_si(double lattice) const noexcept {
        return lattice * rho_ * dx_ * dx_ * dx_ / (dt_ * dt_);
    }

private:
    LatticeUnits(double dx, double dt, double rho, double u_lat, double nu_lat, double reynolds,
                 double cells) noexcept
        : dx_(dx), dt_(dt), rho_(rho), u_lat_(u_lat), nu_lat_(nu_lat), reynolds_(reynolds),
          cells_(cells) {}
    double dx_ = 1.0;
    double dt_ = 1.0;
    double rho_ = 1.0;
    double u_lat_ = 0.0;
    double nu_lat_ = 0.0;
    double reynolds_ = 0.0;
    double cells_ = 0.0;
};

} // namespace orrery::aero::lbm
