#pragma once

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <vector>

#include "../lbm/lbm_test_support.h"
#include "orrery/aero/lbm3d/lbm3d.h"

// Shared helpers of the D3Q19 tests: error norms and the configurations used
// by both the CPU and the GPU tests (take() and require_ok() come from the
// 2D support header).
namespace orrery::aero::lbm::test {

// Relative L2 difference of the velocity field (all three components).
inline double velocity_l2(const MacroFields3D& a, const MacroFields3D& b) {
    std::vector<double> va = a.velocity_x;
    va.insert(va.end(), a.velocity_y.begin(), a.velocity_y.end());
    va.insert(va.end(), a.velocity_z.begin(), a.velocity_z.end());
    std::vector<double> vb = b.velocity_x;
    vb.insert(vb.end(), b.velocity_y.begin(), b.velocity_y.end());
    vb.insert(vb.end(), b.velocity_z.begin(), b.velocity_z.end());
    std::vector<std::uint8_t> skip = b.solid;
    skip.insert(skip.end(), b.solid.begin(), b.solid.end());
    skip.insert(skip.end(), b.solid.begin(), b.solid.end());
    return l2_relative(va, vb, skip);
}

// The larger of a running worst error and a new error; a non-finite error
// (a NaN or infinite field value) makes it infinite, where std::max would
// keep the old value for a NaN and let an all-NaN field pass.
inline double worse(double worst, double error) noexcept {
    if (!std::isfinite(error)) {
        return std::numeric_limits<double>::infinity();
    }
    return std::max(worst, error);
}

// Largest deviation of a field from the uniform state (rho, u): the density
// relative to rho, each velocity component relative to `speed_scale`, over
// every node; infinite when any value is not finite.
inline double uniform_state_error(const MacroFields3D& fields, double rho,
                                  const std::array<double, 3>& u, double speed_scale) {
    double worst = 0.0;
    for (std::size_t c = 0; c < fields.density.size(); ++c) {
        worst = worse(worst, std::abs(fields.density[c] - rho) / rho);
        worst = worse(worst, std::abs(fields.velocity_x[c] - u[0]) / speed_scale);
        worst = worse(worst, std::abs(fields.velocity_y[c] - u[1]) / speed_scale);
        worst = worse(worst, std::abs(fields.velocity_z[c] - u[2]) / speed_scale);
    }
    return worst;
}

inline Wall wall(WallKind kind, double velocity = 0.0) {
    return Wall{kind, velocity};
}

// Periodic box (x, and y and z unless walls are set later).
inline Lbm3DConfig periodic_box(std::size_t nx, std::size_t ny, std::size_t nz, double tau) {
    Lbm3DConfig config;
    config.nx = nx;
    config.ny = ny;
    config.nz = nz;
    config.tau = tau;
    config.x_boundary = XBoundary::periodic;
    config.y_low = wall(WallKind::periodic);
    config.y_high = wall(WallKind::periodic);
    config.z_low = wall(WallKind::periodic);
    config.z_high = wall(WallKind::periodic);
    return config;
}

// Plane Poiseuille flow: periodic in x and z, no-slip walls at y = -1/2 and
// ny - 1/2, driven by a body force g along x. Steady profile
//   u(y) = g / (2 nu) (y + 1/2) (ny - 1/2 - y).
inline Lbm3DConfig poiseuille3d_config(std::size_t nx, std::size_t ny, std::size_t nz, double tau,
                                       double g) {
    Lbm3DConfig config = periodic_box(nx, ny, nz, tau);
    config.y_low = wall(WallKind::no_slip);
    config.y_high = wall(WallKind::no_slip);
    config.body_force_x = g;
    return config;
}

inline double poiseuille3d_velocity(const Lbm3DConfig& config, double y) {
    const double nu = viscosity_of_tau(config.tau);
    const auto h = static_cast<double>(config.ny);
    return config.body_force_x / (2.0 * nu) * (y + 0.5) * (h - 0.5 - y);
}

// Rectangular duct: periodic in x, no-slip walls on all four sides (halfway
// outside the domain), driven by a body force g along x. With half-widths
// a = ny / 2 and b = nz / 2 and coordinates from the duct axis, the steady
// profile is the series (White, Viscous Fluid Flow, 3rd ed., eq. 3-48)
//   u = 16 a^2 g / (nu pi^3) sum_{n odd} (-1)^((n-1)/2) / n^3
//       [1 - cosh(n pi z / (2a)) / cosh(n pi b / (2a))] cos(n pi y / (2a)).
inline Lbm3DConfig duct_config(std::size_t nx, std::size_t ny, std::size_t nz, double tau,
                               double g) {
    Lbm3DConfig config = poiseuille3d_config(nx, ny, nz, tau, g);
    config.z_low = wall(WallKind::no_slip);
    config.z_high = wall(WallKind::no_slip);
    return config;
}

inline double duct_velocity(const Lbm3DConfig& config, std::size_t yi, std::size_t zi) {
    const double nu = viscosity_of_tau(config.tau);
    const double a = 0.5 * static_cast<double>(config.ny);
    const double b = 0.5 * static_cast<double>(config.nz);
    const double y = static_cast<double>(yi) - 0.5 * static_cast<double>(config.ny - 1);
    const double z = static_cast<double>(zi) - 0.5 * static_cast<double>(config.nz - 1);
    const double pi = std::numbers::pi;
    double sum = 0.0;
    for (int n = 1; n < 200; n += 2) {
        const double k = n * pi / (2.0 * a);
        const double sign = ((n - 1) / 2) % 2 == 0 ? 1.0 : -1.0;
        // cosh(k z) / cosh(k b) evaluated without overflow for large k.
        const double ratio = std::exp(k * (std::abs(z) - b)) *
                             (1.0 + std::exp(-2.0 * k * std::abs(z))) /
                             (1.0 + std::exp(-2.0 * k * b));
        sum += sign / (static_cast<double>(n) * n * n) * (1.0 - ratio) * std::cos(k * y);
    }
    return 16.0 * a * a * config.body_force_x / (nu * pi * pi * pi) * sum;
}

// Plane Couette flow: periodic in x and y, resting wall at z = -1/2, wall at
// z = nz - 1/2 moving at u_w along x. Steady profile u(z) = u_w (z + 1/2) / nz.
inline Lbm3DConfig couette3d_config(std::size_t nx, std::size_t ny, std::size_t nz, double tau,
                                    double u_wall) {
    Lbm3DConfig config = periodic_box(nx, ny, nz, tau);
    config.z_low = wall(WallKind::no_slip);
    config.z_high = wall(WallKind::no_slip, u_wall);
    return config;
}

inline InitialState3D uniform_state3d(double rho, double ux) {
    return [rho, ux](std::size_t, std::size_t, std::size_t) {
        return d3q19::Moments{rho, ux, 0.0, 0.0};
    };
}

// Sphere of diameter d in a channel with a velocity inlet, a convective
// outlet and free-slip sides; centre `upstream` diameters behind the inlet.
struct SphereSetup {
    Lbm3DConfig config;
    SolidMask3D mask;
    double diameter = 0.0;
};

inline SphereSetup sphere_channel(std::size_t nx, std::size_t ny, std::size_t nz, double diameter,
                                  double upstream, double u, double reynolds) {
    SphereSetup setup;
    setup.diameter = diameter;
    setup.config.nx = nx;
    setup.config.ny = ny;
    setup.config.nz = nz;
    setup.config.tau = tau_of_viscosity(u * diameter / reynolds);
    setup.config.x_boundary = XBoundary::inflow_outflow;
    setup.config.y_low = wall(WallKind::slip);
    setup.config.y_high = wall(WallKind::slip);
    setup.config.z_low = wall(WallKind::slip);
    setup.config.z_high = wall(WallKind::slip);
    setup.config.inlet_ux = u;
    // A quarter node off the channel axis, so the discrete sphere is not
    // symmetric about it.
    const std::array<double, 3> centre{upstream * diameter,
                                       0.5 * static_cast<double>(ny - 1) + 0.25,
                                       0.5 * static_cast<double>(nz - 1) + 0.25};
    setup.mask = sphere_mask(nx, ny, nz, centre, 0.5 * diameter);
    return setup;
}

} // namespace orrery::aero::lbm::test
