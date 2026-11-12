#pragma once

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "orrery/aero/lbm/lbm2d.h"

// Shared helpers of the LBM tests: result unwrapping, error norms and the
// configurations used by both the CPU and the GPU tests.
namespace orrery::aero::lbm::test {

// The value of a successful result; fails the test with the error otherwise.
template <typename T> T take(core::Result<T, std::string> result) {
    if (!result) {
        FAIL(result.error());
    }
    return std::move(result).value();
}

inline void require_ok(const Status& status) {
    if (!status) {
        FAIL(status.error());
    }
}

// ||a - b||_2 / ||b||_2 over the entries where `use` is set (all if empty).
inline double l2_relative(const std::vector<double>& a, const std::vector<double>& b,
                          const std::vector<std::uint8_t>& skip = {}) {
    double diff = 0.0;
    double norm = 0.0;
    for (std::size_t k = 0; k < a.size() && k < b.size(); ++k) {
        if (!skip.empty() && skip[k] != 0) {
            continue;
        }
        diff += (a[k] - b[k]) * (a[k] - b[k]);
        norm += b[k] * b[k];
    }
    return norm > 0.0 ? std::sqrt(diff / norm) : std::sqrt(diff);
}

// Relative L2 difference of the velocity field (both components together).
inline double velocity_l2(const MacroFields2D& a, const MacroFields2D& b) {
    std::vector<double> va = a.velocity_x;
    va.insert(va.end(), a.velocity_y.begin(), a.velocity_y.end());
    std::vector<double> vb = b.velocity_x;
    vb.insert(vb.end(), b.velocity_y.begin(), b.velocity_y.end());
    std::vector<std::uint8_t> skip = b.solid;
    skip.insert(skip.end(), b.solid.begin(), b.solid.end());
    return l2_relative(va, vb, skip);
}

// Relative L2 difference of the density deviation rho - 1, normalised by the
// largest deviation of the reference (rho itself is ~1 everywhere).
inline double density_deviation_error(const MacroFields2D& a, const MacroFields2D& b) {
    double diff = 0.0;
    double peak = 0.0;
    std::size_t count = 0;
    for (std::size_t k = 0; k < b.density.size(); ++k) {
        if (b.solid[k] != 0) {
            continue;
        }
        diff += (a.density[k] - b.density[k]) * (a.density[k] - b.density[k]);
        peak = std::max(peak, std::abs(b.density[k] - 1.0));
        ++count;
    }
    if (count == 0 || peak == 0.0) {
        return std::sqrt(diff);
    }
    return std::sqrt(diff / static_cast<double>(count)) / peak;
}

// Plane Poiseuille flow: periodic in x, no-slip walls at y = -1/2 and
// ny - 1/2, driven by a body force g along x. Steady profile
//   u(y) = g / (2 nu) (y + 1/2) (ny - 1/2 - y).
inline Lbm2DConfig poiseuille_config(std::size_t nx, std::size_t ny, double tau, double g) {
    Lbm2DConfig config;
    config.nx = nx;
    config.ny = ny;
    config.tau = tau;
    config.x_boundary = XBoundary::periodic;
    config.bottom = Wall{WallKind::no_slip, 0.0};
    config.top = Wall{WallKind::no_slip, 0.0};
    config.body_force_x = g;
    return config;
}

inline double poiseuille_velocity(const Lbm2DConfig& config, double y) {
    const double nu = viscosity_of_tau(config.tau);
    const double h = static_cast<double>(config.ny);
    return config.body_force_x / (2.0 * nu) * (y + 0.5) * (h - 0.5 - y);
}

// Plane Couette flow: periodic in x, resting bottom wall, top wall moving at
// u_w. Steady profile u(y) = u_w (y + 1/2) / ny.
inline Lbm2DConfig couette_config(std::size_t nx, std::size_t ny, double tau, double wall) {
    Lbm2DConfig config;
    config.nx = nx;
    config.ny = ny;
    config.tau = tau;
    config.x_boundary = XBoundary::periodic;
    config.bottom = Wall{WallKind::no_slip, 0.0};
    config.top = Wall{WallKind::no_slip, wall};
    return config;
}

// Channel with a Zou-He inlet, an extrapolation outlet and free-slip sides
// around a circular cylinder of diameter d centred at (cx, cy).
struct CylinderSetup {
    Lbm2DConfig config;
    SolidMask2D mask;
    double diameter = 0.0;
};

inline CylinderSetup cylinder_setup(std::size_t nx, std::size_t ny, double diameter, double cx,
                                    double cy, double u, double reynolds) {
    CylinderSetup setup;
    setup.diameter = diameter;
    setup.config.nx = nx;
    setup.config.ny = ny;
    setup.config.tau = tau_of_viscosity(u * diameter / reynolds);
    setup.config.x_boundary = XBoundary::inflow_outflow;
    setup.config.bottom = Wall{WallKind::slip, 0.0};
    setup.config.top = Wall{WallKind::slip, 0.0};
    setup.config.inlet_ux = u;
    setup.mask = circle_mask(nx, ny, cx, cy, 0.5 * diameter);
    return setup;
}

} // namespace orrery::aero::lbm::test
