#pragma once

#include <array>
#include <cstddef>

// The D2Q9 lattice (Qian, d'Humieres and Lallemand 1992) in lattice units
// (dx = dt = 1). Direction order: 0 rest, 1 E, 2 N, 3 W, 4 S, 5 NE, 6 NW,
// 7 SW, 8 SE. The GLSL kernels (src/render/shaders/lbm/lbm_d2q9_common.glsl)
// use the same order; tests check both against each other.
namespace orrery::aero::lbm::d2q9 {

inline constexpr std::size_t q = 9;
inline constexpr std::array<int, q> cx{0, 1, 0, -1, 0, 1, -1, -1, 1};
inline constexpr std::array<int, q> cy{0, 0, 1, 0, -1, 1, 1, -1, -1};
inline constexpr std::array<double, q> w{4.0 / 9.0,  1.0 / 9.0,  1.0 / 9.0,  1.0 / 9.0, 1.0 / 9.0,
                                         1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0};
// Direction with the opposite velocity (bounce-back).
inline constexpr std::array<std::size_t, q> opposite{0, 3, 4, 1, 2, 7, 8, 5, 6};
// Direction with the y velocity reversed (specular reflection on a wall
// normal to y).
inline constexpr std::array<std::size_t, q> mirror_y{0, 1, 4, 3, 2, 8, 7, 6, 5};
// Lattice speed of sound squared.
inline constexpr double cs2 = 1.0 / 3.0;

using Populations = std::array<double, q>;

struct Moments {
    double rho = 0.0;
    double ux = 0.0; // momentum density / rho, without forcing correction
    double uy = 0.0;
};

// Second-order Maxwell-Boltzmann equilibrium
//   f_i^eq = w_i rho (1 + 3 c.u + 9/2 (c.u)^2 - 3/2 u.u).
[[nodiscard]] constexpr double equilibrium(std::size_t i, double rho, double ux,
                                           double uy) noexcept {
    const double cu = static_cast<double>(cx[i]) * ux + static_cast<double>(cy[i]) * uy;
    const double uu = ux * ux + uy * uy;
    return w[i] * rho * (1.0 + 3.0 * cu + 4.5 * cu * cu - 1.5 * uu);
}

[[nodiscard]] constexpr Populations equilibrium(double rho, double ux, double uy) noexcept {
    Populations f{};
    for (std::size_t i = 0; i < q; ++i) {
        f[i] = equilibrium(i, rho, ux, uy);
    }
    return f;
}

// Density and velocity (momentum / density) of a set of populations.
[[nodiscard]] constexpr Moments moments(const Populations& f) noexcept {
    double rho = 0.0;
    double jx = 0.0;
    double jy = 0.0;
    for (std::size_t i = 0; i < q; ++i) {
        rho += f[i];
        jx += static_cast<double>(cx[i]) * f[i];
        jy += static_cast<double>(cy[i]) * f[i];
    }
    return Moments{rho, jx / rho, jy / rho};
}

} // namespace orrery::aero::lbm::d2q9
