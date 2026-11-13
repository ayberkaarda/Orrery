#pragma once

#include <array>
#include <cstddef>

// The D3Q19 lattice (Qian, d'Humieres and Lallemand 1992) in lattice units
// (dx = dt = 1). Direction order: 0 rest; 1 +x, 2 -x, 3 +y, 4 -y, 5 +z,
// 6 -z; then the twelve face diagonals in opposite pairs: 7 (+x+y),
// 8 (-x-y), 9 (+x-y), 10 (-x+y), 11 (+x+z), 12 (-x-z), 13 (+x-z),
// 14 (-x+z), 15 (+y+z), 16 (-y-z), 17 (+y-z), 18 (-y+z). The GLSL kernels
// (src/render/shaders/lbm/lbm_d3q19_common.glsl) use the same order; tests
// check both against each other.
namespace orrery::aero::lbm::d3q19 {

inline constexpr std::size_t q = 19;
inline constexpr std::array<int, q> cx{0,  1, -1, 0, 0,  0, 0, 1, -1, 1,
                                       -1, 1, -1, 1, -1, 0, 0, 0, 0};
inline constexpr std::array<int, q> cy{0, 0, 0, 1, -1, 0, 0,  1, -1, -1,
                                       1, 0, 0, 0, 0,  1, -1, 1, -1};
inline constexpr std::array<int, q> cz{0, 0, 0,  0,  0, 1, -1, 0,  0, 0,
                                       0, 1, -1, -1, 1, 1, -1, -1, 1};
inline constexpr std::array<double, q> w{1.0 / 3.0,  1.0 / 18.0, 1.0 / 18.0, 1.0 / 18.0, 1.0 / 18.0,
                                         1.0 / 18.0, 1.0 / 18.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0,
                                         1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0,
                                         1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0};
// Direction with the opposite velocity (bounce-back).
inline constexpr std::array<std::size_t, q> opposite{0, 2,  1,  4,  3,  6,  5,  8,  7, 10,
                                                     9, 12, 11, 14, 13, 16, 15, 18, 17};
// Direction with the y (z) velocity reversed: specular reflection on a wall
// normal to y (z).
inline constexpr std::array<std::size_t, q> mirror_y{0, 1,  2,  4,  3,  5,  6,  9,  10, 7,
                                                     8, 11, 12, 13, 14, 18, 17, 16, 15};
inline constexpr std::array<std::size_t, q> mirror_z{0,  1,  2,  3,  4,  6,  5,  7,  8, 9,
                                                     10, 13, 14, 11, 12, 17, 18, 15, 16};
// Lattice speed of sound squared.
inline constexpr double cs2 = 1.0 / 3.0;

using Populations = std::array<double, q>;

struct Moments {
    double rho = 0.0;
    double ux = 0.0; // momentum density / rho, without forcing correction
    double uy = 0.0;
    double uz = 0.0;
};

// Second-order Maxwell-Boltzmann equilibrium
//   f_i^eq = w_i rho (1 + 3 c.u + 9/2 (c.u)^2 - 3/2 u.u).
[[nodiscard]] constexpr double equilibrium(std::size_t i, double rho, double ux, double uy,
                                           double uz) noexcept {
    const double cu = static_cast<double>(cx[i]) * ux + static_cast<double>(cy[i]) * uy +
                      static_cast<double>(cz[i]) * uz;
    const double uu = ux * ux + uy * uy + uz * uz;
    return w[i] * rho * (1.0 + 3.0 * cu + 4.5 * cu * cu - 1.5 * uu);
}

[[nodiscard]] constexpr Populations equilibrium(const Moments& m) noexcept {
    Populations f{};
    for (std::size_t i = 0; i < q; ++i) {
        f[i] = equilibrium(i, m.rho, m.ux, m.uy, m.uz);
    }
    return f;
}

// Density and velocity (momentum / density) of a set of populations.
[[nodiscard]] constexpr Moments moments(const Populations& f) noexcept {
    double rho = 0.0;
    double jx = 0.0;
    double jy = 0.0;
    double jz = 0.0;
    for (std::size_t i = 0; i < q; ++i) {
        rho += f[i];
        jx += static_cast<double>(cx[i]) * f[i];
        jy += static_cast<double>(cy[i]) * f[i];
        jz += static_cast<double>(cz[i]) * f[i];
    }
    return Moments{rho, jx / rho, jy / rho, jz / rho};
}

} // namespace orrery::aero::lbm::d3q19
