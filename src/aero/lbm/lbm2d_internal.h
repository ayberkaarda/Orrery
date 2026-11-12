#pragma once

#include <cstddef>
#include <cstdint>

#include "orrery/aero/lbm/d2q9.h"
#include "orrery/aero/lbm/lbm2d.h"

// Private to orrery_aero: the boundary handling shared by the CPU solver and
// the boundary-link search. The GLSL kernel lbm_d2q9_common.glsl implements
// the same rules (keep both in step).
namespace orrery::aero::lbm::detail {

enum class SourceKind : std::uint8_t {
    node,        // read population `direction` of node `cell`
    bounce_back, // solid neighbour: population opposite(i) of this node
    wall,        // no-slip domain wall: bounce-back plus the wall velocity term
    inlet,       // unknown at the Zou-He inlet, reconstructed afterwards
    outlet,      // entering at the outlet: node `cell` (the outlet column),
                 // population `direction`; convective outlets blend it with
                 // the history of the node
};

// Index of an outlet direction (3, 6, 7: c_x = -1) in the outlet history.
[[nodiscard]] constexpr std::size_t outlet_slot(std::size_t direction) noexcept {
    return direction == 3 ? 0 : (direction == 6 ? 1 : 2);
}

struct Source {
    SourceKind kind = SourceKind::node;
    std::size_t cell = 0;
    std::size_t direction = 0;
    double wall_ux = 0.0;
};

// Where population i of node (x, y) comes from in the pull step.
[[nodiscard]] inline Source resolve_source(const Lbm2DConfig& config, const SolidMask2D& mask,
                                           std::size_t x, std::size_t y, std::size_t i) noexcept {
    const auto nx = static_cast<std::ptrdiff_t>(config.nx);
    const auto ny = static_cast<std::ptrdiff_t>(config.ny);
    const std::size_t here = x + config.nx * y;
    std::ptrdiff_t sx = static_cast<std::ptrdiff_t>(x) - d2q9::cx[i];
    std::ptrdiff_t sy = static_cast<std::ptrdiff_t>(y) - d2q9::cy[i];
    std::size_t direction = i;
    if (sy < 0 || sy >= ny) {
        const Wall& wall = sy < 0 ? config.bottom : config.top;
        switch (wall.kind) {
            case WallKind::periodic:
                sy = (sy + ny) % ny;
                break;
            case WallKind::slip:
                sy = static_cast<std::ptrdiff_t>(y);
                direction = d2q9::mirror_y[i];
                break;
            case WallKind::no_slip:
                return Source{SourceKind::wall, here, d2q9::opposite[i], wall.velocity_x};
        }
    }
    if (sx < 0 || sx >= nx) {
        if (config.x_boundary == XBoundary::periodic) {
            sx = (sx + nx) % nx;
        } else if (sx < 0) {
            return Source{SourceKind::inlet, here, i, 0.0};
        } else {
            const std::size_t column = config.nx - 1 + config.nx * static_cast<std::size_t>(sy);
            return Source{SourceKind::outlet, column, direction, 0.0};
        }
    }
    const std::size_t cell =
        static_cast<std::size_t>(sx) + config.nx * static_cast<std::size_t>(sy);
    if (mask.solid[cell] != 0) {
        return Source{SourceKind::bounce_back, here, d2q9::opposite[i], 0.0};
    }
    return Source{SourceKind::node, cell, direction, 0.0};
}

// Zou-He velocity boundary on a west face: reconstructs f1, f5 and f8 from
// the known populations and the prescribed velocity (Zou and He 1997), then
// regularizes the node: every population is replaced by its equilibrium
// plus the second-order non-equilibrium part of the Zou-He set,
//   f_i = f_eq,i + w_i / (2 c_s^4) (c_i c_i - c_s^2 I) : Pi_neq,
// which keeps density, velocity and momentum flux but drops the higher-order
// non-equilibrium noise that makes the plain Zou-He node unstable for tau
// close to 1/2 (Latt, Chopard, Malaspinas, Deville and Michler 2008).
inline void zou_he_west(d2q9::Populations& f, double ux, double uy) noexcept {
    const double rho = (f[0] + f[2] + f[4] + 2.0 * (f[3] + f[6] + f[7])) / (1.0 - ux);
    f[1] = f[3] + (2.0 / 3.0) * rho * ux;
    f[5] = f[7] - 0.5 * (f[2] - f[4]) + (1.0 / 6.0) * rho * ux + 0.5 * rho * uy;
    f[8] = f[6] + 0.5 * (f[2] - f[4]) + (1.0 / 6.0) * rho * ux - 0.5 * rho * uy;

    d2q9::Populations feq{};
    double pxx = 0.0;
    double pyy = 0.0;
    double pxy = 0.0;
    for (std::size_t i = 0; i < d2q9::q; ++i) {
        feq[i] = d2q9::equilibrium(i, rho, ux, uy);
        const double neq = f[i] - feq[i];
        const auto ex = static_cast<double>(d2q9::cx[i]);
        const auto ey = static_cast<double>(d2q9::cy[i]);
        pxx += ex * ex * neq;
        pyy += ey * ey * neq;
        pxy += ex * ey * neq;
    }
    for (std::size_t i = 0; i < d2q9::q; ++i) {
        const auto ex = static_cast<double>(d2q9::cx[i]);
        const auto ey = static_cast<double>(d2q9::cy[i]);
        f[i] = feq[i] + 4.5 * d2q9::w[i] *
                            ((ex * ex - d2q9::cs2) * pxx + (ey * ey - d2q9::cs2) * pyy +
                             2.0 * ex * ey * pxy);
    }
}

} // namespace orrery::aero::lbm::detail
