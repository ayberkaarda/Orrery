#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "orrery/aero/lbm3d/d3q19.h"
#include "orrery/aero/lbm3d/lbm3d.h"

// Private to orrery_aero: the boundary handling shared by the CPU solver and
// the boundary-link search. The GLSL snippet lbm_d3q19_common.glsl
// implements the same rules (keep both in step).
namespace orrery::aero::lbm::detail3d {

enum class SourceKind : std::uint8_t {
    node,        // read population `direction` of node `cell`
    bounce_back, // solid neighbour: population opposite(i) of this node
    wall,        // no-slip domain wall: bounce-back plus the wall velocity term
    inlet,       // unknown at the inlet, reconstructed afterwards
    outlet,      // entering at the outlet: node `cell` (outlet face),
                 // population `direction`; the convective outlet blends it
                 // with the history of the receiving node
};

// The populations entering through the outlet (c_x = -1) and their slot in
// the outlet history.
inline constexpr std::array<std::size_t, 5> outlet_directions{2, 8, 10, 12, 14};
[[nodiscard]] constexpr std::size_t outlet_slot(std::size_t direction) noexcept {
    switch (direction) {
        case 2:
            return 0;
        case 8:
            return 1;
        case 10:
            return 2;
        case 12:
            return 3;
        default:
            return 4;
    }
}
// The populations the inlet reconstructs (c_x = +1).
inline constexpr std::array<std::size_t, 5> inlet_directions{1, 7, 9, 11, 13};

struct Source {
    SourceKind kind = SourceKind::node;
    std::size_t cell = 0;
    std::size_t direction = 0;
    double wall_ux = 0.0;
};

// Applies the rule of one transverse side (y or z) when the source
// coordinate `s` leaves [0, n): periodic wrap, specular reflection (the
// coordinate stays at the node's own, `direction` is mirrored), or a no-slip
// wall (returned as `wall`).
struct SideResult {
    bool wall = false;
    double wall_ux = 0.0;
};
[[nodiscard]] inline SideResult
apply_side(const Wall& low, const Wall& high, std::ptrdiff_t& s, std::ptrdiff_t n,
           std::ptrdiff_t own, std::size_t& direction,
           const std::array<std::size_t, d3q19::q>& mirror) noexcept {
    if (s >= 0 && s < n) {
        return {};
    }
    const Wall& wall = s < 0 ? low : high;
    switch (wall.kind) {
        case WallKind::periodic:
            s = (s + n) % n;
            break;
        case WallKind::slip:
            s = own;
            direction = mirror[direction];
            break;
        case WallKind::no_slip:
            return SideResult{true, wall.velocity_x};
    }
    return {};
}

// Where population i of node (x, y, z) comes from in the pull step: the y
// sides first, then the z sides (a population crossing an edge between two
// slip walls is reflected by both), then x.
[[nodiscard]] inline Source resolve_source(const Lbm3DConfig& config, const SolidMask3D& mask,
                                           std::size_t x, std::size_t y, std::size_t z,
                                           std::size_t i) noexcept {
    const auto nx = static_cast<std::ptrdiff_t>(config.nx);
    const auto ny = static_cast<std::ptrdiff_t>(config.ny);
    const auto nz = static_cast<std::ptrdiff_t>(config.nz);
    const std::size_t here = mask.index(x, y, z);
    std::ptrdiff_t sx = static_cast<std::ptrdiff_t>(x) - d3q19::cx[i];
    std::ptrdiff_t sy = static_cast<std::ptrdiff_t>(y) - d3q19::cy[i];
    std::ptrdiff_t sz = static_cast<std::ptrdiff_t>(z) - d3q19::cz[i];
    std::size_t direction = i;
    if (const SideResult side =
            apply_side(config.y_low, config.y_high, sy, ny, static_cast<std::ptrdiff_t>(y),
                       direction, d3q19::mirror_y);
        side.wall) {
        return Source{SourceKind::wall, here, d3q19::opposite[i], side.wall_ux};
    }
    if (const SideResult side =
            apply_side(config.z_low, config.z_high, sz, nz, static_cast<std::ptrdiff_t>(z),
                       direction, d3q19::mirror_z);
        side.wall) {
        return Source{SourceKind::wall, here, d3q19::opposite[i], side.wall_ux};
    }
    if (sx < 0 || sx >= nx) {
        if (config.x_boundary == XBoundary::periodic) {
            sx = (sx + nx) % nx;
        } else if (sx < 0) {
            return Source{SourceKind::inlet, here, i, 0.0};
        } else {
            const std::size_t face = mask.index(config.nx - 1, static_cast<std::size_t>(sy),
                                                static_cast<std::size_t>(sz));
            return Source{SourceKind::outlet, face, direction, 0.0};
        }
    }
    const std::size_t cell = mask.index(static_cast<std::size_t>(sx), static_cast<std::size_t>(sy),
                                        static_cast<std::size_t>(sz));
    if (mask.solid[cell] != 0) {
        return Source{SourceKind::bounce_back, here, d3q19::opposite[i], 0.0};
    }
    return Source{SourceKind::node, cell, direction, 0.0};
}

// Wall fraction q of the link from fluid node (x, y, z) towards its solid
// source x - c_i (SolidWall): 1/2 for the halfway wall; otherwise the zero of
// the linear interpolation of the mask distance between the two nodes,
// q = d_f / (d_f - d_s) in [0, 1] (without overflow for finite distances).
// Distances that place no wall on the link fall back to 1/2: a non-finite
// one, or inconsistent signs (d_f < 0 or d_s > 0, or both zero). So does
// q < 1/2 when the second node x + c_i is not a fluid node of the domain.
[[nodiscard]] inline double link_fraction(const Lbm3DConfig& config, const SolidMask3D& mask,
                                          std::size_t x, std::size_t y, std::size_t z,
                                          std::size_t i) noexcept {
    if (config.solid_wall != SolidWall::interpolated) {
        return 0.5;
    }
    const auto sx = static_cast<std::ptrdiff_t>(x) - d3q19::cx[i];
    const auto sy = static_cast<std::ptrdiff_t>(y) - d3q19::cy[i];
    const auto sz = static_cast<std::ptrdiff_t>(z) - d3q19::cz[i];
    const auto inside = [&](std::ptrdiff_t a, std::ptrdiff_t b, std::ptrdiff_t c) {
        return a >= 0 && b >= 0 && c >= 0 && a < static_cast<std::ptrdiff_t>(config.nx) &&
               b < static_cast<std::ptrdiff_t>(config.ny) &&
               c < static_cast<std::ptrdiff_t>(config.nz);
    };
    if (!inside(sx, sy, sz)) {
        return 0.5;
    }
    const double fluid = mask.distance[mask.index(x, y, z)];
    const double solid = mask.distance[mask.index(
        static_cast<std::size_t>(sx), static_cast<std::size_t>(sy), static_cast<std::size_t>(sz))];
    if (!std::isfinite(fluid) || !std::isfinite(solid) || fluid < 0.0 || solid > 0.0 ||
        (fluid == 0.0 && solid == 0.0)) {
        return 0.5;
    }
    // d_f - d_s > 0; where it overflows, the halves (both large) do not.
    const double span = fluid - solid;
    const double ratio =
        std::isfinite(span) ? fluid / span : 0.5 * fluid / (0.5 * fluid - 0.5 * solid);
    const double q = std::min(ratio, 1.0);
    if (q < 0.5) {
        const auto fx = static_cast<std::ptrdiff_t>(x) + d3q19::cx[i];
        const auto fy = static_cast<std::ptrdiff_t>(y) + d3q19::cy[i];
        const auto fz = static_cast<std::ptrdiff_t>(z) + d3q19::cz[i];
        if (!inside(fx, fy, fz) ||
            mask.at(static_cast<std::size_t>(fx), static_cast<std::size_t>(fy),
                    static_cast<std::size_t>(fz))) {
            return 0.5;
        }
    }
    return q;
}

// The population a fluid node receives from the wall of a link, from the
// post-collision populations `f` (layout of the solvers, `cells` nodes):
// population o = opposite(i) of the node `here` (halfway, q = 1/2) or the
// interpolation of SolidWall (`farther` = here + c_i, used for q < 1/2).
[[nodiscard]] inline double reflected(const std::vector<double>& f, std::size_t cells,
                                      std::size_t here, std::size_t farther, std::size_t i,
                                      double q) noexcept {
    const std::size_t o = d3q19::opposite[i];
    const double out = f[o * cells + here];
    if (q == 0.5) {
        return out;
    }
    if (q < 0.5) {
        return 2.0 * q * out + (1.0 - 2.0 * q) * f[o * cells + farther];
    }
    return (out + (2.0 * q - 1.0) * f[i * cells + here]) / (2.0 * q);
}

// Offset of the source node x - c_i in the linear index, for nodes whose
// neighbours are all inside the domain.
[[nodiscard]] inline std::array<std::ptrdiff_t, d3q19::q>
source_offsets(const Lbm3DConfig& config) noexcept {
    const auto nx = static_cast<std::ptrdiff_t>(config.nx);
    const auto plane = static_cast<std::ptrdiff_t>(config.nx * config.ny);
    std::array<std::ptrdiff_t, d3q19::q> offset{};
    for (std::size_t i = 0; i < d3q19::q; ++i) {
        offset[i] = -static_cast<std::ptrdiff_t>(d3q19::cx[i]) -
                    nx * static_cast<std::ptrdiff_t>(d3q19::cy[i]) -
                    plane * static_cast<std::ptrdiff_t>(d3q19::cz[i]);
    }
    return offset;
}

// Node classes: solid, bulk fluid (every source node inside the domain and
// fluid: plain streaming) and boundary fluid (resolve_source per link). The
// GPU kernels read the same codes.
enum NodeClass : std::uint8_t { node_solid = 0, node_bulk = 1, node_boundary = 2 };

[[nodiscard]] inline std::vector<std::uint8_t> classify_nodes(const Lbm3DConfig& config,
                                                              const SolidMask3D& mask) {
    const std::array<std::ptrdiff_t, d3q19::q> offset = source_offsets(config);
    std::vector<std::uint8_t> classes(mask.solid.size(), std::uint8_t{node_bulk});
    for (std::size_t z = 0; z < config.nz; ++z) {
        for (std::size_t y = 0; y < config.ny; ++y) {
            for (std::size_t x = 0; x < config.nx; ++x) {
                const std::size_t here = mask.index(x, y, z);
                if (mask.solid[here] != 0) {
                    classes[here] = node_solid;
                    continue;
                }
                if (x == 0 || y == 0 || z == 0 || x + 1 == config.nx || y + 1 == config.ny ||
                    z + 1 == config.nz) {
                    classes[here] = node_boundary;
                    continue;
                }
                for (std::size_t i = 1; i < d3q19::q; ++i) {
                    const auto source =
                        static_cast<std::size_t>(static_cast<std::ptrdiff_t>(here) + offset[i]);
                    if (mask.solid[source] != 0) {
                        classes[here] = node_boundary;
                        break;
                    }
                }
            }
        }
    }
    return classes;
}

// Empty when `velocity` can be imposed at the inlet of `config`
// (Lbm3DSolver::set_inlet_velocity), otherwise the reason.
[[nodiscard]] std::optional<std::string>
check_inlet_velocity(const Lbm3DConfig& config, const std::array<double, 3>& velocity);

// Relaxation rate of the absorbing layers at node (x, y, z) (Sponge).
[[nodiscard]] inline double sponge_sigma(const Lbm3DConfig& config, std::size_t x, std::size_t y,
                                         std::size_t z) noexcept {
    const Sponge& sponge = config.sponge;
    if (!(sponge.strength > 0.0) || config.x_boundary != XBoundary::inflow_outflow) {
        return 0.0;
    }
    double best = 0.0;
    const auto layer = [&best](std::size_t distance, std::size_t thickness) {
        if (distance < thickness) {
            const double r =
                static_cast<double>(thickness - distance) / static_cast<double>(thickness);
            best = std::max(best, r * r);
        }
    };
    layer(x, sponge.inlet);
    layer(config.nx - 1 - x, sponge.outlet);
    if (config.y_low.kind != WallKind::periodic) {
        layer(y, sponge.sides);
        layer(config.ny - 1 - y, sponge.sides);
    }
    if (config.z_low.kind != WallKind::periodic) {
        layer(z, sponge.sides);
        layer(config.nz - 1 - z, sponge.sides);
    }
    return sponge.strength * best;
}

// Wall fractions of the interpolated wall per fluid node next to a solid:
// slot[cell] indexes q (19 entries per slot, by direction; 1/2 where the
// direction has no link), kNoSlot for other nodes. Empty for the halfway
// wall.
inline constexpr std::uint32_t kNoSlot = 0xffffffffU;
struct WallTable {
    std::vector<std::uint32_t> slot;
    std::vector<double> q;
};

[[nodiscard]] inline WallTable wall_table(const Lbm3DConfig& config, const SolidMask3D& mask,
                                          const std::vector<BoundaryLink3D>& links) {
    WallTable table;
    if (config.solid_wall != SolidWall::interpolated) {
        return table;
    }
    table.slot.assign(mask.solid.size(), kNoSlot);
    std::uint32_t next = 0;
    for (const BoundaryLink3D& link : links) {
        std::uint32_t& slot = table.slot[link.cell];
        if (slot == kNoSlot) {
            slot = next++;
            table.q.insert(table.q.end(), d3q19::q, 0.5);
        }
        table.q[std::size_t{slot} * d3q19::q + link.direction] = link.q;
    }
    return table;
}

// Velocity inlet on the west face x = 0. The density follows from the known
// populations (c_x = 0 and c_x = -1) and the prescribed normal velocity
// (Zou and He 1997),
//   rho = (sum_{c_x = 0} f + 2 sum_{c_x = -1} f) / (1 - u_x),
// the five unknown populations (c_x = +1) from bounce-back of their
// non-equilibrium part, f_i = f_opp(i) + 6 w_i rho c_i.u, which gives the
// prescribed normal momentum; the node is then regularized (Latt, Chopard,
// Malaspinas, Deville and Michler 2008):
//   f_i = f_eq,i(rho, u) + w_i / (2 c_s^4) (c_i c_i - c_s^2 I) : Pi_neq,
// which sets density and the full velocity exactly (the tangential
// corrections of the D3Q19 Zou-He scheme, Hecht and Harting 2010, are not
// needed) and keeps the momentum flux of the set.
inline void velocity_inlet_west(d3q19::Populations& f, double ux, double uy, double uz) noexcept {
    double known = 0.0;
    for (std::size_t i = 0; i < d3q19::q; ++i) {
        if (d3q19::cx[i] == 0) {
            known += f[i];
        } else if (d3q19::cx[i] < 0) {
            known += 2.0 * f[i];
        }
    }
    const double rho = known / (1.0 - ux);
    for (const std::size_t i : inlet_directions) {
        const double cu = static_cast<double>(d3q19::cx[i]) * ux +
                          static_cast<double>(d3q19::cy[i]) * uy +
                          static_cast<double>(d3q19::cz[i]) * uz;
        f[i] = f[d3q19::opposite[i]] + 6.0 * d3q19::w[i] * rho * cu;
    }
    d3q19::Populations feq{};
    // Pi_neq: xx, yy, zz, xy, xz, yz.
    std::array<double, 6> pi{};
    for (std::size_t i = 0; i < d3q19::q; ++i) {
        feq[i] = d3q19::equilibrium(i, rho, ux, uy, uz);
        const double neq = f[i] - feq[i];
        const auto ex = static_cast<double>(d3q19::cx[i]);
        const auto ey = static_cast<double>(d3q19::cy[i]);
        const auto ez = static_cast<double>(d3q19::cz[i]);
        pi[0] += ex * ex * neq;
        pi[1] += ey * ey * neq;
        pi[2] += ez * ez * neq;
        pi[3] += ex * ey * neq;
        pi[4] += ex * ez * neq;
        pi[5] += ey * ez * neq;
    }
    for (std::size_t i = 0; i < d3q19::q; ++i) {
        const auto ex = static_cast<double>(d3q19::cx[i]);
        const auto ey = static_cast<double>(d3q19::cy[i]);
        const auto ez = static_cast<double>(d3q19::cz[i]);
        f[i] = feq[i] + 4.5 * d3q19::w[i] *
                            ((ex * ex - d3q19::cs2) * pi[0] + (ey * ey - d3q19::cs2) * pi[1] +
                             (ez * ez - d3q19::cs2) * pi[2] +
                             2.0 * (ex * ey * pi[3] + ex * ez * pi[4] + ey * ez * pi[5]));
    }
}

} // namespace orrery::aero::lbm::detail3d
