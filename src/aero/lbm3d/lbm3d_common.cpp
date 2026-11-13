#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>

#include "lbm3d_internal.h"
#include "orrery/aero/lbm/lattice_units.h"
#include "orrery/aero/lbm3d/lbm3d.h"

namespace orrery::aero::lbm {

namespace {

bool finite(double value) noexcept {
    return std::isfinite(value);
}

// The checks below return the reason as a string, empty when the value is
// fine (plain strings keep the many early returns cheap to analyse).
std::string check_mach(const char* what, double speed) {
    if (!(lattice_mach(speed) < max_lattice_mach)) {
        std::ostringstream message;
        message << "lbm3d: " << what << " " << speed << " has lattice Mach number "
                << lattice_mach(speed) << ", not below " << max_lattice_mach
                << " (APPROXIMATION A3)";
        return message.str();
    }
    return {};
}

std::optional<std::string> reason(std::string problem) {
    if (problem.empty()) {
        return std::nullopt;
    }
    return problem;
}

// A byte count that remembers an overflow on the way.
struct Count {
    std::size_t value = 0;
    bool valid = true;
};

// a * b and a + b without wrapping.
Count times(Count a, std::size_t b) noexcept {
    if (!a.valid || (a.value != 0 && b > std::numeric_limits<std::size_t>::max() / a.value)) {
        return Count{0, false};
    }
    return Count{a.value * b, true};
}

Count plus(Count a, Count b) noexcept {
    if (!a.valid || !b.valid || b.value > std::numeric_limits<std::size_t>::max() - a.value) {
        return Count{0, false};
    }
    return Count{a.value + b.value, true};
}

std::optional<std::size_t> result_of(Count c) noexcept {
    if (!c.valid) {
        return std::nullopt;
    }
    return c.value;
}

Count cells_of(const Lbm3DConfig& config) noexcept {
    const std::optional<std::size_t> cells = lattice_cells_3d(config.nx, config.ny, config.nz);
    return cells ? Count{*cells, true} : Count{0, false};
}

constexpr std::size_t kFloat32Bytes = 4;

std::string problem_of(const Lbm3DConfig& config, const SolidMask3D& mask) {
    if (config.nx < 3 || config.ny < 3 || config.nz < 3) {
        return "lbm3d: the lattice needs at least 3 nodes along each axis";
    }
    const Count cells = cells_of(config);
    if (!cells.valid) {
        return "lbm3d: more than geom::max_grid_cells nodes";
    }
    if (mask.nx != config.nx || mask.ny != config.ny || mask.nz != config.nz ||
        mask.solid.size() != cells.value) {
        return "lbm3d: the solid mask does not match the lattice size";
    }
    if (!finite(config.tau) || !(config.tau > 0.5)) {
        return "lbm3d: tau must be finite and above 1/2 (positive viscosity)";
    }
    for (const double v :
         {config.inlet_ux, config.inlet_uy, config.inlet_uz, config.body_force_x,
          config.body_force_y, config.body_force_z, config.y_low.velocity_x,
          config.y_high.velocity_x, config.z_low.velocity_x, config.z_high.velocity_x}) {
        if (!finite(v)) {
            return "lbm3d: velocities and body force must be finite";
        }
    }
    if (config.smagorinsky.enabled &&
        (!finite(config.smagorinsky.constant) || config.smagorinsky.constant < 0.0)) {
        return "lbm3d: the Smagorinsky constant must be finite and non-negative";
    }
    if ((config.y_low.kind == WallKind::periodic) != (config.y_high.kind == WallKind::periodic) ||
        (config.z_low.kind == WallKind::periodic) != (config.z_high.kind == WallKind::periodic)) {
        return "lbm3d: an axis is periodic on both sides or on neither";
    }
    struct Side {
        const char* name;
        const Wall* wall;
    };
    for (const Side side :
         {Side{"y = 0 wall speed", &config.y_low}, Side{"y = ny - 1 wall speed", &config.y_high},
          Side{"z = 0 wall speed", &config.z_low}, Side{"z = nz - 1 wall speed", &config.z_high}}) {
        if (std::string bad = check_mach(side.name, side.wall->velocity_x); !bad.empty()) {
            return bad;
        }
    }
    const Sponge& sponge = config.sponge;
    if (!finite(sponge.strength) || sponge.strength < 0.0 || sponge.strength > 1.0) {
        return "lbm3d: the absorbing-layer strength must be in [0, 1]";
    }
    if (sponge.strength > 0.0 &&
        (config.x_boundary != XBoundary::inflow_outflow || sponge.inlet > config.nx ||
         sponge.outlet > config.nx || sponge.sides > std::min(config.ny, config.nz))) {
        return "lbm3d: absorbing layers need the inflow/outflow x boundary and must fit in the "
               "lattice";
    }
    if (config.solid_wall == SolidWall::interpolated) {
        if (mask.distance.size() != mask.solid.size()) {
            return "lbm3d: the interpolated wall needs the mask's signed distance";
        }
        for (const double d : mask.distance) {
            if (std::isnan(d)) {
                return "lbm3d: the mask's signed distance must not be NaN";
            }
        }
    }
    if (config.x_boundary == XBoundary::inflow_outflow) {
        const double speed =
            std::sqrt(config.inlet_ux * config.inlet_ux + config.inlet_uy * config.inlet_uy +
                      config.inlet_uz * config.inlet_uz);
        if (std::string bad = check_mach("inlet speed", speed); !bad.empty()) {
            return bad;
        }
        for (std::size_t z = 0; z < config.nz; ++z) {
            for (std::size_t y = 0; y < config.ny; ++y) {
                if (mask.at(0, y, z) || mask.at(config.nx - 1, y, z)) {
                    return "lbm3d: the inlet and outlet faces must be fluid";
                }
            }
        }
        if (config.outlet == Outlet::convective && !(config.inlet_ux > 0.0)) {
            return "lbm3d: the convective outlet needs a positive inlet speed";
        }
    }
    return {};
}

} // namespace

std::optional<std::string> validate(const Lbm3DConfig& config, const SolidMask3D& mask) {
    return reason(problem_of(config, mask));
}

std::optional<std::string> detail3d::check_inlet_velocity(const Lbm3DConfig& config,
                                                          const std::array<double, 3>& velocity) {
    if (config.x_boundary != XBoundary::inflow_outflow) {
        return std::string("lbm3d: a periodic lattice has no inlet");
    }
    if (!finite(velocity[0]) || !finite(velocity[1]) || !finite(velocity[2])) {
        return std::string("lbm3d: the inlet velocity must be finite");
    }
    return reason(
        check_mach("inlet speed", std::sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1] +
                                            velocity[2] * velocity[2])));
}

std::vector<BoundaryLink3D> boundary_links(const Lbm3DConfig& config, const SolidMask3D& mask) {
    std::vector<BoundaryLink3D> links;
    for (std::size_t z = 0; z < config.nz; ++z) {
        for (std::size_t y = 0; y < config.ny; ++y) {
            for (std::size_t x = 0; x < config.nx; ++x) {
                const std::size_t here = mask.index(x, y, z);
                if (mask.solid[here] != 0) {
                    continue;
                }
                for (std::size_t i = 1; i < d3q19::q; ++i) {
                    const detail3d::Source source =
                        detail3d::resolve_source(config, mask, x, y, z, i);
                    if (source.kind == detail3d::SourceKind::bounce_back) {
                        links.push_back(BoundaryLink3D{
                            static_cast<std::uint32_t>(here), static_cast<std::uint32_t>(i),
                            detail3d::link_fraction(config, mask, x, y, z, i)});
                    }
                }
            }
        }
    }
    return links;
}

double max_speed(const MacroFields3D& fields) noexcept {
    double best = 0.0;
    for (std::size_t c = 0; c < fields.density.size(); ++c) {
        if (fields.solid[c] == 0) {
            const double u = fields.velocity_x[c];
            const double v = fields.velocity_y[c];
            const double w = fields.velocity_z[c];
            best = std::max(best, std::sqrt(u * u + v * v + w * w));
        }
    }
    return best;
}

std::array<double, 3> link_wall_point(const Lbm3DConfig& config,
                                      const BoundaryLink3D& link) noexcept {
    const std::size_t plane = config.nx * config.ny;
    if (plane == 0) {
        return {0.0, 0.0, 0.0}; // no lattice
    }
    const std::size_t z = link.cell / plane;
    const std::size_t rest = link.cell % plane;
    const std::size_t y = rest / config.nx;
    const std::size_t x = rest % config.nx;
    return {static_cast<double>(x) - link.q * d3q19::cx[link.direction],
            static_cast<double>(y) - link.q * d3q19::cy[link.direction],
            static_cast<double>(z) - link.q * d3q19::cz[link.direction]};
}

void add_link_force(Force3D& total, const Lbm3DConfig& config, const BoundaryLink3D& link,
                    double exchanged) noexcept {
    const std::size_t out = d3q19::opposite[link.direction];
    const std::array<double, 3> f{static_cast<double>(d3q19::cx[out]) * exchanged,
                                  static_cast<double>(d3q19::cy[out]) * exchanged,
                                  static_cast<double>(d3q19::cz[out]) * exchanged};
    const std::array<double, 3> r = link_wall_point(config, link);
    for (std::size_t a = 0; a < 3; ++a) {
        total.force[a] += f[a];
    }
    total.moment[0] += r[1] * f[2] - r[2] * f[1];
    total.moment[1] += r[2] * f[0] - r[0] * f[2];
    total.moment[2] += r[0] * f[1] - r[1] * f[0];
}

std::vector<double> initial_populations(const Lbm3DConfig& config, const SolidMask3D& mask,
                                        const InitialState3D& initial) {
    const std::size_t cells = mask.solid.size();
    std::vector<double> f(d3q19::q * cells);
    for (std::size_t z = 0; z < config.nz; ++z) {
        for (std::size_t y = 0; y < config.ny; ++y) {
            for (std::size_t x = 0; x < config.nx; ++x) {
                const std::size_t here = mask.index(x, y, z);
                const d3q19::Moments m =
                    mask.solid[here] != 0 ? d3q19::Moments{1.0, 0.0, 0.0, 0.0} : initial(x, y, z);
                for (std::size_t i = 0; i < d3q19::q; ++i) {
                    f[i * cells + here] = d3q19::equilibrium(i, m.rho, m.ux, m.uy, m.uz);
                }
            }
        }
    }
    return f;
}

InitialState3D uniform_inlet_state(const Lbm3DConfig& config) {
    const bool inflow = config.x_boundary == XBoundary::inflow_outflow;
    const double ux = inflow ? config.inlet_ux : 0.0;
    const double uy = inflow ? config.inlet_uy : 0.0;
    const double uz = inflow ? config.inlet_uz : 0.0;
    return [ux, uy, uz](std::size_t, std::size_t, std::size_t) {
        return d3q19::Moments{1.0, ux, uy, uz};
    };
}

std::optional<std::size_t> gpu_population_buffer_bytes(const Lbm3DConfig& config) noexcept {
    return result_of(times(times(cells_of(config), d3q19::q), kFloat32Bytes));
}

std::optional<std::size_t> gpu_memory_bytes(const Lbm3DConfig& config) noexcept {
    const Count cells = cells_of(config);
    // Two population buffers, node classes (uint32), four moment planes and
    // the outlet history (five values per outlet node).
    const Count populations = times(times(times(cells, d3q19::q), kFloat32Bytes), 2);
    const Count classes = times(cells, sizeof(std::uint32_t));
    const Count moments = times(times(cells, 4), kFloat32Bytes);
    const Count outlet = times(times(times(Count{config.ny, true}, config.nz), 5), kFloat32Bytes);
    return result_of(plus(plus(populations, classes), plus(moments, outlet)));
}

std::optional<std::size_t> gpu_memory_bytes(const Lbm3DConfig& config, std::size_t links,
                                            std::size_t wall_slots) noexcept {
    // The wall fractions (19 per slot; one slot, bound but unread, for the
    // halfway wall), and per link the packed (cell, direction) pair and the
    // link force.
    const Count slots{std::max<std::size_t>(wall_slots, 1), true};
    const Count wall = times(times(slots, d3q19::q), kFloat32Bytes);
    const Count per_link = times(Count{links, true}, 2 * sizeof(std::uint32_t) + kFloat32Bytes);
    const std::optional<std::size_t> base = gpu_memory_bytes(config);
    if (!base) {
        return std::nullopt;
    }
    return result_of(plus(Count{*base, true}, plus(wall, per_link)));
}

std::optional<std::size_t> cpu_memory_bytes(const Lbm3DConfig& config) noexcept {
    const Count cells = cells_of(config);
    // Two population arrays (double), the mask and the node classes.
    const Count populations = times(times(times(cells, d3q19::q), sizeof(double)), 2);
    return result_of(plus(populations, times(cells, 2)));
}

} // namespace orrery::aero::lbm
