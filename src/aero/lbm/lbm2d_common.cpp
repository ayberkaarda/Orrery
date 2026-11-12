#include <algorithm>
#include <cmath>
#include <optional>
#include <sstream>

#include "lbm2d_internal.h"
#include "orrery/aero/lbm/lattice_units.h"
#include "orrery/aero/lbm/lbm2d.h"

namespace orrery::aero::lbm {

namespace {

bool finite(double value) noexcept {
    return std::isfinite(value);
}

std::optional<std::string> check_mach(const char* what, double speed) {
    if (!(lattice_mach(speed) < max_lattice_mach)) {
        std::ostringstream message;
        message << "lbm2d: " << what << " " << speed << " has lattice Mach number "
                << lattice_mach(speed) << ", not below " << max_lattice_mach
                << " (APPROXIMATION A3)";
        return message.str();
    }
    return std::nullopt;
}

} // namespace

std::optional<std::string> validate(const Lbm2DConfig& config, const SolidMask2D& mask) {
    if (config.nx < 3 || config.ny < 3) {
        return std::string("lbm2d: the lattice needs at least 3 x 3 nodes");
    }
    if (config.nx > geom::max_grid_cells / config.ny) {
        return std::string("lbm2d: more than geom::max_grid_cells nodes");
    }
    if (mask.nx != config.nx || mask.ny != config.ny ||
        mask.solid.size() != config.nx * config.ny) {
        return std::string("lbm2d: the solid mask does not match the lattice size");
    }
    if (!finite(config.tau) || !(config.tau > 0.5)) {
        return std::string("lbm2d: tau must be finite and above 1/2 (positive viscosity)");
    }
    if (!finite(config.inlet_ux) || !finite(config.inlet_uy) || !finite(config.body_force_x) ||
        !finite(config.body_force_y) || !finite(config.bottom.velocity_x) ||
        !finite(config.top.velocity_x)) {
        return std::string("lbm2d: velocities and body force must be finite");
    }
    if (config.smagorinsky.enabled &&
        (!finite(config.smagorinsky.constant) || config.smagorinsky.constant < 0.0)) {
        return std::string("lbm2d: the Smagorinsky constant must be finite and non-negative");
    }
    if ((config.bottom.kind == WallKind::periodic) != (config.top.kind == WallKind::periodic)) {
        return std::string("lbm2d: y is periodic on both sides or on neither");
    }
    if (auto bad = check_mach("bottom wall speed", config.bottom.velocity_x)) {
        return bad;
    }
    if (auto bad = check_mach("top wall speed", config.top.velocity_x)) {
        return bad;
    }
    if (config.x_boundary == XBoundary::inflow_outflow) {
        if (auto bad = check_mach("inlet speed", std::hypot(config.inlet_ux, config.inlet_uy))) {
            return bad;
        }
        for (std::size_t y = 0; y < config.ny; ++y) {
            if (mask.at(0, y) || mask.at(config.nx - 1, y)) {
                return std::string("lbm2d: the inlet and outlet columns must be fluid");
            }
        }
        if (config.outlet == Outlet::convective && !(config.inlet_ux > 0.0)) {
            return std::string("lbm2d: the convective outlet needs a positive inlet speed");
        }
    }
    return std::nullopt;
}

std::vector<BoundaryLink> boundary_links(const Lbm2DConfig& config, const SolidMask2D& mask) {
    std::vector<BoundaryLink> links;
    for (std::size_t y = 0; y < config.ny; ++y) {
        for (std::size_t x = 0; x < config.nx; ++x) {
            const std::size_t here = x + config.nx * y;
            if (mask.solid[here] != 0) {
                continue;
            }
            for (std::size_t i = 1; i < d2q9::q; ++i) {
                const detail::Source source = detail::resolve_source(config, mask, x, y, i);
                if (source.kind == detail::SourceKind::bounce_back) {
                    links.push_back(BoundaryLink{static_cast<std::uint32_t>(here),
                                                 static_cast<std::uint32_t>(i)});
                }
            }
        }
    }
    return links;
}

std::vector<double> pressure(const MacroFields2D& fields) {
    std::vector<double> p(fields.density.size());
    for (std::size_t c = 0; c < p.size(); ++c) {
        p[c] = d2q9::cs2 * (fields.density[c] - 1.0);
    }
    return p;
}

namespace {

// Derivative of `u` along one axis at a node from its two neighbours (either
// may be unavailable: outside a non-periodic edge or solid).
double derivative(const std::vector<double>& u, std::size_t here, std::optional<std::size_t> lo,
                  std::optional<std::size_t> hi) {
    if (lo && hi) {
        return 0.5 * (u[*hi] - u[*lo]);
    }
    if (hi) {
        return u[*hi] - u[here];
    }
    if (lo) {
        return u[here] - u[*lo];
    }
    return 0.0;
}

} // namespace

std::vector<double> vorticity(const MacroFields2D& fields, const Lbm2DConfig& config) {
    const std::size_t nx = fields.nx;
    const std::size_t ny = fields.ny;
    std::vector<double> omega(nx * ny, 0.0);
    const bool x_periodic = config.x_boundary == XBoundary::periodic;
    const bool y_periodic = config.bottom.kind == WallKind::periodic;
    const auto fluid = [&](std::size_t cell) -> std::optional<std::size_t> {
        if (fields.solid[cell] != 0) {
            return std::nullopt;
        }
        return cell;
    };
    for (std::size_t y = 0; y < ny; ++y) {
        for (std::size_t x = 0; x < nx; ++x) {
            const std::size_t here = x + nx * y;
            if (fields.solid[here] != 0) {
                continue;
            }
            std::optional<std::size_t> left;
            std::optional<std::size_t> right;
            std::optional<std::size_t> down;
            std::optional<std::size_t> up;
            if (x > 0 || x_periodic) {
                left = fluid((x + nx - 1) % nx + nx * y);
            }
            if (x + 1 < nx || x_periodic) {
                right = fluid((x + 1) % nx + nx * y);
            }
            if (y > 0 || y_periodic) {
                down = fluid(x + nx * ((y + ny - 1) % ny));
            }
            if (y + 1 < ny || y_periodic) {
                up = fluid(x + nx * ((y + 1) % ny));
            }
            omega[here] = derivative(fields.velocity_y, here, left, right) -
                          derivative(fields.velocity_x, here, down, up);
        }
    }
    return omega;
}

double max_speed(const MacroFields2D& fields) noexcept {
    double best = 0.0;
    for (std::size_t c = 0; c < fields.density.size(); ++c) {
        if (fields.solid[c] == 0) {
            best = std::max(best, std::hypot(fields.velocity_x[c], fields.velocity_y[c]));
        }
    }
    return best;
}

std::vector<double> initial_populations(const Lbm2DConfig& config, const SolidMask2D& mask,
                                        const InitialState& initial) {
    const std::size_t cells = config.nx * config.ny;
    std::vector<double> f(d2q9::q * cells);
    for (std::size_t y = 0; y < config.ny; ++y) {
        for (std::size_t x = 0; x < config.nx; ++x) {
            const std::size_t here = x + config.nx * y;
            const d2q9::Moments m =
                mask.solid[here] != 0 ? d2q9::Moments{1.0, 0.0, 0.0} : initial(x, y);
            for (std::size_t i = 0; i < d2q9::q; ++i) {
                f[i * cells + here] = d2q9::equilibrium(i, m.rho, m.ux, m.uy);
            }
        }
    }
    return f;
}

InitialState uniform_inlet_state(const Lbm2DConfig& config) {
    const double ux = config.x_boundary == XBoundary::inflow_outflow ? config.inlet_ux : 0.0;
    const double uy = config.x_boundary == XBoundary::inflow_outflow ? config.inlet_uy : 0.0;
    return [ux, uy](std::size_t, std::size_t) { return d2q9::Moments{1.0, ux, uy}; };
}

} // namespace orrery::aero::lbm
