// CPU reference of the D2Q9 step (double precision, oneTBB over rows). Each
// node is written by exactly one task and reads only the previous state, so
// the result does not depend on the thread count or the partition.
#include <array>
#include <cmath>
#include <utility>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "lbm2d_internal.h"
#include "orrery/aero/lbm/lbm2d.h"
#include "orrery/core/profile.h"

namespace orrery::aero::lbm {

namespace {

using d2q9::Populations;
using d2q9::q;

// 18 sqrt(2): the Smagorinsky relaxation time solves
// tau_eff^2 - tau tau_eff - (9 sqrt(2) / 2) C^2 Q / rho = 0.
constexpr double kSmagorinskyFactor = 18.0 * 1.4142135623730951;

// BGK collision with Guo forcing and the optional Smagorinsky closure;
// overwrites f with the post-collision populations.
void collide(Populations& f, const Lbm2DConfig& config) noexcept {
    // Moments and equilibrium written out for the D2Q9 velocity set (the
    // same values as d2q9::moments and d2q9::equilibrium, fewer operations).
    const double rho = f[0] + f[1] + f[2] + f[3] + f[4] + f[5] + f[6] + f[7] + f[8];
    const double jx = f[1] - f[3] + f[5] - f[6] - f[7] + f[8];
    const double jy = f[2] - f[4] + f[5] + f[6] - f[7] - f[8];
    const double fx = config.body_force_x;
    const double fy = config.body_force_y;
    const double ux = (jx + 0.5 * fx) / rho;
    const double uy = (jy + 0.5 * fy) / rho;

    const double base = 1.0 - 1.5 * (ux * ux + uy * uy);
    const std::array<double, q> cu{0.0, ux, uy, -ux, -uy, ux + uy, uy - ux, -ux - uy, ux - uy};
    Populations feq{};
    for (std::size_t i = 0; i < q; ++i) {
        feq[i] = d2q9::w[i] * rho * (base + cu[i] * (3.0 + 4.5 * cu[i]));
    }
    double tau = config.tau;
    if (config.smagorinsky.enabled) {
        double pxx = 0.0;
        double pyy = 0.0;
        double pxy = 0.0;
        for (std::size_t i = 0; i < q; ++i) {
            const double neq = f[i] - feq[i];
            const auto ex = static_cast<double>(d2q9::cx[i]);
            const auto ey = static_cast<double>(d2q9::cy[i]);
            pxx += ex * ex * neq;
            pyy += ey * ey * neq;
            pxy += ex * ey * neq;
        }
        const double strain = std::sqrt(pxx * pxx + pyy * pyy + 2.0 * pxy * pxy);
        const double c = config.smagorinsky.constant;
        tau = 0.5 * (tau + std::sqrt(tau * tau + kSmagorinskyFactor * c * c * strain / rho));
    }
    const double omega = 1.0 / tau;
    const bool forced = fx != 0.0 || fy != 0.0;
    const double force_scale = 1.0 - 0.5 * omega;
    for (std::size_t i = 0; i < q; ++i) {
        double value = f[i] - omega * (f[i] - feq[i]);
        if (forced) {
            const auto ex = static_cast<double>(d2q9::cx[i]);
            const auto ey = static_cast<double>(d2q9::cy[i]);
            value += force_scale * d2q9::w[i] *
                     (3.0 * ((ex - ux) * fx + (ey - uy) * fy) + 9.0 * cu[i] * (ex * fx + ey * fy));
        }
        f[i] = value;
    }
}

class CpuLbm2D final : public Lbm2DSolver {
public:
    CpuLbm2D(const Lbm2DConfig& config, SolidMask2D mask)
        : config_(config), mask_(std::move(mask)), cells_(config.nx * config.ny),
          links_(boundary_links(config_, mask_)) {
        for (std::size_t i = 0; i < q; ++i) {
            offset_[i] = -static_cast<std::ptrdiff_t>(d2q9::cx[i]) -
                         static_cast<std::ptrdiff_t>(config_.nx) * d2q9::cy[i];
        }
        classify_nodes();
        f_ = initial_populations(config_, mask_, uniform_inlet_state(config_));
        g_ = f_;
        reset_outlet();
    }

    [[nodiscard]] const Lbm2DConfig& config() const noexcept override { return config_; }
    [[nodiscard]] const SolidMask2D& mask() const noexcept override { return mask_; }
    [[nodiscard]] std::uint64_t steps() const noexcept override { return steps_; }

    [[nodiscard]] Status initialize(const InitialState& initial) override {
        f_ = initial_populations(config_, mask_, initial);
        g_ = f_;
        reset_outlet();
        steps_ = 0;
        return std::monostate{};
    }

    [[nodiscard]] Status step(std::size_t count) override {
        ORRERY_ZONE("lbm_step");
        for (std::size_t n = 0; n < count; ++n) {
            tbb::parallel_for(tbb::blocked_range<std::size_t>(0, config_.ny),
                              [this](const tbb::blocked_range<std::size_t>& rows) {
                                  for (std::size_t y = rows.begin(); y != rows.end(); ++y) {
                                      update_row(y);
                                  }
                              });
            std::swap(f_, g_);
            ++steps_;
        }
        return std::monostate{};
    }

    [[nodiscard]] core::Result<MacroFields2D, std::string> macroscopic() override {
        MacroFields2D out;
        out.nx = config_.nx;
        out.ny = config_.ny;
        out.density.assign(cells_, 1.0);
        out.velocity_x.assign(cells_, 0.0);
        out.velocity_y.assign(cells_, 0.0);
        out.solid = mask_.solid;
        for (std::size_t c = 0; c < cells_; ++c) {
            if (mask_.solid[c] != 0) {
                continue;
            }
            double rho = 0.0;
            double jx = 0.0;
            double jy = 0.0;
            for (std::size_t i = 0; i < q; ++i) {
                const double v = f_[i * cells_ + c];
                rho += v;
                jx += static_cast<double>(d2q9::cx[i]) * v;
                jy += static_cast<double>(d2q9::cy[i]) * v;
            }
            // Post-collision momentum is the pre-collision one plus F.
            out.density[c] = rho;
            out.velocity_x[c] = (jx - 0.5 * config_.body_force_x) / rho;
            out.velocity_y[c] = (jy - 0.5 * config_.body_force_y) / rho;
        }
        return out;
    }

    [[nodiscard]] core::Result<std::vector<double>, std::string> populations() override {
        return f_;
    }

    [[nodiscard]] core::Result<Force2D, std::string> force() override {
        Force2D total;
        for (const BoundaryLink& link : links_) {
            const std::size_t out = d2q9::opposite[link.direction];
            add_link_force(total, config_, link, f_[out * cells_ + link.cell]);
        }
        return total;
    }

    [[nodiscard]] core::Result<double, std::string> mass() override {
        double total = 0.0;
        for (std::size_t c = 0; c < cells_; ++c) {
            if (mask_.solid[c] == 0) {
                for (std::size_t i = 0; i < q; ++i) {
                    total += f_[i * cells_ + c];
                }
            }
        }
        return total;
    }

private:
    // Previous post-streaming populations 3, 6, 7 of the outlet column (the
    // convective outlet's time level t), slot-major: outlet_[slot * ny + y].
    void reset_outlet() {
        outlet_.assign(3 * config_.ny, 0.0);
        for (std::size_t y = 0; y < config_.ny; ++y) {
            const std::size_t cell = config_.nx - 1 + config_.nx * y;
            for (const std::size_t i : {std::size_t{3}, std::size_t{6}, std::size_t{7}}) {
                outlet_[detail::outlet_slot(i) * config_.ny + y] = f_[i * cells_ + cell];
            }
        }
    }

    // Incoming population i of the outlet node in row y; `upstream` is the
    // value the zero-gradient rule would copy. Each row touches only its own
    // history entries, so rows can run in parallel.
    double outlet_value(double upstream, std::size_t i, std::size_t y) noexcept {
        if (config_.outlet == Outlet::extrapolation) {
            return upstream;
        }
        const double u = config_.inlet_ux;
        double& previous = outlet_[detail::outlet_slot(i) * config_.ny + y];
        previous = (previous + u * upstream) / (1.0 + u);
        return previous;
    }

    // Node classes: solid, bulk fluid (every source node inside the domain and
    // fluid: plain streaming) and boundary fluid (resolve_source per link).
    enum : std::uint8_t { kSolid = 0, kBulk = 1, kBoundary = 2 };

    void classify_nodes() {
        node_class_.assign(cells_, std::uint8_t{kBulk});
        const std::size_t nx = config_.nx;
        const std::size_t ny = config_.ny;
        for (std::size_t y = 0; y < ny; ++y) {
            for (std::size_t x = 0; x < nx; ++x) {
                const std::size_t here = x + nx * y;
                if (mask_.solid[here] != 0) {
                    node_class_[here] = kSolid;
                    continue;
                }
                if (x == 0 || y == 0 || x + 1 == nx || y + 1 == ny) {
                    node_class_[here] = kBoundary;
                    continue;
                }
                for (std::size_t i = 1; i < q; ++i) {
                    const auto source =
                        static_cast<std::size_t>(static_cast<std::ptrdiff_t>(here) + offset_[i]);
                    if (mask_.solid[source] != 0) {
                        node_class_[here] = kBoundary;
                        break;
                    }
                }
            }
        }
    }

    // Sum of the current populations of node `cell`.
    [[nodiscard]] double density_at(std::size_t cell) const noexcept {
        double rho = 0.0;
        for (std::size_t i = 0; i < q; ++i) {
            rho += f_[i * cells_ + cell];
        }
        return rho;
    }

    void update_row(std::size_t y) noexcept {
        const std::size_t nx = config_.nx;
        const bool inlet = config_.x_boundary == XBoundary::inflow_outflow;
        for (std::size_t x = 0; x < nx; ++x) {
            const std::size_t here = x + nx * y;
            const std::uint8_t node = node_class_[here];
            if (node == kSolid) {
                continue;
            }
            Populations f{};
            if (node == kBulk) {
                for (std::size_t i = 0; i < q; ++i) {
                    f[i] = f_[i * cells_ + static_cast<std::size_t>(
                                               static_cast<std::ptrdiff_t>(here) + offset_[i])];
                }
            } else {
                // Density of this node at the previous step (collision keeps
                // it), the wall density of the moving-wall term; computed on
                // first use.
                double wall_rho = -1.0;
                for (std::size_t i = 0; i < q; ++i) {
                    const detail::Source s = detail::resolve_source(config_, mask_, x, y, i);
                    switch (s.kind) {
                        case detail::SourceKind::node:
                        case detail::SourceKind::bounce_back:
                            f[i] = f_[s.direction * cells_ + s.cell];
                            break;
                        case detail::SourceKind::wall:
                            if (wall_rho < 0.0) {
                                wall_rho = density_at(here);
                            }
                            f[i] = f_[s.direction * cells_ + s.cell] +
                                   6.0 * d2q9::w[i] * wall_rho * static_cast<double>(d2q9::cx[i]) *
                                       s.wall_ux;
                            break;
                        case detail::SourceKind::inlet:
                            f[i] = 0.0;
                            break;
                        case detail::SourceKind::outlet:
                            f[i] = outlet_value(f_[s.direction * cells_ + s.cell], i, y);
                            break;
                    }
                }
                if (inlet && x == 0) {
                    detail::zou_he_west(f, config_.inlet_ux, config_.inlet_uy);
                }
            }
            collide(f, config_);
            for (std::size_t i = 0; i < q; ++i) {
                g_[i * cells_ + here] = f[i];
            }
        }
    }

    Lbm2DConfig config_;
    SolidMask2D mask_;
    std::size_t cells_ = 0;
    std::vector<BoundaryLink> links_;
    std::array<std::ptrdiff_t, q> offset_{};
    std::vector<std::uint8_t> node_class_;
    std::vector<double> f_;
    std::vector<double> g_;
    std::vector<double> outlet_;
    std::uint64_t steps_ = 0;
};

} // namespace

core::Result<std::unique_ptr<Lbm2DSolver>, std::string> make_cpu_solver(const Lbm2DConfig& config,
                                                                        SolidMask2D mask) {
    if (auto problem = validate(config, mask)) {
        return core::failure(std::move(*problem));
    }
    return std::unique_ptr<Lbm2DSolver>(std::make_unique<CpuLbm2D>(config, std::move(mask)));
}

} // namespace orrery::aero::lbm
