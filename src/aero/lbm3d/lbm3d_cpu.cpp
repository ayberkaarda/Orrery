// CPU reference of the D3Q19 step (double precision, oneTBB over rows of
// constant y and z). Each node is written by exactly one task and reads only
// the previous state, so the result does not depend on the thread count or
// the partition.
#include <array>
#include <cmath>
#include <utility>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "lbm3d_internal.h"
#include "orrery/aero/lbm3d/lbm3d.h"
#include "orrery/core/profile.h"

namespace orrery::aero::lbm {

namespace {

using d3q19::Populations;
using d3q19::q;

// 18 sqrt(2): the Smagorinsky relaxation time solves
// tau_eff^2 - tau tau_eff - (9 sqrt(2) / 2) C^2 Q / rho = 0 (the same in two
// and three dimensions).
constexpr double kSmagorinskyFactor = 18.0 * 1.4142135623730951;

// BGK collision with Guo forcing and the optional Smagorinsky closure,
// followed by the absorbing-layer relaxation of rate `sponge` towards the
// free-stream equilibrium (density 1, velocity `stream`); overwrites f with
// the post-collision populations.
void collide(Populations& f, const Lbm3DConfig& config, double sponge,
             const std::array<double, 3>& stream) noexcept {
    // Moments and c.u written out for the D3Q19 velocity set (the same values
    // as d3q19::moments, fewer operations).
    double rho = 0.0;
    for (const double v : f) {
        rho += v;
    }
    const double jx = f[1] - f[2] + f[7] - f[8] + f[9] - f[10] + f[11] - f[12] + f[13] - f[14];
    const double jy = f[3] - f[4] + f[7] - f[8] - f[9] + f[10] + f[15] - f[16] + f[17] - f[18];
    const double jz = f[5] - f[6] + f[11] - f[12] - f[13] + f[14] + f[15] - f[16] - f[17] + f[18];
    const double fx = config.body_force_x;
    const double fy = config.body_force_y;
    const double fz = config.body_force_z;
    const double ux = (jx + 0.5 * fx) / rho;
    const double uy = (jy + 0.5 * fy) / rho;
    const double uz = (jz + 0.5 * fz) / rho;

    const double base = 1.0 - 1.5 * (ux * ux + uy * uy + uz * uz);
    const std::array<double, q> cu{0.0,     ux,       -ux,      uy,      -uy,     uz,       -uz,
                                   ux + uy, -ux - uy, ux - uy,  uy - ux, ux + uz, -ux - uz, ux - uz,
                                   uz - ux, uy + uz,  -uy - uz, uy - uz, uz - uy};
    Populations feq{};
    for (std::size_t i = 0; i < q; ++i) {
        feq[i] = d3q19::w[i] * rho * (base + cu[i] * (3.0 + 4.5 * cu[i]));
    }
    double tau = config.tau;
    if (config.smagorinsky.enabled) {
        // Pi_neq: xx, yy, zz, xy, xz, yz.
        std::array<double, 6> pi{};
        for (std::size_t i = 0; i < q; ++i) {
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
        const double strain = std::sqrt(pi[0] * pi[0] + pi[1] * pi[1] + pi[2] * pi[2] +
                                        2.0 * (pi[3] * pi[3] + pi[4] * pi[4] + pi[5] * pi[5]));
        const double c = config.smagorinsky.constant;
        tau = 0.5 * (tau + std::sqrt(tau * tau + kSmagorinskyFactor * c * c * strain / rho));
    }
    const double omega = 1.0 / tau;
    const bool forced = fx != 0.0 || fy != 0.0 || fz != 0.0;
    const double force_scale = 1.0 - 0.5 * omega;
    for (std::size_t i = 0; i < q; ++i) {
        double value = f[i] - omega * (f[i] - feq[i]);
        if (forced) {
            const auto ex = static_cast<double>(d3q19::cx[i]);
            const auto ey = static_cast<double>(d3q19::cy[i]);
            const auto ez = static_cast<double>(d3q19::cz[i]);
            value += force_scale * d3q19::w[i] *
                     (3.0 * ((ex - ux) * fx + (ey - uy) * fy + (ez - uz) * fz) +
                      9.0 * cu[i] * (ex * fx + ey * fy + ez * fz));
        }
        f[i] = value;
    }
    if (sponge > 0.0) {
        for (std::size_t i = 0; i < q; ++i) {
            const double target = d3q19::equilibrium(i, 1.0, stream[0], stream[1], stream[2]);
            f[i] -= sponge * (f[i] - target);
        }
    }
}

class CpuLbm3D final : public Lbm3DSolver {
public:
    CpuLbm3D(const Lbm3DConfig& config, SolidMask3D mask)
        : config_(config), mask_(std::move(mask)), cells_(mask_.solid.size()),
          links_(boundary_links(config_, mask_)), offset_(detail3d::source_offsets(config_)),
          node_class_(detail3d::classify_nodes(config_, mask_)),
          walls_(detail3d::wall_table(config_, mask_, links_)),
          f_(initial_populations(config_, mask_, uniform_inlet_state(config_))), g_(f_) {
        reset_outlet();
    }

    [[nodiscard]] Status set_inlet_velocity(const std::array<double, 3>& velocity) override {
        if (auto problem = detail3d::check_inlet_velocity(config_, velocity)) {
            return core::failure(std::move(*problem));
        }
        inlet_ = velocity;
        return std::monostate{};
    }
    [[nodiscard]] std::array<double, 3> inlet_velocity() const noexcept override { return inlet_; }

    [[nodiscard]] const Lbm3DConfig& config() const noexcept override { return config_; }
    [[nodiscard]] const SolidMask3D& mask() const noexcept override { return mask_; }
    [[nodiscard]] std::uint64_t steps() const noexcept override { return steps_; }
    [[nodiscard]] const std::vector<BoundaryLink3D>& links() const noexcept override {
        return links_;
    }

    [[nodiscard]] Status initialize(const InitialState3D& initial) override {
        f_ = initial_populations(config_, mask_, initial);
        g_ = f_;
        reset_outlet();
        inlet_ = {config_.inlet_ux, config_.inlet_uy, config_.inlet_uz};
        steps_ = 0;
        return std::monostate{};
    }

    [[nodiscard]] Status step(std::size_t count) override {
        ORRERY_ZONE("lbm_step");
        const std::size_t rows = config_.ny * config_.nz;
        for (std::size_t n = 0; n < count; ++n) {
            tbb::parallel_for(tbb::blocked_range<std::size_t>(0, rows),
                              [this](const tbb::blocked_range<std::size_t>& range) {
                                  for (std::size_t r = range.begin(); r != range.end(); ++r) {
                                      update_row(r % config_.ny, r / config_.ny);
                                  }
                              });
            std::swap(f_, g_);
            ++steps_;
        }
        return std::monostate{};
    }

    [[nodiscard]] core::Result<MacroFields3D, std::string> macroscopic() override {
        MacroFields3D out;
        out.nx = config_.nx;
        out.ny = config_.ny;
        out.nz = config_.nz;
        out.density.assign(cells_, 1.0);
        out.velocity_x.assign(cells_, 0.0);
        out.velocity_y.assign(cells_, 0.0);
        out.velocity_z.assign(cells_, 0.0);
        out.solid = mask_.solid;
        for (std::size_t c = 0; c < cells_; ++c) {
            if (mask_.solid[c] != 0) {
                continue;
            }
            double rho = 0.0;
            double jx = 0.0;
            double jy = 0.0;
            double jz = 0.0;
            for (std::size_t i = 0; i < q; ++i) {
                const double v = f_[i * cells_ + c];
                rho += v;
                jx += static_cast<double>(d3q19::cx[i]) * v;
                jy += static_cast<double>(d3q19::cy[i]) * v;
                jz += static_cast<double>(d3q19::cz[i]) * v;
            }
            // Post-collision momentum is the pre-collision one plus F.
            out.density[c] = rho;
            out.velocity_x[c] = (jx - 0.5 * config_.body_force_x) / rho;
            out.velocity_y[c] = (jy - 0.5 * config_.body_force_y) / rho;
            out.velocity_z[c] = (jz - 0.5 * config_.body_force_z) / rho;
        }
        return out;
    }

    [[nodiscard]] core::Result<std::vector<double>, std::string> populations() override {
        return f_;
    }

    [[nodiscard]] core::Result<Force3D, std::string> force() override {
        Force3D total;
        for (const BoundaryLink3D& link : links_) {
            const std::size_t out = d3q19::opposite[link.direction];
            const auto farther = static_cast<std::size_t>(static_cast<std::ptrdiff_t>(link.cell) -
                                                          offset_[link.direction]);
            const double back =
                detail3d::reflected(f_, cells_, link.cell, farther, link.direction, link.q);
            add_link_force(total, config_, link, f_[out * cells_ + link.cell] + back);
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
    // Index of an outlet node's history entry, slot-major:
    // slot * ny * nz + y + ny z.
    [[nodiscard]] std::size_t outlet_index(std::size_t slot, std::size_t y,
                                           std::size_t z) const noexcept {
        return slot * config_.ny * config_.nz + y + config_.ny * z;
    }

    // Previous post-streaming populations entering the outlet face (the
    // convective outlet's time level t).
    void reset_outlet() {
        outlet_.assign(detail3d::outlet_directions.size() * config_.ny * config_.nz, 0.0);
        for (std::size_t z = 0; z < config_.nz; ++z) {
            for (std::size_t y = 0; y < config_.ny; ++y) {
                const std::size_t cell = mask_.index(config_.nx - 1, y, z);
                for (const std::size_t i : detail3d::outlet_directions) {
                    outlet_[outlet_index(detail3d::outlet_slot(i), y, z)] = f_[i * cells_ + cell];
                }
            }
        }
    }

    // Incoming population i of the outlet node (y, z); `upstream` is the
    // value the zero-gradient rule would copy. Each row touches only its own
    // history entries, so rows can run in parallel.
    double outlet_value(double upstream, std::size_t i, std::size_t y, std::size_t z) noexcept {
        if (config_.outlet == Outlet::extrapolation) {
            return upstream;
        }
        const double u = config_.inlet_ux;
        double& previous = outlet_[outlet_index(detail3d::outlet_slot(i), y, z)];
        previous = (previous + u * upstream) / (1.0 + u);
        return previous;
    }

    // Sum of the current populations of node `cell`.
    [[nodiscard]] double density_at(std::size_t cell) const noexcept {
        double rho = 0.0;
        for (std::size_t i = 0; i < q; ++i) {
            rho += f_[i * cells_ + cell];
        }
        return rho;
    }

    void update_row(std::size_t y, std::size_t z) noexcept {
        const std::size_t nx = config_.nx;
        const bool inlet = config_.x_boundary == XBoundary::inflow_outflow;
        const std::size_t row = mask_.index(0, y, z);
        for (std::size_t x = 0; x < nx; ++x) {
            const std::size_t here = row + x;
            const std::uint8_t node = node_class_[here];
            if (node == detail3d::node_solid) {
                continue;
            }
            Populations f{};
            if (node == detail3d::node_bulk) {
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
                    const detail3d::Source s = detail3d::resolve_source(config_, mask_, x, y, z, i);
                    switch (s.kind) {
                        case detail3d::SourceKind::node:
                            f[i] = f_[s.direction * cells_ + s.cell];
                            break;
                        case detail3d::SourceKind::bounce_back:
                            f[i] = walls_.slot.empty() || walls_.slot[here] == detail3d::kNoSlot
                                       ? f_[s.direction * cells_ + s.cell]
                                       : detail3d::reflected(
                                             f_, cells_, here,
                                             static_cast<std::size_t>(
                                                 static_cast<std::ptrdiff_t>(here) - offset_[i]),
                                             i, walls_.q[std::size_t{walls_.slot[here]} * q + i]);
                            break;
                        case detail3d::SourceKind::wall:
                            if (wall_rho < 0.0) {
                                wall_rho = density_at(here);
                            }
                            f[i] = f_[s.direction * cells_ + s.cell] +
                                   6.0 * d3q19::w[i] * wall_rho *
                                       static_cast<double>(d3q19::cx[i]) * s.wall_ux;
                            break;
                        case detail3d::SourceKind::inlet:
                            f[i] = 0.0;
                            break;
                        case detail3d::SourceKind::outlet:
                            f[i] = outlet_value(f_[s.direction * cells_ + s.cell], i, y, z);
                            break;
                    }
                }
                if (inlet && x == 0) {
                    detail3d::velocity_inlet_west(f, inlet_[0], inlet_[1], inlet_[2]);
                }
            }
            collide(f, config_, detail3d::sponge_sigma(config_, x, y, z), inlet_);
            for (std::size_t i = 0; i < q; ++i) {
                g_[i * cells_ + here] = f[i];
            }
        }
    }

    Lbm3DConfig config_;
    SolidMask3D mask_;
    std::size_t cells_ = 0;
    std::vector<BoundaryLink3D> links_;
    std::array<std::ptrdiff_t, q> offset_{};
    std::vector<std::uint8_t> node_class_;
    detail3d::WallTable walls_;
    std::vector<double> f_;
    std::vector<double> g_;
    std::vector<double> outlet_;
    // Velocity the inlet imposes (set_inlet_velocity), config_'s by default.
    std::array<double, 3> inlet_{config_.inlet_ux, config_.inlet_uy, config_.inlet_uz};
    std::uint64_t steps_ = 0;
};

} // namespace

core::Result<std::unique_ptr<Lbm3DSolver>, std::string> make_cpu_solver(const Lbm3DConfig& config,
                                                                        SolidMask3D mask) {
    if (auto problem = validate(config, mask)) {
        return core::failure(std::move(*problem));
    }
    return std::unique_ptr<Lbm3DSolver>(std::make_unique<CpuLbm3D>(config, std::move(mask)));
}

} // namespace orrery::aero::lbm
