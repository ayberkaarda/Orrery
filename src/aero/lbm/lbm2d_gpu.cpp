// GPU D2Q9 solver: the step of lbm2d_cpu.cpp run by the compute kernels
// lbm_d2q9_step, lbm_d2q9_moments and lbm_d2q9_force through an
// IComputeBackend. Device buffers hold binary32 values; populations are
// stored shifted by their lattice weight (f_i - w_i), which keeps the small
// deviations that carry the flow out of the rounding error of the O(1)
// weights (Skordos 1993; docs/lbm.md, "GPU precision"). Every host-side sum
// (force, mass) is done in double after readback.
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include "orrery/aero/compute/float32_pack.h"
#include "orrery/aero/lbm/lbm2d.h"
#include "orrery/core/profile.h"

namespace orrery::aero::lbm {

namespace {

using compute::BufferBinding;
using compute::BufferId;
using compute::DispatchCommand;
using compute::DispatchSize;
using compute::Float32Uniform;
using compute::IComputeBackend;
using compute::ScopedBuffer;
using compute::Uniform;
using d2q9::q;

// Binding points of the kernels (src/render/shaders/lbm/*.glsl).
constexpr std::uint32_t kBindSource = 0;
constexpr std::uint32_t kBindSolid = 1;
constexpr std::uint32_t kBindDestination = 2;
constexpr std::uint32_t kBindMoments = 3;
constexpr std::uint32_t kBindLinks = 4;
constexpr std::uint32_t kBindLinkForce = 5;
constexpr std::uint32_t kBindOutlet = 6;

std::uint32_t wall_code(WallKind kind) noexcept {
    switch (kind) {
        case WallKind::periodic:
            return 0;
        case WallKind::slip:
            return 1;
        case WallKind::no_slip:
            return 2;
    }
    return 0;
}

template <typename T> std::span<const std::byte> bytes_of(const std::vector<T>& values) noexcept {
    return std::as_bytes(std::span<const T>(values));
}

std::string with_context(const char* what, const std::string& error) {
    return std::string("lbm2d gpu: ") + what + ": " + error;
}

// The kernels see every real parameter rounded to binary32: each must stay
// finite, and tau must stay above 1/2 (a tau that rounds to 1/2 is zero
// viscosity on the GPU while the CPU keeps a positive one).
std::optional<std::string> check_binary32(const Lbm2DConfig& config) {
    const double c = config.smagorinsky.enabled ? config.smagorinsky.constant : 0.0;
    struct Parameter {
        const char* name;
        double value;
    };
    for (const Parameter p :
         {Parameter{"tau", config.tau}, Parameter{"bottom wall speed", config.bottom.velocity_x},
          Parameter{"top wall speed", config.top.velocity_x},
          Parameter{"inlet x velocity", config.inlet_ux},
          Parameter{"inlet y velocity", config.inlet_uy},
          Parameter{"squared Smagorinsky constant", c * c},
          Parameter{"body force x", config.body_force_x},
          Parameter{"body force y", config.body_force_y}}) {
        if (!std::isfinite(compute::round_to_float32(p.value))) {
            std::ostringstream message;
            message << "lbm2d gpu: " << p.name << " " << p.value
                    << " is outside the binary32 range of the kernels";
            return message.str();
        }
    }
    if (!(compute::round_to_float32(config.tau) > 0.5)) {
        std::ostringstream message;
        message << std::setprecision(17) << "lbm2d gpu: tau " << config.tau
                << " rounds to 1/2 in binary32 (zero viscosity in the kernels); tau - 1/2 must "
                   "exceed 2^-25 (about 3e-8)";
        return message.str();
    }
    return std::nullopt;
}

class GpuLbm2D final : public Lbm2DSolver {
public:
    GpuLbm2D(IComputeBackend& backend, const Lbm2DConfig& config, SolidMask2D mask)
        : backend_(&backend), config_(config), mask_(std::move(mask)),
          cells_(config.nx * config.ny), links_(boundary_links(config_, mask_)) {
        const double c = config_.smagorinsky.enabled ? config_.smagorinsky.constant : 0.0;
        uniforms_ = {
            Uniform{"u_nx", static_cast<std::uint32_t>(config_.nx)},
            Uniform{"u_ny", static_cast<std::uint32_t>(config_.ny)},
            Uniform{"u_x_mode",
                    static_cast<std::uint32_t>(config_.x_boundary == XBoundary::inflow_outflow)},
            Uniform{"u_bottom_kind", wall_code(config_.bottom.kind)},
            Uniform{"u_top_kind", wall_code(config_.top.kind)},
            Uniform{"u_bottom_ux", Float32Uniform{config_.bottom.velocity_x}},
            Uniform{"u_top_ux", Float32Uniform{config_.top.velocity_x}},
            Uniform{"u_inlet_ux", Float32Uniform{config_.inlet_ux}},
            Uniform{"u_inlet_uy", Float32Uniform{config_.inlet_uy}},
            Uniform{"u_tau", Float32Uniform{config_.tau}},
            Uniform{"u_smagorinsky_c2", Float32Uniform{c * c}},
            Uniform{"u_force_x", Float32Uniform{config_.body_force_x}},
            Uniform{"u_force_y", Float32Uniform{config_.body_force_y}},
            Uniform{"u_link_count", static_cast<std::uint32_t>(links_.size())},
            Uniform{"u_outlet_mode",
                    static_cast<std::uint32_t>(config_.outlet == Outlet::extrapolation)},
            Uniform{"u_outlet_speed", Float32Uniform{config_.inlet_ux}},
        };
    }

    // Allocates and fills the device buffers.
    [[nodiscard]] Status allocate() {
        const std::size_t population_bytes = q * cells_ * compute::float32_bytes;
        for (ScopedBuffer& buffer : populations_) {
            auto made = compute::make_buffer(*backend_, population_bytes);
            if (!made) {
                return core::failure(with_context("population buffer", made.error()));
            }
            buffer = std::move(made).value();
        }
        auto solid = compute::make_buffer(*backend_, cells_ * sizeof(std::uint32_t));
        if (!solid) {
            return core::failure(with_context("solid buffer", solid.error()));
        }
        solid_ = std::move(solid).value();
        std::vector<std::uint32_t> flags(cells_, 0U);
        for (std::size_t c = 0; c < cells_; ++c) {
            flags[c] = mask_.solid[c] != 0 ? 1U : 0U;
        }
        if (auto up = backend_->upload(solid_.id(), 0, bytes_of(flags)); !up) {
            return core::failure(with_context("solid upload", up.error()));
        }
        auto moments = compute::make_buffer(*backend_, 3 * cells_ * compute::float32_bytes);
        if (!moments) {
            return core::failure(with_context("moment buffer", moments.error()));
        }
        moments_ = std::move(moments).value();
        auto outlet = compute::make_buffer(*backend_, 3 * config_.ny * compute::float32_bytes);
        if (!outlet) {
            return core::failure(with_context("outlet buffer", outlet.error()));
        }
        outlet_ = std::move(outlet).value();
        if (!links_.empty()) {
            std::vector<std::uint32_t> packed;
            packed.reserve(2 * links_.size());
            for (const BoundaryLink& link : links_) {
                packed.push_back(link.cell);
                packed.push_back(link.direction);
            }
            auto links = compute::make_buffer(*backend_, packed.size() * sizeof(std::uint32_t));
            if (!links) {
                return core::failure(with_context("link buffer", links.error()));
            }
            link_buffer_ = std::move(links).value();
            if (auto up = backend_->upload(link_buffer_.id(), 0, bytes_of(packed)); !up) {
                return core::failure(with_context("link upload", up.error()));
            }
            auto force =
                compute::make_buffer(*backend_, 2 * links_.size() * compute::float32_bytes);
            if (!force) {
                return core::failure(with_context("link force buffer", force.error()));
            }
            link_force_ = std::move(force).value();
        }
        return initialize(uniform_inlet_state(config_));
    }

    [[nodiscard]] const Lbm2DConfig& config() const noexcept override { return config_; }
    [[nodiscard]] const SolidMask2D& mask() const noexcept override { return mask_; }
    [[nodiscard]] std::uint64_t steps() const noexcept override { return steps_; }

    [[nodiscard]] Status initialize(const InitialState& initial) override {
        std::vector<double> f = initial_populations(config_, mask_, initial);
        for (std::size_t i = 0; i < q; ++i) {
            for (std::size_t c = 0; c < cells_; ++c) {
                f[i * cells_ + c] -= d2q9::w[i];
            }
        }
        const std::vector<std::byte> packed = compute::pack_float32(f);
        for (const ScopedBuffer& buffer : populations_) {
            if (auto up = backend_->upload(buffer.id(), 0, packed); !up) {
                return core::failure(with_context("population upload", up.error()));
            }
        }
        // History of the convective outlet: populations 3, 6, 7 of the
        // outlet column, slot-major (slot * ny + y), shifted like f.
        std::vector<double> history(3 * config_.ny);
        for (std::size_t y = 0; y < config_.ny; ++y) {
            const std::size_t cell = config_.nx - 1 + config_.nx * y;
            history[y] = f[3 * cells_ + cell];
            history[config_.ny + y] = f[6 * cells_ + cell];
            history[2 * config_.ny + y] = f[7 * cells_ + cell];
        }
        if (auto up = backend_->upload(outlet_.id(), 0, compute::pack_float32(history)); !up) {
            return core::failure(with_context("outlet upload", up.error()));
        }
        current_ = 0;
        steps_ = 0;
        return std::monostate{};
    }

    [[nodiscard]] Status step(std::size_t count) override {
        ORRERY_ZONE("lbm_step");
        const DispatchSize size{static_cast<std::uint32_t>(config_.nx),
                                static_cast<std::uint32_t>(config_.ny), 1};
        for (std::size_t n = 0; n < count; ++n) {
            const std::array<BufferBinding, 4> bindings{
                BufferBinding{kBindSource, populations_[current_].id()},
                BufferBinding{kBindSolid, solid_.id()},
                BufferBinding{kBindDestination, populations_[1 - current_].id()},
                BufferBinding{kBindOutlet, outlet_.id()},
            };
            const DispatchCommand command{kernel_step, size, uniforms_, bindings};
            if (auto done = backend_->dispatch(command); !done) {
                return core::failure(with_context("step", done.error()));
            }
            backend_->barrier();
            current_ = 1 - current_;
            ++steps_;
        }
        return std::monostate{};
    }

    [[nodiscard]] core::Result<MacroFields2D, std::string> macroscopic() override {
        const std::array<BufferBinding, 3> bindings{
            BufferBinding{kBindSource, populations_[current_].id()},
            BufferBinding{kBindSolid, solid_.id()},
            BufferBinding{kBindMoments, moments_.id()},
        };
        const DispatchSize size{static_cast<std::uint32_t>(config_.nx),
                                static_cast<std::uint32_t>(config_.ny), 1};
        if (auto done = backend_->dispatch({kernel_moments, size, uniforms_, bindings}); !done) {
            return core::failure(with_context("moments", done.error()));
        }
        backend_->barrier();
        std::vector<double> values(3 * cells_);
        if (auto read = read_float32(moments_.id(), values); !read) {
            return core::failure(std::string(read.error()));
        }
        MacroFields2D out;
        out.nx = config_.nx;
        out.ny = config_.ny;
        out.density.assign(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(cells_));
        for (double& rho : out.density) {
            rho += 1.0; // the kernel writes rho - 1
        }
        out.velocity_x.assign(values.begin() + static_cast<std::ptrdiff_t>(cells_),
                              values.begin() + static_cast<std::ptrdiff_t>(2 * cells_));
        out.velocity_y.assign(values.begin() + static_cast<std::ptrdiff_t>(2 * cells_),
                              values.end());
        out.solid = mask_.solid;
        return out;
    }

    [[nodiscard]] core::Result<std::vector<double>, std::string> populations() override {
        std::vector<double> f(q * cells_);
        if (auto read = read_float32(populations_[current_].id(), f); !read) {
            return core::failure(std::string(read.error()));
        }
        for (std::size_t i = 0; i < q; ++i) {
            for (std::size_t c = 0; c < cells_; ++c) {
                f[i * cells_ + c] += d2q9::w[i];
            }
        }
        return f;
    }

    [[nodiscard]] core::Result<Force2D, std::string> force() override {
        if (links_.empty()) {
            return Force2D{};
        }
        const std::array<BufferBinding, 3> bindings{
            BufferBinding{kBindSource, populations_[current_].id()},
            BufferBinding{kBindLinks, link_buffer_.id()},
            BufferBinding{kBindLinkForce, link_force_.id()},
        };
        const DispatchSize size{static_cast<std::uint32_t>(links_.size()), 1, 1};
        if (auto done = backend_->dispatch({kernel_force, size, uniforms_, bindings}); !done) {
            return core::failure(with_context("force", done.error()));
        }
        backend_->barrier();
        std::vector<double> parts(2 * links_.size());
        if (auto read = read_float32(link_force_.id(), parts); !read) {
            return core::failure(std::string(read.error()));
        }
        // The kernel writes 2 c_o fs_o per link; adding the weight back gives
        // 2 c_o f_o. Only one component of c_o is needed to recover fs_o.
        Force2D total;
        for (std::size_t k = 0; k < links_.size(); ++k) {
            const BoundaryLink& link = links_[k];
            const std::size_t out = d2q9::opposite[link.direction];
            const double shifted = d2q9::cx[out] != 0 ? parts[2 * k] / (2.0 * d2q9::cx[out])
                                                      : parts[2 * k + 1] / (2.0 * d2q9::cy[out]);
            add_link_force(total, config_, link, shifted + d2q9::w[out]);
        }
        return total;
    }

    [[nodiscard]] core::Result<double, std::string> mass() override {
        auto f = populations();
        if (!f) {
            return core::failure(std::string(f.error()));
        }
        const std::vector<double>& values = f.value();
        double total = 0.0;
        for (std::size_t c = 0; c < cells_; ++c) {
            if (mask_.solid[c] == 0) {
                for (std::size_t i = 0; i < q; ++i) {
                    total += values[i * cells_ + c];
                }
            }
        }
        return total;
    }

private:
    [[nodiscard]] Status read_float32(BufferId buffer, std::vector<double>& values) {
        std::vector<std::byte> bytes(values.size() * compute::float32_bytes);
        if (auto read = backend_->readback(buffer, 0, bytes); !read) {
            return core::failure(with_context("readback", read.error()));
        }
        if (!compute::unpack_float32(bytes, values)) {
            return core::failure(std::string("lbm2d gpu: readback size mismatch"));
        }
        return std::monostate{};
    }

    IComputeBackend* backend_;
    Lbm2DConfig config_;
    SolidMask2D mask_;
    std::size_t cells_ = 0;
    std::vector<BoundaryLink> links_;
    std::vector<Uniform> uniforms_;
    std::array<ScopedBuffer, 2> populations_;
    ScopedBuffer solid_;
    ScopedBuffer moments_;
    ScopedBuffer outlet_;
    ScopedBuffer link_buffer_;
    ScopedBuffer link_force_;
    std::size_t current_ = 0;
    std::uint64_t steps_ = 0;
};

} // namespace

core::Result<std::unique_ptr<Lbm2DSolver>, std::string>
make_gpu_solver(compute::IComputeBackend& backend, const Lbm2DConfig& config, SolidMask2D mask) {
    if (auto problem = validate(config, mask)) {
        return core::failure(std::move(*problem));
    }
    if (q * config.nx * config.ny > std::numeric_limits<std::uint32_t>::max()) {
        return core::failure(std::string("lbm2d gpu: lattice too large for 32-bit indices"));
    }
    if (auto problem = check_binary32(config)) {
        return core::failure(std::move(*problem));
    }
    auto solver = std::make_unique<GpuLbm2D>(backend, config, std::move(mask));
    if (auto ready = solver->allocate(); !ready) {
        return core::failure(std::string(ready.error()));
    }
    return std::unique_ptr<Lbm2DSolver>(std::move(solver));
}

} // namespace orrery::aero::lbm
