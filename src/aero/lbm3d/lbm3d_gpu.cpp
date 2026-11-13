// GPU D3Q19 solver: the step of lbm3d_cpu.cpp run by the compute kernels
// lbm_d3q19_step, lbm_d3q19_moments and lbm_d3q19_force through an
// IComputeBackend. Device buffers hold binary32 values; populations are
// stored shifted by their lattice weight (f_i - w_i), which keeps the small
// deviations that carry the flow out of the rounding error of the O(1)
// weights (Skordos 1993; docs/lbm.md, "GPU precision"). Every host-side sum
// (force, mass) is done in double after readback.
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include "lbm3d_internal.h"
#include "orrery/aero/compute/float32_pack.h"
#include "orrery/aero/lbm3d/lbm3d.h"

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
using d3q19::q;

// Binding points of the kernels (src/render/shaders/lbm/lbm_d3q19_*.glsl).
constexpr std::uint32_t kBindSource = 0;
constexpr std::uint32_t kBindClass = 1;
constexpr std::uint32_t kBindDestination = 2;
constexpr std::uint32_t kBindMoments = 3;
constexpr std::uint32_t kBindLinks = 4;
constexpr std::uint32_t kBindLinkForce = 5;
constexpr std::uint32_t kBindOutlet = 6;
constexpr std::uint32_t kBindWall = 7;

constexpr std::size_t kOutletSlots = detail3d::outlet_directions.size();

// The force kernel runs one work item per link in rows of kLinkRow items
// (1024 groups of its 64), link k = x + kLinkRow y: no dimension needs more
// than the 65535 groups every device supports (GL_MAX_COMPUTE_WORK_GROUP_COUNT
// minimum); 2^32 links would take 65536 rows, and the 32-bit cell index
// bound of make_gpu_solver keeps the link count (at most 18 per cell) below
// that.
constexpr std::uint32_t kLinkRow = 65536;

DispatchSize link_dispatch(std::size_t links) noexcept {
    if (links <= kLinkRow) {
        return DispatchSize{static_cast<std::uint32_t>(links), 1, 1};
    }
    const std::size_t rows = (links + kLinkRow - 1) / kLinkRow;
    return DispatchSize{kLinkRow, static_cast<std::uint32_t>(rows), 1};
}

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
    return std::string("lbm3d gpu: ") + what + ": " + error;
}

std::string megabytes(std::size_t bytes) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(1) << static_cast<double>(bytes) / (1024.0 * 1024.0)
        << " MiB";
    return out.str();
}

// The kernels see every real parameter rounded to binary32: each must stay
// finite, and tau must stay above 1/2 (a tau that rounds to 1/2 is zero
// viscosity on the GPU while the CPU keeps a positive one).
std::optional<std::string> check_binary32(const Lbm3DConfig& config) {
    const double c = config.smagorinsky.enabled ? config.smagorinsky.constant : 0.0;
    struct Parameter {
        const char* name;
        double value;
    };
    for (const Parameter p :
         {Parameter{"tau", config.tau}, Parameter{"y = 0 wall speed", config.y_low.velocity_x},
          Parameter{"y = ny - 1 wall speed", config.y_high.velocity_x},
          Parameter{"z = 0 wall speed", config.z_low.velocity_x},
          Parameter{"z = nz - 1 wall speed", config.z_high.velocity_x},
          Parameter{"inlet x velocity", config.inlet_ux},
          Parameter{"inlet y velocity", config.inlet_uy},
          Parameter{"inlet z velocity", config.inlet_uz},
          Parameter{"squared Smagorinsky constant", c * c},
          Parameter{"body force x", config.body_force_x},
          Parameter{"body force y", config.body_force_y},
          Parameter{"body force z", config.body_force_z}}) {
        if (!std::isfinite(compute::round_to_float32(p.value))) {
            std::ostringstream message;
            message << "lbm3d gpu: " << p.name << " " << p.value
                    << " is outside the binary32 range of the kernels";
            return message.str();
        }
    }
    if (!(compute::round_to_float32(config.tau) > 0.5)) {
        std::ostringstream message;
        message << std::setprecision(17) << "lbm3d gpu: tau " << config.tau
                << " rounds to 1/2 in binary32 (zero viscosity in the kernels); tau - 1/2 must "
                   "exceed 2^-25 (about 3e-8)";
        return message.str();
    }
    return std::nullopt;
}

// The device must hold the lattice: one population buffer within the largest
// binding, everything within the free memory, when the backend reports them.
std::optional<std::string> check_memory(const IComputeBackend& backend, const Lbm3DConfig& config,
                                        std::size_t links, std::size_t wall_slots) {
    const std::optional<std::size_t> buffer = gpu_population_buffer_bytes(config);
    const std::optional<std::size_t> total = gpu_memory_bytes(config, links, wall_slots);
    if (!buffer || !total) {
        return std::string("lbm3d gpu: the device memory of the lattice overflows std::size_t");
    }
    const compute::DeviceMemory memory = backend.memory();
    if (memory.max_binding_bytes && *buffer > *memory.max_binding_bytes) {
        return "lbm3d gpu: a population buffer of " + megabytes(*buffer) +
               " exceeds the largest buffer the device can bind (" +
               megabytes(*memory.max_binding_bytes) + "); use fewer cells";
    }
    if (memory.available_bytes && *total > *memory.available_bytes) {
        return "lbm3d gpu: the lattice needs " + megabytes(*total) + " of device memory, " +
               megabytes(*memory.available_bytes) + " are available; use fewer cells";
    }
    return std::nullopt;
}

class GpuLbm3D final : public Lbm3DSolver {
public:
    GpuLbm3D(IComputeBackend& backend, const Lbm3DConfig& config, SolidMask3D mask,
             std::vector<BoundaryLink3D> links)
        : backend_(&backend), config_(config), mask_(std::move(mask)), cells_(mask_.solid.size()),
          links_(std::move(links)) {
        const double c = config_.smagorinsky.enabled ? config_.smagorinsky.constant : 0.0;
        uniforms_ = {
            Uniform{"u_nx", static_cast<std::uint32_t>(config_.nx)},
            Uniform{"u_ny", static_cast<std::uint32_t>(config_.ny)},
            Uniform{"u_nz", static_cast<std::uint32_t>(config_.nz)},
            Uniform{"u_x_mode",
                    static_cast<std::uint32_t>(config_.x_boundary == XBoundary::inflow_outflow)},
            Uniform{"u_y_low_kind", wall_code(config_.y_low.kind)},
            Uniform{"u_y_high_kind", wall_code(config_.y_high.kind)},
            Uniform{"u_z_low_kind", wall_code(config_.z_low.kind)},
            Uniform{"u_z_high_kind", wall_code(config_.z_high.kind)},
            Uniform{"u_y_low_ux", Float32Uniform{config_.y_low.velocity_x}},
            Uniform{"u_y_high_ux", Float32Uniform{config_.y_high.velocity_x}},
            Uniform{"u_z_low_ux", Float32Uniform{config_.z_low.velocity_x}},
            Uniform{"u_z_high_ux", Float32Uniform{config_.z_high.velocity_x}},
            Uniform{"u_inlet_ux", Float32Uniform{config_.inlet_ux}},
            Uniform{"u_inlet_uy", Float32Uniform{config_.inlet_uy}},
            Uniform{"u_inlet_uz", Float32Uniform{config_.inlet_uz}},
            Uniform{"u_tau", Float32Uniform{config_.tau}},
            Uniform{"u_smagorinsky_c2", Float32Uniform{c * c}},
            Uniform{"u_force_x", Float32Uniform{config_.body_force_x}},
            Uniform{"u_force_y", Float32Uniform{config_.body_force_y}},
            Uniform{"u_force_z", Float32Uniform{config_.body_force_z}},
            Uniform{"u_link_count", static_cast<std::uint32_t>(links_.size())},
            Uniform{"u_link_row", kLinkRow},
            Uniform{"u_outlet_mode",
                    static_cast<std::uint32_t>(config_.outlet == Outlet::extrapolation)},
            Uniform{"u_outlet_speed", Float32Uniform{config_.inlet_ux}},
            Uniform{"u_sponge_strength", Float32Uniform{config_.sponge.strength}},
            Uniform{"u_sponge_inlet", static_cast<std::uint32_t>(config_.sponge.inlet)},
            Uniform{"u_sponge_outlet", static_cast<std::uint32_t>(config_.sponge.outlet)},
            Uniform{"u_sponge_sides", static_cast<std::uint32_t>(config_.sponge.sides)},
        };
    }

    // Allocates and fills the device buffers; `walls` is the wall table of
    // the links (make_gpu_solver counted it in the memory check).
    [[nodiscard]] Status allocate(const detail3d::WallTable& walls) {
        const std::size_t population_bytes = q * cells_ * compute::float32_bytes;
        for (ScopedBuffer& buffer : populations_) {
            auto made = compute::make_buffer(*backend_, population_bytes);
            if (!made) {
                return core::failure(with_context("population buffer", made.error()));
            }
            buffer = std::move(made).value();
        }
        auto classes = compute::make_buffer(*backend_, cells_ * sizeof(std::uint32_t));
        if (!classes) {
            return core::failure(with_context("node class buffer", classes.error()));
        }
        classes_ = std::move(classes).value();
        {
            // Node classes, and 3 + slot for the nodes with wall fractions.
            const std::vector<std::uint8_t> node_class = detail3d::classify_nodes(config_, mask_);
            std::vector<std::uint32_t> codes;
            codes.reserve(cells_);
            for (std::size_t c = 0; c < cells_; ++c) {
                const bool slotted = !walls.slot.empty() && walls.slot[c] != detail3d::kNoSlot;
                codes.push_back(slotted ? 3U + walls.slot[c] : std::uint32_t{node_class[c]});
            }
            if (auto up = backend_->upload(classes_.id(), 0, bytes_of(codes)); !up) {
                return core::failure(with_context("node class upload", up.error()));
            }
            std::vector<double> fractions = walls.q;
            if (fractions.empty()) {
                fractions.assign(q, 0.5); // bound but never read by the halfway wall
            }
            auto wall = compute::make_buffer(*backend_, fractions.size() * compute::float32_bytes);
            if (!wall) {
                return core::failure(with_context("wall buffer", wall.error()));
            }
            wall_ = std::move(wall).value();
            if (auto up = backend_->upload(wall_.id(), 0, compute::pack_float32(fractions)); !up) {
                return core::failure(with_context("wall upload", up.error()));
            }
        }
        auto moments = compute::make_buffer(*backend_, 4 * cells_ * compute::float32_bytes);
        if (!moments) {
            return core::failure(with_context("moment buffer", moments.error()));
        }
        moments_ = std::move(moments).value();
        auto outlet = compute::make_buffer(*backend_, kOutletSlots * config_.ny * config_.nz *
                                                          compute::float32_bytes);
        if (!outlet) {
            return core::failure(with_context("outlet buffer", outlet.error()));
        }
        outlet_ = std::move(outlet).value();
        if (!links_.empty()) {
            std::vector<std::uint32_t> packed;
            packed.reserve(2 * links_.size());
            for (const BoundaryLink3D& link : links_) {
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
            auto force = compute::make_buffer(*backend_, links_.size() * compute::float32_bytes);
            if (!force) {
                return core::failure(with_context("link force buffer", force.error()));
            }
            link_force_ = std::move(force).value();
        }
        return initialize(uniform_inlet_state(config_));
    }

    [[nodiscard]] Status set_inlet_velocity(const std::array<double, 3>& velocity) override {
        if (auto problem = detail3d::check_inlet_velocity(config_, velocity)) {
            return core::failure(std::move(*problem));
        }
        apply_inlet(velocity);
        return std::monostate{};
    }
    [[nodiscard]] std::array<double, 3> inlet_velocity() const noexcept override { return inlet_; }

    // The inlet velocity of the kernels, unchecked.
    void apply_inlet(const std::array<double, 3>& velocity) {
        inlet_ = velocity;
        for (Uniform& uniform : uniforms_) {
            if (uniform.name == "u_inlet_ux") {
                uniform.value = Float32Uniform{velocity[0]};
            } else if (uniform.name == "u_inlet_uy") {
                uniform.value = Float32Uniform{velocity[1]};
            } else if (uniform.name == "u_inlet_uz") {
                uniform.value = Float32Uniform{velocity[2]};
            }
        }
    }

    [[nodiscard]] const Lbm3DConfig& config() const noexcept override { return config_; }
    [[nodiscard]] const SolidMask3D& mask() const noexcept override { return mask_; }
    [[nodiscard]] std::uint64_t steps() const noexcept override { return steps_; }
    [[nodiscard]] const std::vector<BoundaryLink3D>& links() const noexcept override {
        return links_;
    }

    [[nodiscard]] Status initialize(const InitialState3D& initial) override {
        std::vector<double> f = initial_populations(config_, mask_, initial);
        for (std::size_t i = 0; i < q; ++i) {
            for (std::size_t c = 0; c < cells_; ++c) {
                f[i * cells_ + c] -= d3q19::w[i];
            }
        }
        const std::vector<std::byte> packed = compute::pack_float32(f);
        for (const ScopedBuffer& buffer : populations_) {
            if (auto up = backend_->upload(buffer.id(), 0, packed); !up) {
                return core::failure(with_context("population upload", up.error()));
            }
        }
        // History of the convective outlet: the populations entering the
        // outlet face, slot-major (slot * ny * nz + y + ny z), shifted like f.
        const std::size_t face = config_.ny * config_.nz;
        std::vector<double> history(kOutletSlots * face);
        for (std::size_t z = 0; z < config_.nz; ++z) {
            for (std::size_t y = 0; y < config_.ny; ++y) {
                const std::size_t cell = mask_.index(config_.nx - 1, y, z);
                for (const std::size_t i : detail3d::outlet_directions) {
                    history[detail3d::outlet_slot(i) * face + y + config_.ny * z] =
                        f[i * cells_ + cell];
                }
            }
        }
        if (auto up = backend_->upload(outlet_.id(), 0, compute::pack_float32(history)); !up) {
            return core::failure(with_context("outlet upload", up.error()));
        }
        current_ = 0;
        steps_ = 0;
        apply_inlet({config_.inlet_ux, config_.inlet_uy, config_.inlet_uz});
        return std::monostate{};
    }

    [[nodiscard]] Status step(std::size_t count) override {
        const DispatchSize size{static_cast<std::uint32_t>(config_.nx),
                                static_cast<std::uint32_t>(config_.ny),
                                static_cast<std::uint32_t>(config_.nz)};
        for (std::size_t n = 0; n < count; ++n) {
            const std::array<BufferBinding, 5> bindings{
                BufferBinding{kBindSource, populations_[current_].id()},
                BufferBinding{kBindClass, classes_.id()},
                BufferBinding{kBindDestination, populations_[1 - current_].id()},
                BufferBinding{kBindOutlet, outlet_.id()},
                BufferBinding{kBindWall, wall_.id()},
            };
            const DispatchCommand command{kernel3d_step, size, uniforms_, bindings};
            if (auto done = backend_->dispatch(command); !done) {
                return core::failure(with_context("step", done.error()));
            }
            backend_->barrier();
            current_ = 1 - current_;
            ++steps_;
        }
        return std::monostate{};
    }

    [[nodiscard]] core::Result<MacroFields3D, std::string> macroscopic() override {
        const std::array<BufferBinding, 3> bindings{
            BufferBinding{kBindSource, populations_[current_].id()},
            BufferBinding{kBindClass, classes_.id()},
            BufferBinding{kBindMoments, moments_.id()},
        };
        const DispatchSize size{static_cast<std::uint32_t>(config_.nx),
                                static_cast<std::uint32_t>(config_.ny),
                                static_cast<std::uint32_t>(config_.nz)};
        if (auto done = backend_->dispatch({kernel3d_moments, size, uniforms_, bindings}); !done) {
            return core::failure(with_context("moments", done.error()));
        }
        backend_->barrier();
        std::vector<double> values(4 * cells_);
        if (auto read = read_float32(moments_.id(), values); !read) {
            return core::failure(std::string(read.error()));
        }
        MacroFields3D out;
        out.nx = config_.nx;
        out.ny = config_.ny;
        out.nz = config_.nz;
        const auto plane = [&](std::size_t k) {
            return std::vector<double>(values.begin() + static_cast<std::ptrdiff_t>(k * cells_),
                                       values.begin() +
                                           static_cast<std::ptrdiff_t>((k + 1) * cells_));
        };
        out.density = plane(0);
        for (double& rho : out.density) {
            rho += 1.0; // the kernel writes rho - 1
        }
        out.velocity_x = plane(1);
        out.velocity_y = plane(2);
        out.velocity_z = plane(3);
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
                f[i * cells_ + c] += d3q19::w[i];
            }
        }
        return f;
    }

    [[nodiscard]] core::Result<Force3D, std::string> force() override {
        if (links_.empty()) {
            return Force3D{};
        }
        const std::array<BufferBinding, 5> bindings{
            BufferBinding{kBindSource, populations_[current_].id()},
            BufferBinding{kBindClass, classes_.id()},
            BufferBinding{kBindLinks, link_buffer_.id()},
            BufferBinding{kBindLinkForce, link_force_.id()},
            BufferBinding{kBindWall, wall_.id()},
        };
        const DispatchSize size = link_dispatch(links_.size());
        if (auto done = backend_->dispatch({kernel3d_force, size, uniforms_, bindings}); !done) {
            return core::failure(with_context("force", done.error()));
        }
        backend_->barrier();
        std::vector<double> exchanged(links_.size());
        if (auto read = read_float32(link_force_.id(), exchanged); !read) {
            return core::failure(std::string(read.error()));
        }
        // The kernel writes the shifted sum fs_o + fs_j of every link; the
        // weights (w_j = w_o) are added back and the links summed in double.
        Force3D total;
        for (std::size_t k = 0; k < links_.size(); ++k) {
            const BoundaryLink3D& link = links_[k];
            const std::size_t out = d3q19::opposite[link.direction];
            add_link_force(total, config_, link, exchanged[k] + 2.0 * d3q19::w[out]);
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
            return core::failure(std::string("lbm3d gpu: readback size mismatch"));
        }
        return std::monostate{};
    }

    IComputeBackend* backend_;
    Lbm3DConfig config_;
    SolidMask3D mask_;
    std::size_t cells_ = 0;
    std::vector<BoundaryLink3D> links_;
    std::vector<Uniform> uniforms_;
    std::array<ScopedBuffer, 2> populations_;
    ScopedBuffer classes_;
    ScopedBuffer moments_;
    ScopedBuffer outlet_;
    ScopedBuffer link_buffer_;
    ScopedBuffer link_force_;
    ScopedBuffer wall_;
    std::array<double, 3> inlet_{config_.inlet_ux, config_.inlet_uy, config_.inlet_uz};
    std::size_t current_ = 0;
    std::uint64_t steps_ = 0;
};

} // namespace

core::Result<std::unique_ptr<Lbm3DSolver>, std::string>
make_gpu_solver(compute::IComputeBackend& backend, const Lbm3DConfig& config, SolidMask3D mask) {
    if (auto problem = validate(config, mask)) {
        return core::failure(std::move(*problem));
    }
    // Every index the kernels form (population i * cells + cell, outlet
    // entries) must fit in 32 bits.
    const std::optional<std::size_t> cells = lattice_cells_3d(config.nx, config.ny, config.nz);
    if (!cells || *cells > std::numeric_limits<std::uint32_t>::max() / q) {
        return core::failure(std::string("lbm3d gpu: lattice too large for 32-bit indices"));
    }
    if (auto problem = check_binary32(config)) {
        return core::failure(std::move(*problem));
    }
    // The boundary tables are sized by the geometry: built on the host
    // first, so the memory check counts every buffer the solver creates.
    std::vector<BoundaryLink3D> links = boundary_links(config, mask);
    const detail3d::WallTable walls = detail3d::wall_table(config, mask, links);
    if (auto problem = check_memory(backend, config, links.size(), walls.q.size() / q)) {
        return core::failure(std::move(*problem));
    }
    auto solver = std::make_unique<GpuLbm3D>(backend, config, std::move(mask), std::move(links));
    if (auto ready = solver->allocate(walls); !ready) {
        return core::failure(std::string(ready.error()));
    }
    return std::unique_ptr<Lbm3DSolver>(std::move(solver));
}

} // namespace orrery::aero::lbm
