#include "orrery/render/aero_compute.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <glm/geometric.hpp>

#include "aero_internal.h"
#include "orrery/render/aero_streamlines.h"
#include "orrery/render/embedded_aero_shaders.h"

namespace orrery::render {

namespace {

constexpr std::string_view kKernelPrefix = "aero_vis_";

} // namespace

core::Result<std::unique_ptr<GlComputeBackend>, std::string> make_aero_compute_backend() {
    std::vector<ShaderSource> sources;
    for (const aero_shaders::EmbeddedShader* shader : aero_shaders::all) {
        if (shader->name.starts_with(kKernelPrefix)) {
            sources.push_back({shader->name, shader->source});
        }
    }
    return GlComputeBackend::create(sources);
}

namespace detail {

core::Result<aero::compute::ScopedBuffer, std::string>
upload_new(aero::compute::IComputeBackend& backend, std::span<const std::byte> bytes) {
    auto buffer = aero::compute::make_buffer(backend, std::max<std::size_t>(bytes.size(), 16));
    if (!buffer) {
        return core::failure(std::string(buffer.error()));
    }
    if (!bytes.empty()) {
        if (auto done = backend.upload(buffer.value().id(), 0, bytes); !done) {
            return core::failure(std::string(done.error()));
        }
    }
    return std::move(buffer).value();
}

PackedField pack_field(const VectorGrid& velocity, std::span<const std::uint8_t> solid) {
    PackedField packed;
    packed.velocity.reserve(velocity.values.size());
    for (const glm::dvec3& v : velocity.values) {
        packed.velocity.emplace_back(to_float(v.x), to_float(v.y), to_float(v.z), 0.0F);
    }
    packed.solid.assign((velocity.values.size() + 3) / 4, 0U);
    for (std::size_t n = 0; n < solid.size() && n < velocity.values.size(); ++n) {
        if (solid[n] != 0) {
            packed.solid[n / 4] |= 1U << (8U * static_cast<unsigned>(n % 4));
        }
    }
    packed.max_speed = to_float(max_fluid_speed(velocity, solid));
    return packed;
}

glm::vec4 to_index_space(const GridGeometry& grid, const glm::dvec3& world) {
    const glm::dvec3 p = (world - grid.origin) / grid.spacing;
    return {to_float(p.x), to_float(p.y), grid.is_2d() ? 0.0F : to_float(p.z), 0.0F};
}

glm::dvec3 from_index_space(const GridGeometry& grid, const glm::vec4& index) {
    return grid.origin +
           grid.spacing * glm::dvec3(static_cast<double>(index.x), static_cast<double>(index.y),
                                     grid.is_2d() ? 0.0 : static_cast<double>(index.z));
}

core::Result<FieldBuffers, std::string> upload_field(aero::compute::IComputeBackend& backend,
                                                     const VectorGrid& velocity,
                                                     std::span<const std::uint8_t> solid) {
    const PackedField packed = pack_field(velocity, solid);
    auto v = upload_new(backend, std::as_bytes(std::span<const glm::vec4>(packed.velocity)));
    if (!v) {
        return core::failure("velocity: " + v.error());
    }
    auto s = upload_new(backend, std::as_bytes(std::span<const std::uint32_t>(packed.solid)));
    if (!s) {
        return core::failure("solid flags: " + s.error());
    }
    return FieldBuffers{std::move(v).value(), std::move(s).value(), packed.max_speed};
}

} // namespace detail

core::Result<std::vector<std::optional<glm::dvec3>>, std::string>
advect_particles_gpu(aero::compute::IComputeBackend& backend, const VectorGrid& velocity,
                     std::span<const std::uint8_t> solid, std::span<const glm::dvec3> positions,
                     double dt) {
    using aero::compute::BufferBinding;
    using aero::compute::DispatchCommand;
    using aero::compute::Float32Uniform;
    using aero::compute::Uniform;
    std::vector<std::optional<glm::dvec3>> result;
    const std::size_t count = velocity.grid.node_count();
    if (validate(velocity.grid) || velocity.values.size() != count ||
        (!solid.empty() && solid.size() != count) || positions.empty()) {
        return result;
    }
    if (!backend.has_kernel(kernel_particles)) {
        return core::failure(std::string("particles: the backend has no kernel ") +
                             std::string(kernel_particles));
    }
    auto field = detail::upload_field(backend, velocity, solid);
    if (!field) {
        return core::failure("particles: " + field.error());
    }
    std::vector<glm::vec4> state;
    state.reserve(positions.size());
    for (const glm::dvec3& p : positions) {
        state.push_back(detail::to_index_space(velocity.grid, p));
    }
    auto particles = aero::compute::make_buffer(backend, state.size() * sizeof(glm::vec4));
    if (!particles) {
        return core::failure("particles: " + particles.error());
    }
    if (auto up = backend.upload(particles.value().id(), 0,
                                 std::as_bytes(std::span<const glm::vec4>(state)));
        !up) {
        return core::failure("particles: " + up.error());
    }
    auto seeds = aero::compute::make_buffer(backend, sizeof(glm::vec4));
    if (!seeds) {
        return core::failure("particles: " + seeds.error());
    }
    auto traces = aero::compute::make_buffer(backend, sizeof(glm::vec4));
    if (!traces) {
        return core::failure("particles: " + traces.error());
    }
    const std::array<Uniform, 13> uniforms{
        Uniform{"u_nx", static_cast<std::uint32_t>(velocity.grid.dims[0])},
        Uniform{"u_ny", static_cast<std::uint32_t>(velocity.grid.dims[1])},
        Uniform{"u_nz", static_cast<std::uint32_t>(velocity.grid.dims[2])},
        Uniform{"u_count", static_cast<std::uint32_t>(positions.size())},
        Uniform{"u_seed_count", std::uint32_t{0}},
        Uniform{"u_trace_points", std::uint32_t{0}},
        Uniform{"u_head", std::uint32_t{0}},
        Uniform{"u_release", std::uint32_t{0}},
        Uniform{"u_frame", std::uint32_t{0}},
        Uniform{"u_dt", Float32Uniform{dt}},
        Uniform{"u_dt_index", Float32Uniform{dt / velocity.grid.spacing}},
        Uniform{"u_lifetime", Float32Uniform{3.0e38}},
        Uniform{"u_jitter", Float32Uniform{0.0}},
    };
    const std::array<BufferBinding, 5> buffers{
        BufferBinding{0, field.value().velocity.id()}, BufferBinding{1, field.value().solid.id()},
        BufferBinding{2, seeds.value().id()},          BufferBinding{3, particles.value().id()},
        BufferBinding{4, traces.value().id()},
    };
    const DispatchCommand command{
        kernel_particles, {static_cast<std::uint32_t>(positions.size()), 1, 1}, uniforms, buffers};
    if (auto done = backend.dispatch(command); !done) {
        return core::failure("particles: " + done.error());
    }
    backend.barrier();
    if (auto read = backend.readback(particles.value().id(), 0,
                                     std::as_writable_bytes(std::span<glm::vec4>(state)));
        !read) {
        return core::failure("particles: " + read.error());
    }
    result.reserve(state.size());
    for (const glm::vec4& s : state) {
        if (s.w < 0.0F) {
            result.emplace_back(std::nullopt);
        } else {
            result.emplace_back(detail::from_index_space(velocity.grid, s));
        }
    }
    return result;
}

} // namespace orrery::render
