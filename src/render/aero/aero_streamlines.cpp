#include "orrery/render/aero_streamlines.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include <glm/geometric.hpp>

#include "aero_internal.h"
#include "orrery/render/aero_compute.h"

namespace orrery::render {

namespace {

bool usable(const VectorGrid& velocity, std::span<const std::uint8_t> solid) {
    const std::size_t count = velocity.grid.node_count();
    return !validate(velocity.grid) && velocity.values.size() == count &&
           (solid.empty() || solid.size() == count);
}

// True where the node nearest to `p` is solid (and false outside the grid).
bool in_solid(const GridGeometry& grid, std::span<const std::uint8_t> solid,
              const glm::dvec3& p) noexcept {
    if (solid.empty()) {
        return false;
    }
    std::array<std::size_t, 3> at{};
    for (std::size_t a = 0; a < 3; ++a) {
        if (grid.dims[a] < 2) {
            continue;
        }
        const auto axis = static_cast<glm::length_t>(a);
        const double f = std::round((p[axis] - grid.origin[axis]) / grid.spacing);
        at[a] = static_cast<std::size_t>(std::clamp(f, 0.0, static_cast<double>(grid.dims[a] - 1)));
    }
    return solid[grid.index(at[0], at[1], at[2])] != 0;
}

struct Tracer {
    const VectorGrid& velocity;
    std::span<const std::uint8_t> solid;
    double stagnation = 0.0;

    // Unit direction of the flow at p (times `sign`); empty where a line stops.
    [[nodiscard]] std::optional<glm::dvec3> direction(const glm::dvec3& p, double sign) const {
        if (in_solid(velocity.grid, solid, p)) {
            return std::nullopt;
        }
        const std::optional<glm::dvec3> u = sample(velocity, p);
        if (!u) {
            return std::nullopt;
        }
        const double speed = glm::length(*u);
        if (!(speed > stagnation)) {
            return std::nullopt;
        }
        return *u * (sign / speed);
    }

    // Classical RK4 step of arc length ds.
    [[nodiscard]] std::optional<glm::dvec3> step(const glm::dvec3& p, double ds,
                                                 double sign) const {
        const auto k1 = direction(p, sign);
        if (!k1) {
            return std::nullopt;
        }
        const auto k2 = direction(p + 0.5 * ds * *k1, sign);
        if (!k2) {
            return std::nullopt;
        }
        const auto k3 = direction(p + 0.5 * ds * *k2, sign);
        if (!k3) {
            return std::nullopt;
        }
        const auto k4 = direction(p + ds * *k3, sign);
        if (!k4) {
            return std::nullopt;
        }
        const glm::dvec3 next = p + (ds / 6.0) * (*k1 + 2.0 * *k2 + 2.0 * *k3 + *k4);
        if (!direction(next, sign)) {
            return std::nullopt;
        }
        return next;
    }

    // Points from `seed` along sign * u, the seed first, at most `limit`.
    [[nodiscard]] std::vector<glm::dvec3> run(const glm::dvec3& seed, double ds, double sign,
                                              std::size_t limit) const {
        std::vector<glm::dvec3> points;
        if (limit == 0 || !direction(seed, sign)) {
            return points;
        }
        points.reserve(limit);
        points.push_back(seed);
        while (points.size() < limit) {
            const std::optional<glm::dvec3> next = step(points.back(), ds, sign);
            if (!next) {
                break;
            }
            points.push_back(*next);
        }
        return points;
    }
};

Streamline with_speed(const VectorGrid& velocity, std::vector<glm::dvec3> points) {
    Streamline line;
    line.speed.reserve(points.size());
    for (const glm::dvec3& p : points) {
        line.speed.push_back(glm::length(sample(velocity, p).value_or(glm::dvec3(0.0))));
    }
    line.points = std::move(points);
    return line;
}

// Splits max_points between the two directions of a `both` trace.
std::array<std::size_t, 2> budgets(const StreamlineSettings& settings) {
    const std::size_t total = std::max<std::size_t>(settings.max_points, 2);
    switch (settings.direction) {
        case TraceDirection::forward:
            return {total, 0};
        case TraceDirection::backward:
            return {0, total};
        case TraceDirection::both:
            return {(total + 1) / 2, total / 2 + 1}; // the seed is shared
    }
    return {total, 0};
}

Streamline join(std::vector<glm::dvec3> backward, const std::vector<glm::dvec3>& forward) {
    // backward runs from the seed upstream; reverse it and drop the shared seed.
    std::ranges::reverse(backward);
    if (!backward.empty() && !forward.empty()) {
        backward.pop_back();
    }
    backward.insert(backward.end(), forward.begin(), forward.end());
    Streamline line;
    line.points = std::move(backward);
    return line;
}

} // namespace

namespace detail {

double max_fluid_speed(const VectorGrid& velocity, std::span<const std::uint8_t> solid) noexcept {
    double largest = 0.0;
    for (std::size_t n = 0; n < velocity.values.size(); ++n) {
        if (solid.empty() || solid[n] == 0) {
            largest = std::max(largest, glm::length(velocity.values[n]));
        }
    }
    return largest;
}

} // namespace detail

std::vector<Streamline> trace_streamlines(const VectorGrid& velocity,
                                          std::span<const std::uint8_t> solid,
                                          std::span<const glm::dvec3> seeds,
                                          const StreamlineSettings& settings) {
    std::vector<Streamline> lines;
    if (!usable(velocity, solid)) {
        return lines;
    }
    const Tracer tracer{velocity, solid,
                        settings.stagnation_fraction * detail::max_fluid_speed(velocity, solid)};
    const double ds = settings.step * velocity.grid.spacing;
    const std::array<std::size_t, 2> budget = budgets(settings);
    lines.reserve(seeds.size());
    for (const glm::dvec3& seed : seeds) {
        std::vector<glm::dvec3> forward = tracer.run(seed, ds, 1.0, budget[0]);
        std::vector<glm::dvec3> backward = tracer.run(seed, ds, -1.0, budget[1]);
        Streamline joined = join(std::move(backward), forward);
        lines.push_back(with_speed(velocity, std::move(joined.points)));
    }
    return lines;
}

core::Result<std::vector<Streamline>, std::string>
trace_streamlines_gpu(aero::compute::IComputeBackend& backend, const VectorGrid& velocity,
                      std::span<const std::uint8_t> solid, std::span<const glm::dvec3> seeds,
                      const StreamlineSettings& settings) {
    using aero::compute::BufferBinding;
    using aero::compute::DispatchCommand;
    using aero::compute::Float32Uniform;
    using aero::compute::Uniform;
    std::vector<Streamline> lines;
    if (!usable(velocity, solid) || seeds.empty()) {
        return lines;
    }
    if (!backend.has_kernel(kernel_streamlines)) {
        return core::failure(std::string("streamlines: the backend has no kernel ") +
                             std::string(kernel_streamlines));
    }
    auto field = detail::upload_field(backend, velocity, solid);
    if (!field) {
        return core::failure("streamlines: " + field.error());
    }
    std::vector<glm::vec4> seed_points;
    seed_points.reserve(seeds.size());
    for (const glm::dvec3& s : seeds) {
        seed_points.push_back(detail::to_index_space(velocity.grid, s));
    }
    auto seed_buffer =
        detail::upload_new(backend, std::as_bytes(std::span<const glm::vec4>(seed_points)));
    if (!seed_buffer) {
        return core::failure("streamlines: " + seed_buffer.error());
    }
    const std::array<std::size_t, 2> budget = budgets(settings);
    std::array<std::vector<std::vector<glm::dvec3>>, 2> runs;
    for (std::size_t d = 0; d < 2; ++d) {
        runs[d].resize(seeds.size());
        const std::size_t limit = budget[d];
        if (limit == 0) {
            continue;
        }
        auto points = aero::compute::make_buffer(backend, seeds.size() * limit * sizeof(glm::vec4));
        if (!points) {
            return core::failure("streamlines: " + points.error());
        }
        auto counts = aero::compute::make_buffer(backend, seeds.size() * sizeof(std::uint32_t));
        if (!counts) {
            return core::failure("streamlines: " + counts.error());
        }
        const std::array<Uniform, 8> uniforms{
            Uniform{"u_nx", static_cast<std::uint32_t>(velocity.grid.dims[0])},
            Uniform{"u_ny", static_cast<std::uint32_t>(velocity.grid.dims[1])},
            Uniform{"u_nz", static_cast<std::uint32_t>(velocity.grid.dims[2])},
            Uniform{"u_seed_count", static_cast<std::uint32_t>(seeds.size())},
            Uniform{"u_max_points", static_cast<std::uint32_t>(limit)},
            Uniform{"u_step", Float32Uniform{settings.step}},
            Uniform{"u_sign", Float32Uniform{d == 0 ? 1.0 : -1.0}},
            Uniform{"u_stagnation", Float32Uniform{settings.stagnation_fraction *
                                                   static_cast<double>(field.value().max_speed)}},
        };
        const std::array<BufferBinding, 5> buffers{
            BufferBinding{0, field.value().velocity.id()},
            BufferBinding{1, field.value().solid.id()},
            BufferBinding{2, seed_buffer.value().id()},
            BufferBinding{3, points.value().id()},
            BufferBinding{4, counts.value().id()},
        };
        const DispatchCommand command{kernel_streamlines,
                                      {static_cast<std::uint32_t>(seeds.size()), 1, 1},
                                      uniforms,
                                      buffers};
        if (auto done = backend.dispatch(command); !done) {
            return core::failure("streamlines: " + done.error());
        }
        backend.barrier();
        std::vector<std::uint32_t> count(seeds.size(), 0U);
        std::vector<glm::vec4> data(seeds.size() * limit);
        if (auto read = backend.readback(counts.value().id(), 0,
                                         std::as_writable_bytes(std::span<std::uint32_t>(count)));
            !read) {
            return core::failure("streamlines: " + read.error());
        }
        if (auto read = backend.readback(points.value().id(), 0,
                                         std::as_writable_bytes(std::span<glm::vec4>(data)));
            !read) {
            return core::failure("streamlines: " + read.error());
        }
        for (std::size_t s = 0; s < seeds.size(); ++s) {
            const std::size_t n = std::min<std::size_t>(count[s], limit);
            runs[d][s].reserve(n);
            for (std::size_t p = 0; p < n; ++p) {
                runs[d][s].push_back(detail::from_index_space(velocity.grid, data[s * limit + p]));
            }
        }
    }
    lines.reserve(seeds.size());
    for (std::size_t s = 0; s < seeds.size(); ++s) {
        Streamline joined = join(std::move(runs[1][s]), runs[0][s]);
        lines.push_back(with_speed(velocity, std::move(joined.points)));
    }
    return lines;
}

std::vector<glm::dvec3> seed_line(const glm::dvec3& a, const glm::dvec3& b, std::size_t count) {
    std::vector<glm::dvec3> seeds;
    seeds.reserve(count);
    if (count == 1) {
        seeds.push_back(0.5 * (a + b));
        return seeds;
    }
    for (std::size_t i = 0; i < count; ++i) {
        seeds.push_back(a + (b - a) * (static_cast<double>(i) / static_cast<double>(count - 1)));
    }
    return seeds;
}

std::vector<glm::dvec3> seed_plane(const GridGeometry& grid, Axis axis, double position,
                                   std::size_t count_u, std::size_t count_v) {
    std::vector<glm::dvec3> seeds;
    if (validate(grid) || count_u == 0 || count_v == 0) {
        return seeds;
    }
    const auto a = static_cast<std::size_t>(axis);
    const std::size_t u_axis = a == 0 ? 1 : 0;
    const std::size_t v_axis = a == 2 ? 1 : 2;
    const glm::dvec3 low = grid.origin;
    const glm::dvec3 high = grid.upper_corner();
    seeds.reserve(count_u * count_v);
    for (std::size_t v = 0; v < count_v; ++v) {
        for (std::size_t u = 0; u < count_u; ++u) {
            glm::dvec3 p{0.0};
            const auto gu = static_cast<glm::length_t>(u_axis);
            const auto gv = static_cast<glm::length_t>(v_axis);
            const auto ga = static_cast<glm::length_t>(a);
            p[gu] = low[gu] + (high[gu] - low[gu]) * (static_cast<double>(u) + 0.5) /
                                  static_cast<double>(count_u);
            p[gv] = low[gv] + (high[gv] - low[gv]) * (static_cast<double>(v) + 0.5) /
                                  static_cast<double>(count_v);
            p[ga] = std::clamp(position, low[ga], high[ga]);
            seeds.push_back(p);
        }
    }
    return seeds;
}

std::optional<glm::dvec3> advect_particle(const VectorGrid& velocity,
                                          std::span<const std::uint8_t> solid,
                                          const glm::dvec3& position, double dt) {
    if (!usable(velocity, solid) || in_solid(velocity.grid, solid, position)) {
        return std::nullopt;
    }
    const std::optional<glm::dvec3> u0 = sample(velocity, position);
    if (!u0) {
        return std::nullopt;
    }
    const glm::dvec3 middle = position + 0.5 * dt * *u0;
    if (in_solid(velocity.grid, solid, middle)) {
        return std::nullopt;
    }
    const std::optional<glm::dvec3> u1 = sample(velocity, middle);
    if (!u1) {
        return std::nullopt;
    }
    const glm::dvec3 next = position + dt * *u1;
    if (!sample(velocity, next) || in_solid(velocity.grid, solid, next)) {
        return std::nullopt;
    }
    return next;
}

} // namespace orrery::render
