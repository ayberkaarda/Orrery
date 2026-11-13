#include "orrery/render/aero_field.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

#include <glm/geometric.hpp>

namespace orrery::render {

namespace {

constexpr double kLatticeCs2 = 1.0 / 3.0;

glm::dvec3 to_glm(const geom::Vector3& v) noexcept {
    return {v.x, v.y, v.z};
}

bool finite(const glm::dvec3& v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

std::size_t axis_index(Axis axis) noexcept {
    return static_cast<std::size_t>(axis);
}

bool is_solid(std::span<const std::uint8_t> solid, std::size_t node) noexcept {
    return !solid.empty() && solid[node] != 0;
}

// APPROXIMATION (V6): derived quantities - finite differences of the nodal
// velocity, central (second order) inside the fluid, one-sided (first order)
// at grid edges and next to bodies - vorticity and Q near walls are smeared
// over one cell; display only.
// Derivative of `value(node)` along `axis` at node (i, j, k): central where
// both neighbours are fluid nodes of the grid, one-sided where only one is,
// zero otherwise (and along z on a 2D grid).
template <typename Value>
double derivative(const GridGeometry& grid, std::span<const std::uint8_t> solid,
                  const std::array<std::size_t, 3>& at, std::size_t axis, const Value& value) {
    const std::size_t n = grid.dims[axis];
    if (n < 2) {
        return 0.0;
    }
    std::array<std::size_t, 3> minus = at;
    std::array<std::size_t, 3> plus = at;
    const bool has_minus = at[axis] > 0;
    const bool has_plus = at[axis] + 1 < n;
    if (has_minus) {
        --minus[axis];
    }
    if (has_plus) {
        ++plus[axis];
    }
    const std::size_t centre = grid.index(at[0], at[1], at[2]);
    const std::size_t lo = grid.index(minus[0], minus[1], minus[2]);
    const std::size_t hi = grid.index(plus[0], plus[1], plus[2]);
    const bool lo_ok = has_minus && !is_solid(solid, lo);
    const bool hi_ok = has_plus && !is_solid(solid, hi);
    const double h = grid.spacing;
    if (lo_ok && hi_ok) {
        return (value(hi) - value(lo)) / (2.0 * h);
    }
    if (hi_ok) {
        return (value(hi) - value(centre)) / h;
    }
    if (lo_ok) {
        return (value(centre) - value(lo)) / h;
    }
    return 0.0;
}

// grad u at a node: g[r][c] = d u_r / d x_c.
using Gradient = std::array<std::array<double, 3>, 3>;

Gradient velocity_gradient(const VectorGrid& velocity, std::span<const std::uint8_t> solid,
                           const std::array<std::size_t, 3>& at) {
    Gradient g{};
    for (std::size_t c = 0; c < 3; ++c) {
        for (std::size_t r = 0; r < 3; ++r) {
            g[r][c] = derivative(velocity.grid, solid, at, c, [&](std::size_t node) {
                return velocity.values[node][static_cast<glm::length_t>(r)];
            });
        }
    }
    return g;
}

template <typename Visit> void for_each_node(const GridGeometry& grid, const Visit& visit) {
    for (std::size_t k = 0; k < grid.dims[2]; ++k) {
        for (std::size_t j = 0; j < grid.dims[1]; ++j) {
            for (std::size_t i = 0; i < grid.dims[0]; ++i) {
                visit(std::array<std::size_t, 3>{i, j, k}, grid.index(i, j, k));
            }
        }
    }
}

// APPROXIMATION (V1): field interpolation - trilinear (bilinear in 2D)
// between grid nodes - exact for linear fields, second order otherwise;
// structures below two grid spacings are not resolved.
// Cell and weights of a point for interpolation; empty outside the grid.
struct Stencil {
    std::array<std::size_t, 3> cell{};
    std::array<double, 3> weight{};
};

std::optional<Stencil> stencil(const GridGeometry& grid, const glm::dvec3& point) noexcept {
    constexpr double kTolerance = 1.0e-9;
    Stencil s;
    for (std::size_t a = 0; a < 3; ++a) {
        if (grid.dims[a] < 2) {
            s.cell[a] = 0;
            s.weight[a] = 0.0;
            continue;
        }
        const auto axis = static_cast<glm::length_t>(a);
        const double f = (point[axis] - grid.origin[axis]) / grid.spacing;
        const auto last = static_cast<double>(grid.dims[a] - 1);
        if (!(f >= -kTolerance && f <= last + kTolerance)) {
            return std::nullopt;
        }
        const double clamped = std::clamp(f, 0.0, last);
        const double base = std::min(std::floor(clamped), last - 1.0);
        s.cell[a] = static_cast<std::size_t>(base);
        s.weight[a] = clamped - base;
    }
    return s;
}

template <typename T, typename Values>
T interpolate(const GridGeometry& grid, const Stencil& s, const Values& values) {
    T sum{};
    const std::size_t kz = grid.dims[2] < 2 ? 1 : 2;
    for (std::size_t dz = 0; dz < kz; ++dz) {
        const double wz = kz == 1 ? 1.0 : (dz == 0 ? 1.0 - s.weight[2] : s.weight[2]);
        for (std::size_t dy = 0; dy < 2; ++dy) {
            const double wy = dy == 0 ? 1.0 - s.weight[1] : s.weight[1];
            for (std::size_t dx = 0; dx < 2; ++dx) {
                const double wx = dx == 0 ? 1.0 - s.weight[0] : s.weight[0];
                const double w = wx * wy * wz;
                if (w != 0.0) {
                    sum += values[grid.index(s.cell[0] + dx, s.cell[1] + dy, s.cell[2] + dz)] * w;
                }
            }
        }
    }
    return sum;
}

} // namespace

std::size_t GridGeometry::node_count() const noexcept {
    constexpr std::size_t kMax = std::numeric_limits<std::size_t>::max();
    std::size_t count = 1;
    for (const std::size_t d : dims) {
        if (d != 0 && count > kMax / d) {
            return 0;
        }
        count *= d;
    }
    return count;
}

glm::dvec3 GridGeometry::node_position(std::size_t i, std::size_t j, std::size_t k) const noexcept {
    return origin + spacing * glm::dvec3(static_cast<double>(i), static_cast<double>(j),
                                         static_cast<double>(k));
}

glm::dvec3 GridGeometry::upper_corner() const noexcept {
    const auto last = [](std::size_t d) { return d > 0 ? static_cast<double>(d - 1) : 0.0; };
    return origin + spacing * glm::dvec3(last(dims[0]), last(dims[1]), last(dims[2]));
}

std::optional<std::string> validate(const GridGeometry& grid) {
    if (grid.dims[0] < 2 || grid.dims[1] < 2 || grid.dims[2] < 1) {
        return std::string("flow grid: needs at least 2 nodes along x and y");
    }
    const std::size_t count = grid.node_count();
    if (count == 0 || count > kMaxFieldNodes) {
        return "flow grid: more than " + std::to_string(kMaxFieldNodes) + " nodes";
    }
    if (!(grid.spacing > 0.0) || !std::isfinite(grid.spacing)) {
        return std::string("flow grid: spacing must be positive and finite");
    }
    if (!finite(grid.origin) || !finite(grid.upper_corner())) {
        return std::string("flow grid: origin and extent must be finite");
    }
    return std::nullopt;
}

std::optional<std::string> validate(const FlowField& field) {
    if (auto bad = validate(field.velocity.grid)) {
        return bad;
    }
    const std::size_t count = field.velocity.grid.node_count();
    if (field.velocity.values.size() != count) {
        return std::string("flow field: velocity count differs from the grid nodes");
    }
    if (!std::ranges::all_of(field.velocity.values,
                             [](const glm::dvec3& v) { return finite(v); })) {
        return std::string("flow field: velocity is not finite");
    }
    if (!field.solid.empty() && field.solid.size() != count) {
        return std::string("flow field: solid flag count differs from the grid nodes");
    }
    if (field.pressure) {
        const GridGeometry& p = field.pressure->grid;
        const GridGeometry& v = field.velocity.grid;
        if (p.dims != v.dims || p.origin != v.origin || p.spacing != v.spacing) {
            return std::string("flow field: pressure lies on a different grid");
        }
        if (field.pressure->values.size() != count) {
            return std::string("flow field: pressure count differs from the grid nodes");
        }
        if (!std::ranges::all_of(field.pressure->values,
                                 [](double x) { return std::isfinite(x); })) {
            return std::string("flow field: pressure is not finite");
        }
    }
    return std::nullopt;
}

FlowField flow_field_from(const aero::VelocityField& velocity) {
    FlowField field;
    field.velocity.grid.dims = velocity.dims;
    field.velocity.grid.origin = to_glm(velocity.origin);
    field.velocity.grid.spacing = velocity.spacing;
    field.velocity.values.reserve(velocity.values.size());
    for (const geom::Vector3& v : velocity.values) {
        field.velocity.values.push_back(to_glm(v));
    }
    return field;
}

namespace {

// APPROXIMATION (V10): lattice pressure - the weakly compressible LBM
// pressure c_s^2 (rho - 1) against rho_ref = 1, as Cp with the inlet speed -
// valid at lattice Mach below 0.1 (A3); includes the acoustic noise of the
// solver.
ScalarGrid lattice_pressure(const GridGeometry& grid, const std::vector<double>& density,
                            const LatticeMapping& mapping, std::string& label) {
    ScalarGrid p;
    p.grid = grid;
    const double u = mapping.reference_speed;
    const double scale = u > 0.0 ? kLatticeCs2 / (0.5 * u * u) : kLatticeCs2;
    label = u > 0.0 ? "Cp" : "p (lattice)";
    p.values.reserve(density.size());
    for (const double rho : density) {
        p.values.push_back(scale * (rho - 1.0));
    }
    return p;
}

} // namespace

FlowField flow_field_from(const aero::lbm::MacroFields2D& fields, const LatticeMapping& mapping) {
    FlowField field;
    GridGeometry& grid = field.velocity.grid;
    grid.dims = {fields.nx, fields.ny, 1};
    grid.origin = mapping.origin;
    grid.spacing = mapping.spacing;
    const std::size_t count = std::min(fields.velocity_x.size(), fields.velocity_y.size());
    field.velocity.values.reserve(count);
    for (std::size_t n = 0; n < count; ++n) {
        field.velocity.values.emplace_back(mapping.velocity_scale * fields.velocity_x[n],
                                           mapping.velocity_scale * fields.velocity_y[n], 0.0);
    }
    field.pressure = lattice_pressure(grid, fields.density, mapping, field.pressure_label);
    field.solid = fields.solid;
    return field;
}

FlowField flow_field_from(const aero::lbm::MacroFields3D& fields, const LatticeMapping& mapping) {
    FlowField field;
    GridGeometry& grid = field.velocity.grid;
    grid.dims = {fields.nx, fields.ny, fields.nz};
    grid.origin = mapping.origin;
    grid.spacing = mapping.spacing;
    const std::size_t count =
        std::min({fields.velocity_x.size(), fields.velocity_y.size(), fields.velocity_z.size()});
    field.velocity.values.reserve(count);
    for (std::size_t n = 0; n < count; ++n) {
        field.velocity.values.emplace_back(mapping.velocity_scale * fields.velocity_x[n],
                                           mapping.velocity_scale * fields.velocity_y[n],
                                           mapping.velocity_scale * fields.velocity_z[n]);
    }
    field.pressure = lattice_pressure(grid, fields.density, mapping, field.pressure_label);
    field.solid = fields.solid;
    return field;
}

std::string quantity_label(FieldQuantity quantity, std::string_view pressure_label) {
    switch (quantity) {
        case FieldQuantity::speed:
            return "|u| [m/s]";
        case FieldQuantity::vorticity:
            return "omega [1/s]";
        case FieldQuantity::vorticity_magnitude:
            return "|omega| [1/s]";
        case FieldQuantity::pressure:
            return std::string(pressure_label);
        case FieldQuantity::q_criterion:
            return "Q [1/s2]";
    }
    return {};
}

bool is_signed(FieldQuantity quantity) noexcept {
    return quantity == FieldQuantity::vorticity || quantity == FieldQuantity::pressure;
}

VectorGrid vorticity(const VectorGrid& velocity, std::span<const std::uint8_t> solid) {
    VectorGrid result;
    result.grid = velocity.grid;
    result.values.assign(velocity.values.size(), glm::dvec3(0.0));
    if (validate(velocity.grid) || velocity.values.size() != velocity.grid.node_count()) {
        return result;
    }
    for_each_node(velocity.grid, [&](const std::array<std::size_t, 3>& at, std::size_t node) {
        if (is_solid(solid, node)) {
            return;
        }
        const Gradient g = velocity_gradient(velocity, solid, at);
        result.values[node] = {g[2][1] - g[1][2], g[0][2] - g[2][0], g[1][0] - g[0][1]};
    });
    return result;
}

ScalarGrid q_criterion(const VectorGrid& velocity, std::span<const std::uint8_t> solid) {
    ScalarGrid result;
    result.grid = velocity.grid;
    result.values.assign(velocity.values.size(), 0.0);
    if (validate(velocity.grid) || velocity.values.size() != velocity.grid.node_count()) {
        return result;
    }
    for_each_node(velocity.grid, [&](const std::array<std::size_t, 3>& at, std::size_t node) {
        if (is_solid(solid, node)) {
            return;
        }
        const Gradient g = velocity_gradient(velocity, solid, at);
        // Q = (|Omega|^2 - |S|^2) / 2 = -(1/2) sum_rc g_rc g_cr.
        double q = 0.0;
        for (std::size_t r = 0; r < 3; ++r) {
            for (std::size_t c = 0; c < 3; ++c) {
                q -= 0.5 * g[r][c] * g[c][r];
            }
        }
        result.values[node] = q;
    });
    return result;
}

ScalarGrid speed(const VectorGrid& velocity) {
    ScalarGrid result;
    result.grid = velocity.grid;
    result.values.reserve(velocity.values.size());
    for (const glm::dvec3& v : velocity.values) {
        result.values.push_back(glm::length(v));
    }
    return result;
}

ScalarGrid derive(const FlowField& field, FieldQuantity quantity, Axis normal) {
    switch (quantity) {
        case FieldQuantity::speed:
            return speed(field.velocity);
        case FieldQuantity::vorticity:
        case FieldQuantity::vorticity_magnitude: {
            const VectorGrid w = vorticity(field.velocity, field.solid);
            ScalarGrid result;
            result.grid = w.grid;
            result.values.reserve(w.values.size());
            const std::size_t component = field.velocity.grid.is_2d() ? 2 : axis_index(normal);
            for (const glm::dvec3& v : w.values) {
                result.values.push_back(quantity == FieldQuantity::vorticity
                                            ? v[static_cast<glm::length_t>(component)]
                                            : glm::length(v));
            }
            return result;
        }
        case FieldQuantity::pressure:
            if (field.pressure) {
                return *field.pressure;
            }
            return ScalarGrid{field.velocity.grid, {}};
        case FieldQuantity::q_criterion:
            return q_criterion(field.velocity, field.solid);
    }
    return {};
}

std::optional<double> sample(const ScalarGrid& field, const glm::dvec3& point) {
    if (field.values.size() != field.grid.node_count() || field.values.empty()) {
        return std::nullopt;
    }
    const std::optional<Stencil> s = stencil(field.grid, point);
    if (!s) {
        return std::nullopt;
    }
    return interpolate<double>(field.grid, *s, field.values);
}

std::optional<glm::dvec3> sample(const VectorGrid& field, const glm::dvec3& point) {
    if (field.values.size() != field.grid.node_count() || field.values.empty()) {
        return std::nullopt;
    }
    const std::optional<Stencil> s = stencil(field.grid, point);
    if (!s) {
        return std::nullopt;
    }
    return interpolate<glm::dvec3>(field.grid, *s, field.values);
}

// APPROXIMATION (V7): section sampling - grid-node resolution in the plane,
// linear along its normal, solid flag of the nearer node - the cut is as
// sharp as the grid; bodies show their staircase unless a mesh covers them.
SectionImage sample_section(const ScalarGrid& field, std::span<const std::uint8_t> solid, Axis axis,
                            double position) {
    SectionImage image;
    image.axis = axis;
    const GridGeometry& grid = field.grid;
    if (validate(grid) || field.values.size() != grid.node_count() ||
        (!solid.empty() && solid.size() != grid.node_count()) ||
        (grid.is_2d() && axis != Axis::z)) {
        return image;
    }
    const std::size_t a = axis_index(axis);
    const std::size_t u_axis = a == 0 ? 1 : 0;
    const std::size_t v_axis = a == 2 ? 1 : 2;
    const auto ga = static_cast<glm::length_t>(a);
    const double low = grid.origin[ga];
    const double high = grid.upper_corner()[ga];
    image.position = std::isfinite(position) ? std::clamp(position, low, high) : low;
    std::size_t k0 = 0;
    std::size_t k1 = 0;
    double w = 0.0;
    if (grid.dims[a] > 1) {
        const double f = (image.position - low) / grid.spacing;
        const auto last = static_cast<double>(grid.dims[a] - 1);
        const double base = std::min(std::floor(f), last - 1.0);
        k0 = static_cast<std::size_t>(base);
        k1 = k0 + 1;
        w = std::clamp(f - base, 0.0, 1.0);
    }
    image.width = grid.dims[u_axis];
    image.height = grid.dims[v_axis];
    image.corner = grid.origin;
    image.corner[ga] = image.position;
    image.u_step = glm::dvec3(0.0);
    image.v_step = glm::dvec3(0.0);
    image.u_step[static_cast<glm::length_t>(u_axis)] = grid.spacing;
    image.v_step[static_cast<glm::length_t>(v_axis)] = grid.spacing;
    image.values.assign(image.width * image.height, 0.0);
    image.solid.assign(image.width * image.height, std::uint8_t{0});
    for (std::size_t v = 0; v < image.height; ++v) {
        for (std::size_t u = 0; u < image.width; ++u) {
            std::array<std::size_t, 3> lo{};
            lo[u_axis] = u;
            lo[v_axis] = v;
            lo[a] = k0;
            std::array<std::size_t, 3> hi = lo;
            hi[a] = k1;
            const std::size_t n0 = grid.index(lo[0], lo[1], lo[2]);
            const std::size_t n1 = grid.index(hi[0], hi[1], hi[2]);
            const std::size_t texel = u + image.width * v;
            image.values[texel] = (1.0 - w) * field.values[n0] + w * field.values[n1];
            image.solid[texel] = is_solid(solid, w < 0.5 ? n0 : n1) ? 1 : 0;
        }
    }
    return image;
}

} // namespace orrery::render
