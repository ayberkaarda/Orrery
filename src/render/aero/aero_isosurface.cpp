#include "orrery/render/aero_isosurface.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <span>
#include <utility>

#include <glm/geometric.hpp>

#include "aero_internal.h"
#include "orrery/render/aero_compute.h"

namespace orrery::render {

namespace {

// Cube corners: corner c sits at offset (c & 1, (c >> 1) & 1, (c >> 2) & 1).
// The six faces, each listed counter-clockwise seen from outside the cube.
constexpr std::array<std::array<std::uint8_t, 4>, 6> kFaces{{
    {0, 4, 6, 2}, // x = 0
    {1, 3, 7, 5}, // x = 1
    {0, 1, 5, 4}, // y = 0
    {2, 6, 7, 3}, // y = 1
    {0, 2, 3, 1}, // z = 0
    {4, 5, 7, 6}, // z = 1
}};

// Cell edge between two corners that differ in one coordinate: 0-3 along x
// (index y + 2 z of the lower corner), 4-7 along y (x + 2 z), 8-11 along z
// (x + 2 y).
std::uint8_t edge_between(std::uint8_t a, std::uint8_t b) noexcept {
    const auto lo = static_cast<unsigned>(std::min(a, b));
    const unsigned axis_bit = static_cast<unsigned>(a ^ b);
    const unsigned x = lo & 1U;
    const unsigned y = (lo >> 1U) & 1U;
    const unsigned z = (lo >> 2U) & 1U;
    if (axis_bit == 1U) {
        return static_cast<std::uint8_t>(y + 2U * z);
    }
    if (axis_bit == 2U) {
        return static_cast<std::uint8_t>(4U + x + 2U * z);
    }
    return static_cast<std::uint8_t>(8U + x + 2U * y);
}

// Corners at the ends of each cell edge (lower corner first).
constexpr std::array<std::array<std::uint8_t, 2>, 12> kEdgeCorners{{
    {0, 1},
    {2, 3},
    {4, 5},
    {6, 7}, // along x
    {0, 2},
    {1, 3},
    {4, 6},
    {5, 7}, // along y
    {0, 4},
    {1, 5},
    {2, 6},
    {3, 7}, // along z
}};

// True when cell edges a and b lie on a common cube face.
bool share_face(std::uint8_t a, std::uint8_t b) noexcept {
    for (const auto& face : kFaces) {
        const auto on = [&face](std::uint8_t e) {
            const auto has = [&face](std::uint8_t c) {
                return std::ranges::find(face, c) != face.end();
            };
            return has(kEdgeCorners[e][0]) && has(kEdgeCorners[e][1]);
        };
        if (on(a) && on(b)) {
            return true;
        }
    }
    return false;
}

glm::dvec3 edge_midpoint(std::uint8_t e) noexcept {
    const auto corner = [](std::uint8_t c) {
        return glm::dvec3(c & 1U, (c >> 1U) & 1U, (c >> 2U) & 1U);
    };
    return 0.5 * (corner(kEdgeCorners[e][0]) + corner(kEdgeCorners[e][1]));
}

// Triangulates one loop of cell edges (in loop order, which fixes the
// winding) by dynamic programming over all polygon triangulations. A
// diagonal between two points on the same cube face costs far more than any
// other: the neighbouring cell may join the same two points as well, and an
// edge in four triangles would not be manifold. Among the rest the shortest
// diagonals win (better shaped triangles).
void triangulate_loop(const std::vector<std::uint8_t>& loop, std::vector<std::uint8_t>& out) {
    const std::size_t m = loop.size();
    if (m < 3) {
        return;
    }
    constexpr double kSameFace = 1000.0;
    const auto diagonal = [&](std::size_t a, std::size_t b) {
        if (b == a + 1 || (a == 0 && b == m - 1)) {
            return 0.0; // a side of the loop
        }
        const double length = glm::length(edge_midpoint(loop[a]) - edge_midpoint(loop[b]));
        return length + (share_face(loop[a], loop[b]) ? kSameFace : 0.0);
    };
    std::vector<double> cost(m * m, 0.0);
    std::vector<std::size_t> split(m * m, 0);
    for (std::size_t gap = 2; gap < m; ++gap) {
        for (std::size_t i = 0; i + gap < m; ++i) {
            const std::size_t j = i + gap;
            double best = std::numeric_limits<double>::infinity();
            for (std::size_t k = i + 1; k < j; ++k) {
                const double c =
                    cost[i * m + k] + cost[k * m + j] + diagonal(i, k) + diagonal(k, j);
                if (c < best) {
                    best = c;
                    split[i * m + j] = k;
                }
            }
            cost[i * m + j] = best;
        }
    }
    // Emit (i, k, j) for every sub-polygon, keeping the loop's winding.
    std::vector<std::array<std::size_t, 2>> pending{{0, m - 1}};
    while (!pending.empty()) {
        const auto [i, j] = pending.back();
        pending.pop_back();
        if (j < i + 2) {
            continue;
        }
        const std::size_t k = split[i * m + j];
        out.insert(out.end(), {loop[i], loop[k], loop[j]});
        pending.push_back({i, k});
        pending.push_back({k, j});
    }
}

// Triangles of one corner configuration (bit c of `mask` = corner c inside).
// Per face, walking its corners counter-clockwise (seen from outside), each
// crossing from outside to inside is joined to the next crossing, which
// leaves the inside: one segment per inside run, so on an ambiguous face the
// two inside corners are cut off separately. Every crossed edge belongs to
// two faces and ends one segment and starts another, so the segments form
// closed loops (triangulate_loop). With this direction the triangles wind
// counter-clockwise seen from the outside (from lower values).
std::vector<std::uint8_t> triangulate(unsigned mask) {
    std::array<int, 12> next{};
    next.fill(-1);
    for (const auto& face : kFaces) {
        std::array<bool, 4> inside{};
        for (std::size_t i = 0; i < 4; ++i) {
            inside[i] = ((mask >> face[i]) & 1U) != 0U;
        }
        for (std::size_t i = 0; i < 4; ++i) {
            const std::size_t j = (i + 1) % 4;
            if (inside[i] || !inside[j]) {
                continue; // not an entry from outside (corner i) to inside (corner j)
            }
            // The exit: the first edge after j whose start is inside and end outside.
            for (std::size_t s = 1; s < 4; ++s) {
                const std::size_t a = (j + s - 1) % 4;
                const std::size_t b = (j + s) % 4;
                if (inside[a] && !inside[b]) {
                    next[edge_between(face[i], face[j])] = edge_between(face[a], face[b]);
                    break;
                }
            }
        }
    }
    std::vector<std::uint8_t> triangles;
    std::array<bool, 12> used{};
    for (std::size_t start = 0; start < 12; ++start) {
        if (next[start] < 0 || used[start]) {
            continue;
        }
        std::vector<std::uint8_t> loop;
        std::size_t e = start;
        while (!used[e]) {
            used[e] = true;
            loop.push_back(static_cast<std::uint8_t>(e));
            e = static_cast<std::size_t>(next[e]);
        }
        triangulate_loop(loop, triangles);
    }
    return triangles;
}

std::size_t corner_node(const GridGeometry& grid, std::size_t i, std::size_t j, std::size_t k,
                        std::uint8_t corner) noexcept {
    return grid.index(i + (corner & 1U), j + ((corner >> 1U) & 1U), k + ((corner >> 2U) & 1U));
}

// Global id of cell edge e of cell (i, j, k): 3 * (lower node) + axis.
std::uint64_t global_edge(const GridGeometry& grid, std::size_t i, std::size_t j, std::size_t k,
                          std::uint8_t e) noexcept {
    const std::uint8_t lower = kEdgeCorners[e][0];
    const std::uint64_t axis = e / 4U;
    return 3U * static_cast<std::uint64_t>(corner_node(grid, i, j, k, lower)) + axis;
}

// Node gradient by central differences (one-sided at the grid edges).
glm::dvec3 node_gradient(const ScalarGrid& field, std::size_t node) {
    const GridGeometry& g = field.grid;
    std::array<std::size_t, 3> at{node % g.dims[0], (node / g.dims[0]) % g.dims[1],
                                  node / (g.dims[0] * g.dims[1])};
    glm::dvec3 gradient{0.0};
    for (std::size_t a = 0; a < 3; ++a) {
        if (g.dims[a] < 2) {
            continue;
        }
        std::array<std::size_t, 3> lo = at;
        std::array<std::size_t, 3> hi = at;
        if (at[a] > 0) {
            --lo[a];
        }
        if (at[a] + 1 < g.dims[a]) {
            ++hi[a];
        }
        const double span = static_cast<double>(hi[a] - lo[a]) * g.spacing;
        gradient[static_cast<glm::length_t>(a)] = (field.values[g.index(hi[0], hi[1], hi[2])] -
                                                   field.values[g.index(lo[0], lo[1], lo[2])]) /
                                                  span;
    }
    return gradient;
}

// Position and normal of the vertex on global edge `edge`.
struct EdgeVertex {
    glm::dvec3 position{0.0};
    glm::dvec3 normal{0.0};
};

double edge_fraction(double a, double b, double level) noexcept {
    const double denominator = b - a;
    return denominator != 0.0 ? std::clamp((level - a) / denominator, 0.0, 1.0) : 0.5;
}

std::size_t node_of_edge(std::uint64_t edge) noexcept {
    return static_cast<std::size_t>(edge / 3U);
}

std::size_t neighbour_of_edge(const GridGeometry& g, std::uint64_t edge) noexcept {
    const std::size_t node = node_of_edge(edge);
    const auto axis = static_cast<std::size_t>(edge % 3U);
    const std::size_t stride = axis == 0 ? 1 : (axis == 1 ? g.dims[0] : g.dims[0] * g.dims[1]);
    return node + stride;
}

glm::dvec3 vertex_normal(const ScalarGrid& field, std::uint64_t edge, double t) {
    const glm::dvec3 g0 = node_gradient(field, node_of_edge(edge));
    const glm::dvec3 g1 = node_gradient(field, neighbour_of_edge(field.grid, edge));
    const glm::dvec3 g = g0 + (g1 - g0) * t;
    const double length = glm::length(g);
    return length > 0.0 ? -g / length : glm::dvec3(0.0);
}

// Indexed surface from triangles given as global edges, with each vertex
// placed by `place(edge)`; triangles are rotated to start at their smallest
// vertex and sorted, so the result does not depend on emission order.
template <typename Place>
IsoSurface assemble(std::vector<std::array<std::uint64_t, 3>> triangles, const Place& place) {
    std::vector<std::uint64_t> edges;
    edges.reserve(triangles.size() * 3);
    for (const auto& t : triangles) {
        edges.insert(edges.end(), t.begin(), t.end());
    }
    std::ranges::sort(edges);
    edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
    IsoSurface surface;
    surface.edges = edges;
    surface.positions.reserve(edges.size());
    surface.normals.reserve(edges.size());
    for (const std::uint64_t e : edges) {
        const EdgeVertex v = place(e);
        surface.positions.push_back(v.position);
        surface.normals.push_back(v.normal);
    }
    const auto index_of = [&](std::uint64_t e) {
        return static_cast<std::uint32_t>(std::ranges::lower_bound(edges, e) - edges.begin());
    };
    std::vector<std::array<std::uint32_t, 3>> indexed;
    indexed.reserve(triangles.size());
    for (const auto& t : triangles) {
        std::array<std::uint32_t, 3> v{index_of(t[0]), index_of(t[1]), index_of(t[2])};
        const auto smallest = std::ranges::min_element(v) - v.begin();
        std::ranges::rotate(v, v.begin() + smallest);
        indexed.push_back(v);
    }
    std::ranges::sort(indexed);
    surface.indices.reserve(indexed.size() * 3);
    for (const auto& t : indexed) {
        surface.indices.insert(surface.indices.end(), t.begin(), t.end());
    }
    return surface;
}

bool usable(const ScalarGrid& field) {
    return !validate(field.grid) && !field.grid.is_2d() && field.grid.dims[2] >= 2 &&
           field.values.size() == field.grid.node_count();
}

} // namespace

const std::vector<std::vector<std::uint8_t>>& marching_cubes_table() {
    static const std::vector<std::vector<std::uint8_t>> table = [] {
        std::vector<std::vector<std::uint8_t>> cases;
        cases.reserve(256);
        for (unsigned mask = 0; mask < 256U; ++mask) {
            cases.push_back(triangulate(mask));
        }
        return cases;
    }();
    return table;
}

IsoSurface marching_cubes(const ScalarGrid& field, double level) {
    if (!usable(field)) {
        return {};
    }
    const GridGeometry& g = field.grid;
    const auto& table = marching_cubes_table();
    std::vector<std::array<std::uint64_t, 3>> triangles;
    for (std::size_t k = 0; k + 1 < g.dims[2]; ++k) {
        for (std::size_t j = 0; j + 1 < g.dims[1]; ++j) {
            for (std::size_t i = 0; i + 1 < g.dims[0]; ++i) {
                unsigned mask = 0;
                for (std::uint8_t c = 0; c < 8; ++c) {
                    if (field.values[corner_node(g, i, j, k, c)] > level) {
                        mask |= 1U << c;
                    }
                }
                const std::vector<std::uint8_t>& edges = table[mask];
                for (std::size_t t = 0; t + 2 < edges.size(); t += 3) {
                    triangles.push_back({global_edge(g, i, j, k, edges[t]),
                                         global_edge(g, i, j, k, edges[t + 1]),
                                         global_edge(g, i, j, k, edges[t + 2])});
                }
            }
        }
    }
    return assemble(std::move(triangles), [&](std::uint64_t e) {
        const std::size_t a = node_of_edge(e);
        const std::size_t b = neighbour_of_edge(g, e);
        const double t = edge_fraction(field.values[a], field.values[b], level);
        const auto axis = static_cast<glm::length_t>(e % 3U);
        glm::dvec3 p = g.node_position(a % g.dims[0], (a / g.dims[0]) % g.dims[1],
                                       a / (g.dims[0] * g.dims[1]));
        p[axis] += t * g.spacing;
        return EdgeVertex{p, vertex_normal(field, e, t)};
    });
}

core::Result<IsoSurface, std::string> marching_cubes_gpu(aero::compute::IComputeBackend& backend,
                                                         const ScalarGrid& field, double level,
                                                         std::size_t max_triangles) {
    using aero::compute::BufferBinding;
    using aero::compute::DispatchCommand;
    using aero::compute::Uniform;
    if (!usable(field)) {
        return IsoSurface{};
    }
    if (!backend.has_kernel(kernel_marching_cubes)) {
        return core::failure(std::string("marching cubes: the backend has no kernel ") +
                             std::string(kernel_marching_cubes));
    }
    if (max_triangles == 0 || max_triangles > (std::size_t{1} << 24U)) {
        return core::failure(std::string("marching cubes: max_triangles must be in [1, 2^24]"));
    }
    const GridGeometry& g = field.grid;
    // Values relative to the level, in binary32 (the kernel's inside test is
    // value > 0, so the comparison is exact against the double test for every
    // value that rounds away from zero).
    std::vector<float> values;
    values.reserve(field.values.size());
    for (const double v : field.values) {
        values.push_back(detail::to_float(v - level));
    }
    const auto& table = marching_cubes_table();
    std::vector<std::uint32_t> packed(256U * detail::kMarchingCubesStride, 0U);
    for (std::size_t c = 0; c < 256; ++c) {
        packed[c * detail::kMarchingCubesStride] = static_cast<std::uint32_t>(table[c].size() / 3);
        for (std::size_t e = 0; e < table[c].size(); ++e) {
            packed[c * detail::kMarchingCubesStride + 1 + e] = table[c][e];
        }
    }
    auto value_buffer = detail::upload_new(backend, std::as_bytes(std::span<const float>(values)));
    if (!value_buffer) {
        return core::failure("marching cubes: " + value_buffer.error());
    }
    auto table_buffer =
        detail::upload_new(backend, std::as_bytes(std::span<const std::uint32_t>(packed)));
    if (!table_buffer) {
        return core::failure("marching cubes: " + table_buffer.error());
    }
    auto counter = aero::compute::make_buffer(backend, sizeof(std::uint32_t));
    if (!counter) {
        return core::failure("marching cubes: " + counter.error());
    }
    auto output = aero::compute::make_buffer(backend, max_triangles * detail::kTriangleRecordBytes);
    if (!output) {
        return core::failure("marching cubes: " + output.error());
    }
    const std::array<Uniform, 4> uniforms{
        Uniform{"u_nx", static_cast<std::uint32_t>(g.dims[0])},
        Uniform{"u_ny", static_cast<std::uint32_t>(g.dims[1])},
        Uniform{"u_nz", static_cast<std::uint32_t>(g.dims[2])},
        Uniform{"u_max_triangles", static_cast<std::uint32_t>(max_triangles)},
    };
    const std::array<BufferBinding, 4> buffers{
        BufferBinding{0, value_buffer.value().id()},
        BufferBinding{1, table_buffer.value().id()},
        BufferBinding{2, counter.value().id()},
        BufferBinding{3, output.value().id()},
    };
    const DispatchCommand command{kernel_marching_cubes,
                                  {static_cast<std::uint32_t>(g.dims[0] - 1),
                                   static_cast<std::uint32_t>(g.dims[1] - 1),
                                   static_cast<std::uint32_t>(g.dims[2] - 1)},
                                  uniforms,
                                  buffers};
    if (auto done = backend.dispatch(command); !done) {
        return core::failure("marching cubes: " + done.error());
    }
    backend.barrier();
    std::uint32_t emitted = 0;
    if (auto read = backend.readback(counter.value().id(), 0,
                                     std::as_writable_bytes(std::span<std::uint32_t>(&emitted, 1)));
        !read) {
        return core::failure("marching cubes: " + read.error());
    }
    if (emitted > max_triangles) {
        return core::failure("marching cubes: " + std::to_string(emitted) +
                             " triangles exceed the limit of " + std::to_string(max_triangles));
    }
    std::vector<detail::TriangleRecord> records(emitted);
    if (emitted > 0) {
        if (auto read = backend.readback(
                output.value().id(), 0,
                std::as_writable_bytes(std::span<detail::TriangleRecord>(records)));
            !read) {
            return core::failure("marching cubes: " + read.error());
        }
    }
    std::vector<std::array<std::uint64_t, 3>> triangles;
    triangles.reserve(records.size());
    std::map<std::uint64_t, glm::dvec3> placed;
    for (const detail::TriangleRecord& r : records) {
        std::array<std::uint64_t, 3> t{};
        for (std::size_t v = 0; v < 3; ++v) {
            t[v] = r.edge[v];
            placed.emplace(r.edge[v],
                           g.origin + g.spacing * glm::dvec3(r.position[v][0], r.position[v][1],
                                                             r.position[v][2]));
        }
        triangles.push_back(t);
    }
    return assemble(std::move(triangles), [&](std::uint64_t e) {
        const std::size_t a = node_of_edge(e);
        const std::size_t b = neighbour_of_edge(g, e);
        const double t = edge_fraction(field.values[a], field.values[b], level);
        return EdgeVertex{placed.at(e), vertex_normal(field, e, t)};
    });
}

double surface_area(const IsoSurface& surface) {
    double area = 0.0;
    for (std::size_t t = 0; t + 2 < surface.indices.size(); t += 3) {
        const glm::dvec3& a = surface.positions[surface.indices[t]];
        const glm::dvec3& b = surface.positions[surface.indices[t + 1]];
        const glm::dvec3& c = surface.positions[surface.indices[t + 2]];
        area += 0.5 * glm::length(glm::cross(b - a, c - a));
    }
    return area;
}

MeshTopology topology(const IsoSurface& surface) {
    // Directed edge (from, to) -> uses.
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::size_t> directed;
    MeshTopology result;
    for (std::size_t t = 0; t + 2 < surface.indices.size(); t += 3) {
        const std::array<std::uint32_t, 3> v{surface.indices[t], surface.indices[t + 1],
                                             surface.indices[t + 2]};
        if (v[0] == v[1] || v[1] == v[2] || v[0] == v[2]) {
            ++result.degenerate_triangles;
        }
        for (std::size_t e = 0; e < 3; ++e) {
            ++directed[{v[e], v[(e + 1) % 3]}];
        }
    }
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::pair<std::size_t, std::size_t>>
        undirected;
    for (const auto& [edge, uses] : directed) {
        const bool forward = edge.first < edge.second;
        auto& count = undirected[forward ? edge : std::make_pair(edge.second, edge.first)];
        (forward ? count.first : count.second) += uses;
    }
    result.edges = undirected.size();
    for (const auto& [edge, count] : undirected) {
        const std::size_t total = count.first + count.second;
        if (total == 1) {
            ++result.boundary_edges;
        } else if (total > 2) {
            ++result.non_manifold_edges;
        } else if (count.first != 1) {
            ++result.misoriented_edges;
        }
    }
    return result;
}

} // namespace orrery::render
