#include "orrery/geom/panel_mesh.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace orrery::geom {

namespace {

// (lo, hi, triangle) for every triangle edge, sorted.
using EdgeKey = std::tuple<std::uint32_t, std::uint32_t, std::uint32_t>;

std::vector<EdgeKey> sorted_edges(const TriangleMesh& mesh) {
    if (mesh.triangles.size() > std::numeric_limits<std::uint32_t>::max()) {
        throw std::length_error("panel mesh: too many triangles");
    }
    std::vector<EdgeKey> edges;
    edges.reserve(3 * mesh.triangles.size());
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        for (std::size_t k = 0; k < 3; ++k) {
            const std::uint32_t a = mesh.triangles[t][k];
            const std::uint32_t b = mesh.triangles[t][(k + 1) % 3];
            edges.emplace_back(std::min(a, b), std::max(a, b), static_cast<std::uint32_t>(t));
        }
    }
    std::sort(edges.begin(), edges.end());
    return edges;
}

// Triangles sharing edge (a, b).
std::vector<std::uint32_t> faces_of(const std::vector<EdgeKey>& edges, std::uint32_t a,
                                    std::uint32_t b) {
    const std::uint32_t lo = std::min(a, b);
    const std::uint32_t hi = std::max(a, b);
    std::vector<std::uint32_t> faces;
    auto it = std::lower_bound(edges.begin(), edges.end(), EdgeKey{lo, hi, 0U});
    for (; it != edges.end() && std::get<0>(*it) == lo && std::get<1>(*it) == hi; ++it) {
        faces.push_back(std::get<2>(*it));
    }
    return faces;
}

// Smallest wake panel area accepted, relative to the area of the rectangle
// spanned by its trailing-edge segment and the wake step.
constexpr double wake_area_ratio = 1e-9;

Vector3 unit_normal(const TriangleMesh& mesh, const Triangle& t) {
    return normalized(triangle_cross(mesh, t));
}

} // namespace

PanelMesh build_panels(const TriangleMesh& mesh) {
    PanelMesh out;
    out.panels.reserve(mesh.triangles.size());
    for (const Triangle& t : mesh.triangles) {
        const Vector3 n2 = triangle_cross(mesh, t);
        const double twice_area = norm(n2);
        if (!(twice_area > 0.0)) {
            throw std::invalid_argument("build_panels: triangle without area");
        }
        Panel p;
        p.vertices = t;
        p.collocation = (mesh.vertices[t[0]] + mesh.vertices[t[1]] + mesh.vertices[t[2]]) / 3.0;
        p.normal = n2 / twice_area;
        p.area = 0.5 * twice_area;
        out.panels.push_back(p);
    }
    return out;
}

std::vector<std::uint32_t> detect_trailing_edge(const TriangleMesh& mesh,
                                                const TrailingEdgeOptions& options) {
    const Vector3 flow = normalized(options.flow_direction);
    const double cos_limit = std::cos(options.min_normal_angle);
    const std::vector<EdgeKey> edges = sorted_edges(mesh);
    std::vector<std::pair<std::uint32_t, std::uint32_t>> sharp;
    for (std::size_t i = 0; i < edges.size();) {
        std::size_t j = i + 1;
        while (j < edges.size() && std::get<0>(edges[j]) == std::get<0>(edges[i]) &&
               std::get<1>(edges[j]) == std::get<1>(edges[i])) {
            ++j;
        }
        if (j - i == 2) {
            const Vector3 n0 = unit_normal(mesh, mesh.triangles[std::get<2>(edges[i])]);
            const Vector3 n1 = unit_normal(mesh, mesh.triangles[std::get<2>(edges[i + 1])]);
            if (dot(n0, n1) < cos_limit && dot(n0, flow) > 0.0 && dot(n1, flow) > 0.0) {
                sharp.emplace_back(std::get<0>(edges[i]), std::get<1>(edges[i]));
            }
        }
        i = j;
    }
    if (sharp.empty()) {
        return {};
    }
    // Adjacency of the sharp edges: a simple chain has two ends of degree 1
    // and inner vertices of degree 2.
    std::vector<std::pair<std::uint32_t, std::uint32_t>> adjacency;
    for (const auto& [a, b] : sharp) {
        adjacency.emplace_back(a, b);
        adjacency.emplace_back(b, a);
    }
    std::sort(adjacency.begin(), adjacency.end());
    auto neighbours = [&adjacency](std::uint32_t v) {
        std::vector<std::uint32_t> out;
        auto it = std::lower_bound(adjacency.begin(), adjacency.end(), std::make_pair(v, 0U));
        for (; it != adjacency.end() && it->first == v; ++it) {
            out.push_back(it->second);
        }
        return out;
    };
    std::vector<std::uint32_t> ends;
    for (std::size_t i = 0; i < adjacency.size();) {
        std::size_t j = i + 1;
        while (j < adjacency.size() && adjacency[j].first == adjacency[i].first) {
            ++j;
        }
        if (j - i == 1) {
            ends.push_back(adjacency[i].first);
        } else if (j - i > 2) {
            return {};
        }
        i = j;
    }
    if (ends.size() != 2) {
        return {};
    }
    const Vector3 span = normalized(options.span_direction);
    std::uint32_t current =
        dot(mesh.vertices[ends[0]], span) <= dot(mesh.vertices[ends[1]], span) ? ends[0] : ends[1];
    std::vector<std::uint32_t> chain{current};
    std::uint32_t previous = std::numeric_limits<std::uint32_t>::max();
    while (chain.size() <= sharp.size()) {
        std::uint32_t next = std::numeric_limits<std::uint32_t>::max();
        for (std::uint32_t n : neighbours(current)) {
            if (n != previous) {
                next = n;
            }
        }
        if (next == std::numeric_limits<std::uint32_t>::max()) {
            break;
        }
        previous = current;
        current = next;
        chain.push_back(current);
    }
    // Every sharp edge must be on the chain (no separate pieces).
    if (chain.size() != sharp.size() + 1) {
        return {};
    }
    return chain;
}

void add_wake(PanelMesh& panels, const TriangleMesh& mesh,
              const std::vector<std::uint32_t>& trailing_edge, const WakeOptions& options) {
    if (options.rows < 1 || !std::isfinite(options.length) || !(options.length > 0.0) ||
        !isfinite(options.direction) || !(norm(options.direction) > 0.0) || !isfinite(options.up)) {
        throw std::invalid_argument(
            "add_wake: need rows >= 1, a positive finite length and finite, "
            "non-zero direction and up vectors");
    }
    const Vector3 step = normalized(options.direction) * (options.length / options.rows);
    if (!isfinite(step) || !(norm(step) > 0.0)) {
        throw std::invalid_argument("add_wake: wake step is not representable");
    }
    const std::vector<EdgeKey> edges = sorted_edges(mesh);
    // Built aside and appended only when every panel is usable.
    std::vector<WakePanel> wake;
    for (std::size_t s = 0; s + 1 < trailing_edge.size(); ++s) {
        const std::uint32_t a = trailing_edge[s];
        const std::uint32_t b = trailing_edge[s + 1];
        if (a >= mesh.vertices.size() || b >= mesh.vertices.size()) {
            throw std::invalid_argument("add_wake: trailing-edge vertex out of range");
        }
        const std::vector<std::uint32_t> faces = faces_of(edges, a, b);
        if (faces.size() != 2) {
            throw std::invalid_argument("add_wake: trailing-edge segment is not a two-face edge");
        }
        const double up0 = dot(unit_normal(mesh, mesh.triangles[faces[0]]), options.up);
        const double up1 = dot(unit_normal(mesh, mesh.triangles[faces[1]]), options.up);
        const std::uint32_t upper = up0 >= up1 ? faces[0] : faces[1];
        const std::uint32_t lower = up0 >= up1 ? faces[1] : faces[0];
        for (int r = 0; r < options.rows; ++r) {
            WakePanel w;
            const Vector3 p0 = mesh.vertices[a] + step * static_cast<double>(r);
            const Vector3 p1 = mesh.vertices[b] + step * static_cast<double>(r);
            w.corners = {p0, p1, p1 + step, p0 + step};
            w.collocation = (w.corners[0] + w.corners[1] + w.corners[2] + w.corners[3]) / 4.0;
            // Area vector of a planar quadrilateral: half the cross product
            // of its diagonals.
            const Vector3 n2 = cross(w.corners[2] - w.corners[0], w.corners[3] - w.corners[1]);
            w.area = 0.5 * norm(n2);
            // A direction along the segment sweeps no area: no normal.
            const double edge = norm(mesh.vertices[b] - mesh.vertices[a]);
            if (!std::isfinite(w.area) || !(w.area > wake_area_ratio * edge * norm(step))) {
                throw std::invalid_argument(
                    "add_wake: wake panel without area (direction along the trailing edge)");
            }
            w.normal = normalized(n2);
            w.upper_panel = upper;
            w.lower_panel = lower;
            w.segment = static_cast<std::uint32_t>(s);
            w.row = static_cast<std::uint32_t>(r);
            wake.push_back(w);
        }
    }
    panels.wake.insert(panels.wake.end(), wake.begin(), wake.end());
}

} // namespace orrery::geom
