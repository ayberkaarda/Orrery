#include "orrery/geom/winding_number.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <numeric>
#include <stdexcept>

namespace orrery::geom {

namespace {

constexpr double four_pi = 4.0 * std::numbers::pi;

// Explicit traversal stack: the tree halves its triangle count per level, so
// its depth stays below 33 and the stack below 2 entries per level.
constexpr std::size_t stack_capacity = 128;

double box_distance_squared(const Bounds& b, const Vector3& p) {
    const double dx = std::max({b.min.x - p.x, 0.0, p.x - b.max.x});
    const double dy = std::max({b.min.y - p.y, 0.0, p.y - b.max.y});
    const double dz = std::max({b.min.z - p.z, 0.0, p.z - b.max.z});
    return dx * dx + dy * dy + dz * dz;
}

Bounds merge(const Bounds& a, const Bounds& b) {
    return {{std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y), std::min(a.min.z, b.min.z)},
            {std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y), std::max(a.max.z, b.max.z)}};
}

Bounds point_bounds(const Vector3& p) {
    return {p, p};
}

// Closest point of segment a-b to p; a when the segment has no length.
Vector3 closest_point_on_segment(const Vector3& p, const Vector3& a, const Vector3& b) noexcept {
    const Vector3 ab = b - a;
    const double length2 = squared_norm(ab);
    if (!(length2 > 0.0)) {
        return a;
    }
    const double t = std::clamp(dot(p - a, ab) / length2, 0.0, 1.0);
    return a + ab * t;
}

// A triangle without area is the union of its three edges (segments or
// points): the closest of their closest points.
Vector3 closest_point_on_edges(const Vector3& p, const Vector3& a, const Vector3& b,
                               const Vector3& c) noexcept {
    Vector3 best = closest_point_on_segment(p, a, b);
    double best2 = squared_norm(best - p);
    for (const Vector3& q :
         {closest_point_on_segment(p, b, c), closest_point_on_segment(p, c, a)}) {
        const double d2 = squared_norm(q - p);
        if (d2 < best2) {
            best = q;
            best2 = d2;
        }
    }
    return best;
}

// Power of two near the largest extent of the mesh: the tree works with
// vectors divided by it, so its products neither underflow for tiny meshes
// nor overflow for huge ones (division by a power of two is exact).
double tree_scale(const TriangleMesh& mesh) noexcept {
    const Vector3 e = bounds(mesh).extent();
    const double extent = std::max({e.x, e.y, e.z});
    if (!(extent > 0.0) || !std::isfinite(extent)) {
        return 1.0;
    }
    return std::ldexp(1.0, std::ilogb(extent));
}

} // namespace

double solid_angle(const Vector3& p, const Vector3& a, const Vector3& b,
                   const Vector3& c) noexcept {
    const Vector3 ra = a - p;
    const Vector3 rb = b - p;
    const Vector3 rc = c - p;
    const double la = norm(ra);
    const double lb = norm(rb);
    const double lc = norm(rc);
    if (!(la > 0.0) || !(lb > 0.0) || !(lc > 0.0)) {
        return 0.0; // p on a corner
    }
    // The formula on unit vectors (both arguments of atan2 divided by
    // la lb lc > 0): products of three lengths would underflow or overflow
    // for very small or very large triangles.
    const Vector3 ua = ra / la;
    const Vector3 ub = rb / lb;
    const Vector3 uc = rc / lc;
    const double numerator = dot(ua, cross(ub, uc));
    const double denominator = 1.0 + dot(ua, ub) + dot(ub, uc) + dot(uc, ua);
    return 2.0 * std::atan2(numerator, denominator);
}

double winding_number(const TriangleMesh& mesh, const Vector3& p) noexcept {
    double sum = 0.0;
    for (const Triangle& t : mesh.triangles) {
        sum += solid_angle(p, mesh.vertices[t[0]], mesh.vertices[t[1]], mesh.vertices[t[2]]);
    }
    return sum / four_pi;
}

Vector3 closest_point_on_triangle(const Vector3& p, const Vector3& a, const Vector3& b,
                                  const Vector3& c) noexcept {
    const Vector3 ab = b - a;
    const Vector3 ac = c - a;
    // Without area the barycentric divisions below can be 0 / 0.
    if (!(squared_norm(cross(ab, ac)) > 0.0)) {
        return closest_point_on_edges(p, a, b, c);
    }
    const Vector3 ap = p - a;
    const double d1 = dot(ab, ap);
    const double d2 = dot(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
        return a;
    }
    const Vector3 bp = p - b;
    const double d3 = dot(ab, bp);
    const double d4 = dot(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) {
        return b;
    }
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        return a + ab * (d1 / (d1 - d3));
    }
    const Vector3 cp = p - c;
    const double d5 = dot(ab, cp);
    const double d6 = dot(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) {
        return c;
    }
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        return a + ac * (d2 / (d2 - d6));
    }
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    const double sum = va + vb + vc;
    if (!(sum > 0.0)) {
        // Nearly degenerate (the area underflowed in these products).
        return closest_point_on_edges(p, a, b, c);
    }
    return a + ab * (vb / sum) + ac * (vc / sum);
}

MeshTree::MeshTree(const TriangleMesh& mesh, double beta, std::size_t leaf_size)
    : mesh_(&mesh), beta_(beta), scale_(tree_scale(mesh)) {
    if (!std::isfinite(beta) || !(beta > 0.0) || leaf_size == 0) {
        throw std::invalid_argument("MeshTree: beta must be positive and leaf_size at least 1");
    }
    if (mesh.triangles.size() > std::numeric_limits<std::uint32_t>::max()) {
        throw std::length_error("MeshTree: too many triangles");
    }
    order_.assign(mesh.triangles.size(), 0U);
    std::iota(order_.begin(), order_.end(), 0U);
    if (!order_.empty()) {
        nodes_.reserve(2 * (mesh.triangles.size() / leaf_size + 1));
        static_cast<void>(build(0, static_cast<std::uint32_t>(order_.size()), leaf_size));
    }
}

std::uint32_t MeshTree::build(std::uint32_t first, std::uint32_t count, std::size_t leaf_size) {
    const TriangleMesh& m = *mesh_;
    const auto index = static_cast<std::uint32_t>(nodes_.size());
    nodes_.emplace_back();
    auto centroid = [&m](std::uint32_t t) {
        const Triangle& tri = m.triangles[t];
        return (m.vertices[tri[0]] + m.vertices[tri[1]] + m.vertices[tri[2]]) / 3.0;
    };

    Node node;
    node.first = first;
    node.count = count;
    node.box = point_bounds(m.vertices[m.triangles[order_[first]][0]]);
    Bounds centres = point_bounds(centroid(order_[first]));
    double area = 0.0;
    Vector3 weighted;
    for (std::uint32_t i = first; i < first + count; ++i) {
        const Triangle& tri = m.triangles[order_[i]];
        for (std::uint32_t v : tri) {
            node.box = merge(node.box, point_bounds(m.vertices[v]));
        }
        const Vector3 c = centroid(order_[i]);
        centres = merge(centres, point_bounds(c));
        // Area vector in units of scale_^2.
        const Vector3& v0 = m.vertices[tri[0]];
        const Vector3 n2 =
            cross((m.vertices[tri[1]] - v0) / scale_, (m.vertices[tri[2]] - v0) / scale_);
        const double a = 0.5 * norm(n2);
        node.area_normal += n2 * 0.5;
        weighted += c * a;
        area += a;
    }
    node.centre = area > 0.0 ? weighted / area : node.box.centre();

    if (count <= leaf_size) {
        node.leaf = true;
        for (std::uint32_t i = first; i < first + count; ++i) {
            for (std::uint32_t v : m.triangles[order_[i]]) {
                node.radius = std::max(node.radius, norm(m.vertices[v] - node.centre));
            }
        }
        nodes_[index] = node;
        return index;
    }
    // Median split of the centroids along their longest axis.
    const Vector3 e = centres.extent();
    const int axis = e.x >= e.y && e.x >= e.z ? 0 : (e.y >= e.z ? 1 : 2);
    auto key = [&](std::uint32_t t) {
        const Vector3 c = centroid(t);
        return axis == 0 ? c.x : (axis == 1 ? c.y : c.z);
    };
    const std::uint32_t half = count / 2;
    std::nth_element(order_.begin() + first, order_.begin() + first + half,
                     order_.begin() + first + count, [&](std::uint32_t a, std::uint32_t b) {
                         const double ka = key(a);
                         const double kb = key(b);
                         return ka != kb ? ka < kb : a < b;
                     });
    node.leaf = false;
    node.left = build(first, half, leaf_size);
    node.right = build(first + half, count - half, leaf_size);
    for (std::uint32_t child : {node.left, node.right}) {
        const Node& c = nodes_[child];
        node.radius = std::max(node.radius, norm(c.centre - node.centre) + c.radius);
    }
    nodes_[index] = node;
    return index;
}

double MeshTree::winding_number(const Vector3& p) const noexcept {
    if (nodes_.empty()) {
        return 0.0;
    }
    const TriangleMesh& m = *mesh_;
    std::array<std::uint32_t, stack_capacity> stack{};
    std::size_t top = 0;
    stack[top++] = 0;
    double sum = 0.0;
    while (top > 0) {
        const Node& node = nodes_[stack[--top]];
        const Vector3 d = node.centre - p;
        const double dist = norm(d);
        if (dist > beta_ * node.radius) {
            // dot(A, d) / dist^3 with A = area_normal scale^2, evaluated in
            // an order that stays within range at any scale.
            const double flux = dot(node.area_normal, d / dist);
            if (flux != 0.0) {
                const double ratio = scale_ / dist;
                sum += flux * ratio * ratio;
            }
            continue;
        }
        if (node.leaf) {
            for (std::uint32_t i = node.first; i < node.first + node.count; ++i) {
                const Triangle& t = m.triangles[order_[i]];
                sum += solid_angle(p, m.vertices[t[0]], m.vertices[t[1]], m.vertices[t[2]]);
            }
            continue;
        }
        stack[top++] = node.left;
        stack[top++] = node.right;
    }
    return sum / four_pi;
}

double MeshTree::distance(const Vector3& p, double max_distance) const noexcept {
    double best2 = max_distance * max_distance;
    if (nodes_.empty()) {
        return max_distance;
    }
    const TriangleMesh& m = *mesh_;
    std::array<std::uint32_t, stack_capacity> stack{};
    std::size_t top = 0;
    stack[top++] = 0;
    while (top > 0) {
        const Node& node = nodes_[stack[--top]];
        if (box_distance_squared(node.box, p) >= best2) {
            continue;
        }
        if (node.leaf) {
            for (std::uint32_t i = node.first; i < node.first + node.count; ++i) {
                const Triangle& t = m.triangles[order_[i]];
                const Vector3 q = closest_point_on_triangle(p, m.vertices[t[0]], m.vertices[t[1]],
                                                            m.vertices[t[2]]);
                best2 = std::min(best2, squared_norm(q - p));
            }
            continue;
        }
        // Visit the nearer child first (pushed last).
        const double dl = box_distance_squared(nodes_[node.left].box, p);
        const double dr = box_distance_squared(nodes_[node.right].box, p);
        const bool left_first = dl <= dr;
        stack[top++] = left_first ? node.right : node.left;
        stack[top++] = left_first ? node.left : node.right;
    }
    return std::sqrt(best2);
}

} // namespace orrery::geom
