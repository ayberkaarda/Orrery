#include "orrery/geom/reference_quantities.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace orrery::geom {

namespace {

Vector3 unit_or_throw(const Vector3& d, const char* what) {
    const double n = norm(d);
    if (!std::isfinite(n) || !(n > 0.0)) {
        throw std::invalid_argument(std::string(what) + " must be a non-zero finite vector");
    }
    return d / n;
}

// Extent of the vertices projected on unit axis `d`.
double extent_along(const TriangleMesh& mesh, const Vector3& d) {
    if (mesh.vertices.empty()) {
        return 0.0;
    }
    double lo = dot(mesh.vertices.front(), d);
    double hi = lo;
    for (const Vector3& v : mesh.vertices) {
        const double s = dot(v, d);
        lo = std::min(lo, s);
        hi = std::max(hi, s);
    }
    return hi - lo;
}

struct Point2 {
    double u = 0.0;
    double v = 0.0;
};

double edge(const Point2& a, const Point2& b, double u, double v) {
    return (b.u - a.u) * (v - a.v) - (b.v - a.v) * (u - a.u);
}

} // namespace

double projected_area_bound(const TriangleMesh& mesh, const Vector3& direction) noexcept {
    const double n = norm(direction);
    if (!(n > 0.0)) {
        return 0.0;
    }
    const Vector3 d = direction / n;
    double sum = 0.0;
    for (const Triangle& t : mesh.triangles) {
        sum += std::abs(dot(triangle_cross(mesh, t), d));
    }
    // |cross| is twice the area, and the silhouette is covered twice.
    return 0.25 * sum;
}

double projected_area(const TriangleMesh& mesh, const Vector3& direction, std::size_t resolution) {
    if (resolution < 2 || resolution > 8192) {
        throw std::invalid_argument("projected_area: resolution must be in [2, 8192]");
    }
    const Vector3 d = unit_or_throw(direction, "projection direction");
    if (mesh.vertices.empty() || mesh.triangles.empty()) {
        return 0.0;
    }
    // Orthonormal basis (u, v) of the projection plane.
    const Vector3 helper =
        std::abs(d.x) <= std::abs(d.y) && std::abs(d.x) <= std::abs(d.z)
            ? Vector3{1.0, 0.0, 0.0}
            : (std::abs(d.y) <= std::abs(d.z) ? Vector3{0.0, 1.0, 0.0} : Vector3{0.0, 0.0, 1.0});
    const Vector3 u = normalized(cross(d, helper));
    const Vector3 v = cross(d, u);
    std::vector<Point2> p;
    p.reserve(mesh.vertices.size());
    for (const Vector3& x : mesh.vertices) {
        p.push_back({dot(x, u), dot(x, v)});
    }
    Point2 lo = p.front();
    Point2 hi = p.front();
    for (const Point2& q : p) {
        lo = {std::min(lo.u, q.u), std::min(lo.v, q.v)};
        hi = {std::max(hi.u, q.u), std::max(hi.v, q.v)};
    }
    const double side = std::max(hi.u - lo.u, hi.v - lo.v);
    if (!(side > 0.0)) {
        return 0.0;
    }
    const double h = side / static_cast<double>(resolution);
    auto cells = [&](double extent) {
        return std::clamp<std::size_t>(static_cast<std::size_t>(std::ceil(extent / h)), 1,
                                       resolution);
    };
    const std::size_t nu = cells(hi.u - lo.u);
    const std::size_t nv = cells(hi.v - lo.v);
    std::vector<std::uint8_t> covered(nu * nv, 0);
    // Cell index range whose centres lie in [a, b] (cell centres at
    // lo + (i + 1/2) h).
    auto range = [h](double a, double b, double origin, std::size_t n) {
        const double first = std::ceil((a - origin) / h - 0.5);
        const double last = std::floor((b - origin) / h - 0.5);
        const double top = static_cast<double>(n) - 1.0;
        return std::make_pair(static_cast<std::size_t>(std::clamp(first, 0.0, top)),
                              static_cast<std::size_t>(std::clamp(last, -1.0, top) + 1.0));
    };
    for (const Triangle& t : mesh.triangles) {
        const Point2& a = p[t[0]];
        const Point2& b = p[t[1]];
        const Point2& c = p[t[2]];
        if (edge(a, b, c.u, c.v) == 0.0) {
            continue; // seen edge-on: covers nothing
        }
        const auto [i0, i1] = range(std::min({a.u, b.u, c.u}), std::max({a.u, b.u, c.u}), lo.u, nu);
        const auto [j0, j1] = range(std::min({a.v, b.v, c.v}), std::max({a.v, b.v, c.v}), lo.v, nv);
        for (std::size_t j = j0; j < j1; ++j) {
            const double cv = lo.v + (static_cast<double>(j) + 0.5) * h;
            for (std::size_t i = i0; i < i1; ++i) {
                const double cu = lo.u + (static_cast<double>(i) + 0.5) * h;
                const double e0 = edge(a, b, cu, cv);
                const double e1 = edge(b, c, cu, cv);
                const double e2 = edge(c, a, cu, cv);
                if ((e0 >= 0.0 && e1 >= 0.0 && e2 >= 0.0) ||
                    (e0 <= 0.0 && e1 <= 0.0 && e2 <= 0.0)) {
                    covered[i + nu * j] = 1;
                }
            }
        }
    }
    const auto count =
        static_cast<double>(std::count(covered.begin(), covered.end(), std::uint8_t{1}));
    return count * h * h;
}

ReferenceQuantities reference_quantities(const TriangleMesh& mesh,
                                         const ReferenceOptions& options) {
    const Vector3 flow = unit_or_throw(options.flow_direction, "flow direction");
    const Vector3 lift = unit_or_throw(options.lift_direction, "lift direction");
    const Vector3 span_axis = cross(lift, flow);
    if (!(norm(span_axis) > 1e-12)) {
        throw std::invalid_argument("flow and lift directions must not be parallel");
    }
    ReferenceQuantities q;
    q.wetted_area = surface_area(mesh);
    q.frontal_area = projected_area(mesh, flow, options.raster_resolution);
    q.planform_area = projected_area(mesh, lift, options.raster_resolution);
    q.frontal_area_bound = projected_area_bound(mesh, flow);
    q.planform_area_bound = projected_area_bound(mesh, lift);
    q.reference_length = extent_along(mesh, flow);
    q.span = extent_along(mesh, normalized(span_axis));
    q.volume = signed_volume(mesh);
    // APPROXIMATION: uniform density - the centre of mass is the volume
    // centroid (A1); valid for solid, roughly homogeneous bodies.
    q.centre_of_mass = volume_centroid(mesh);
    return q;
}

double reference_area(const ReferenceQuantities& q, ReferenceAreaKind kind) noexcept {
    switch (kind) {
        case ReferenceAreaKind::frontal:
            return q.frontal_area;
        case ReferenceAreaKind::planform:
            return q.planform_area;
        case ReferenceAreaKind::wetted:
            return q.wetted_area;
    }
    return q.wetted_area;
}

} // namespace orrery::geom
