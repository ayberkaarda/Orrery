#include "orrery/geom/triangle_mesh.h"

#include <algorithm>
#include <cstddef>
#include <stdexcept>

namespace orrery::geom {

Bounds bounds(const TriangleMesh& mesh) noexcept {
    if (mesh.vertices.empty()) {
        return {};
    }
    Bounds b{mesh.vertices.front(), mesh.vertices.front()};
    for (const Vector3& v : mesh.vertices) {
        b.min = {std::min(b.min.x, v.x), std::min(b.min.y, v.y), std::min(b.min.z, v.z)};
        b.max = {std::max(b.max.x, v.x), std::max(b.max.y, v.y), std::max(b.max.z, v.z)};
    }
    return b;
}

Vector3 triangle_cross(const TriangleMesh& mesh, const Triangle& t) noexcept {
    const Vector3& a = mesh.vertices[t[0]];
    const Vector3& b = mesh.vertices[t[1]];
    const Vector3& c = mesh.vertices[t[2]];
    return cross(b - a, c - a);
}

double triangle_area(const TriangleMesh& mesh, const Triangle& t) noexcept {
    return 0.5 * norm(triangle_cross(mesh, t));
}

double surface_area(const TriangleMesh& mesh) noexcept {
    double sum = 0.0;
    for (const Triangle& t : mesh.triangles) {
        sum += triangle_area(mesh, t);
    }
    return sum;
}

namespace {

// Six times the signed volume of the tetrahedron (origin, a, b, c) with the
// coordinates taken relative to `origin`.
double tetra6(const Vector3& origin, const Vector3& a, const Vector3& b, const Vector3& c) {
    return dot(a - origin, cross(b - origin, c - origin));
}

} // namespace

double signed_volume(const TriangleMesh& mesh) noexcept {
    const Vector3 origin = bounds(mesh).centre();
    double sum = 0.0;
    for (const Triangle& t : mesh.triangles) {
        sum += tetra6(origin, mesh.vertices[t[0]], mesh.vertices[t[1]], mesh.vertices[t[2]]);
    }
    return sum / 6.0;
}

Vector3 volume_centroid(const TriangleMesh& mesh) noexcept {
    const Vector3 origin = bounds(mesh).centre();
    double volume6 = 0.0;
    Vector3 moment;
    for (const Triangle& t : mesh.triangles) {
        const Vector3 a = mesh.vertices[t[0]] - origin;
        const Vector3 b = mesh.vertices[t[1]] - origin;
        const Vector3 c = mesh.vertices[t[2]] - origin;
        const double v6 = dot(a, cross(b, c));
        volume6 += v6;
        // The centroid of the tetrahedron (0, a, b, c) is (a + b + c) / 4.
        moment += (a + b + c) * v6;
    }
    if (volume6 == 0.0) {
        return origin;
    }
    return origin + moment / (4.0 * volume6);
}

void append(TriangleMesh& mesh, const TriangleMesh& other) {
    const std::size_t offset = mesh.vertices.size();
    if (offset + other.vertices.size() > 0xffffffffU) {
        throw std::length_error("TriangleMesh: more than 2^32 - 1 vertices");
    }
    const auto base = static_cast<std::uint32_t>(offset);
    mesh.vertices.insert(mesh.vertices.end(), other.vertices.begin(), other.vertices.end());
    mesh.triangles.reserve(mesh.triangles.size() + other.triangles.size());
    for (const Triangle& t : other.triangles) {
        mesh.triangles.push_back({t[0] + base, t[1] + base, t[2] + base});
    }
}

} // namespace orrery::geom
