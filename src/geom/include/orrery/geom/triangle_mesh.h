#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "orrery/geom/vector3.h"

namespace orrery::geom {

using Triangle = std::array<std::uint32_t, 3>;

// Indexed triangle mesh. Triangles are counter-clockwise seen from outside
// (outward normal by the right-hand rule) for a correctly oriented closed
// surface. Coordinates are metres once the unit scale has been applied.
struct TriangleMesh {
    std::vector<Vector3> vertices;
    std::vector<Triangle> triangles;
};

struct Bounds {
    Vector3 min;
    Vector3 max;

    [[nodiscard]] constexpr Vector3 extent() const noexcept { return max - min; }
    [[nodiscard]] constexpr Vector3 centre() const noexcept { return (min + max) * 0.5; }
};

// Axis-aligned bounds of the vertices (all zero for an empty mesh).
[[nodiscard]] Bounds bounds(const TriangleMesh& mesh) noexcept;

// Area-weighted normal (cross product of two edges, length twice the area).
[[nodiscard]] Vector3 triangle_cross(const TriangleMesh& mesh, const Triangle& t) noexcept;
[[nodiscard]] double triangle_area(const TriangleMesh& mesh, const Triangle& t) noexcept;

// Sum of the triangle areas.
[[nodiscard]] double surface_area(const TriangleMesh& mesh) noexcept;

// Enclosed volume by the divergence theorem (sum of signed tetrahedra about
// the bounds centre). Positive for a closed, outward-oriented surface;
// meaningful only for closed meshes.
[[nodiscard]] double signed_volume(const TriangleMesh& mesh) noexcept;

// Centroid of the enclosed volume (same tetrahedra, volume-weighted). Equal
// to the centre of mass only under uniform density. Returns the bounds centre
// when the signed volume is zero.
[[nodiscard]] Vector3 volume_centroid(const TriangleMesh& mesh) noexcept;

// Appends `other` to `mesh`, offsetting its indices.
void append(TriangleMesh& mesh, const TriangleMesh& other);

} // namespace orrery::geom
