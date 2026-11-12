#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "orrery/geom/triangle_mesh.h"

namespace orrery::geom {

// Signed solid angle of triangle (a, b, c) seen from p (Van Oosterom and
// Strackee 1983): positive when the triangle's right-hand normal points away
// from p. In (-2 pi, 2 pi]; 0 when p is a corner. Evaluated on unit vectors,
// so it does not depend on the triangle's scale.
[[nodiscard]] double solid_angle(const Vector3& p, const Vector3& a, const Vector3& b,
                                 const Vector3& c) noexcept;

// Generalized winding number (Jacobson, Kavan and Sorkine-Hornung 2013): the
// sum of the solid angles of all triangles over 4 pi. 1 inside and 0 outside
// a closed outward surface; for meshes with holes or overlaps it degrades
// smoothly, so thresholding at 1/2 still separates inside from outside.
// Direct sum over every triangle (reference).
[[nodiscard]] double winding_number(const TriangleMesh& mesh, const Vector3& p) noexcept;

// Bounding volume hierarchy over a mesh's triangles answering winding-number
// and closest-distance queries in about O(log n) per point.
//
// Winding number: the hierarchical evaluation of Barill et al. (2018). A
// node whose area-weighted centre c is farther than beta times its radius r
// (largest distance from c to a vertex of the node) is replaced by its
// dipole: (sum of a_t n_t) . (c - p) / (4 pi |c - p|^3).
// APPROXIMATION (G5): first-order far field - the error decays like (r / d)^2
// relative to the node's contribution; with beta = 2 it stays far below the
// 1/2 decision threshold (tested against the direct sum).
class MeshTree {
public:
    // The mesh must have passed validate_mesh and must outlive the tree.
    explicit MeshTree(const TriangleMesh& mesh, double beta = 2.0, std::size_t leaf_size = 8);

    [[nodiscard]] double winding_number(const Vector3& p) const noexcept;

    // Distance from p to the closest point of the surface, or max_distance
    // when no triangle is closer than that.
    [[nodiscard]] double distance(const Vector3& p, double max_distance) const noexcept;

    [[nodiscard]] std::size_t node_count() const noexcept { return nodes_.size(); }

private:
    struct Node {
        Bounds box;
        Vector3 centre;
        // Sum of the triangles' area vectors divided by scale_^2.
        Vector3 area_normal;
        double radius = 0.0;
        std::uint32_t first = 0;
        std::uint32_t count = 0;
        std::uint32_t left = 0;
        std::uint32_t right = 0;
        bool leaf = true;
    };

    std::uint32_t build(std::uint32_t first, std::uint32_t count, std::size_t leaf_size);

    const TriangleMesh* mesh_;
    double beta_;
    // Power of two near the mesh extent (see winding_number.cpp).
    double scale_;
    std::vector<std::uint32_t> order_;
    std::vector<Node> nodes_;
};

// Closest point of triangle (a, b, c) to p (Ericson, Real-Time Collision
// Detection, 5.1.5). A triangle without area (repeated or collinear corners)
// is treated as its edges, so the result is always a point of the triangle.
[[nodiscard]] Vector3 closest_point_on_triangle(const Vector3& p, const Vector3& a,
                                                const Vector3& b, const Vector3& c) noexcept;

} // namespace orrery::geom
