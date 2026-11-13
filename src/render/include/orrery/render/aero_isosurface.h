#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <glm/ext/vector_double3.hpp>

#include "orrery/aero/compute/compute_backend.h"
#include "orrery/core/result.h"
#include "orrery/render/aero_field.h"

// Iso-surfaces by marching cubes (Lorensen and Cline 1987;
// docs/aero-visualisation.md, "Q-criterion iso-surface").
//
// The region `value > level` is inside. Each cell edge whose ends lie on
// different sides carries one vertex, placed by linear interpolation of the
// node values; vertices are shared through their edge, so the mesh is
// indexed. The triangulation of every one of the 256 corner configurations
// is generated (not tabulated by hand) from one rule applied per cube face:
// on a face whose diagonal corners alternate (the ambiguous case) the inside
// corners are kept apart. Neighbouring cells see the same four corners on a
// shared face and therefore make the same choice, so a surface that does not
// reach the grid boundary is closed and consistently oriented (no cracks).
// Triangles are counter-clockwise seen from outside (from lower values);
// vertex normals are minus the normalised interpolated value gradient.
//
// marching_cubes() (double) is the reference of the GPU kernel
// aero_vis_marching_cubes (binary32).
namespace orrery::render {

struct IsoSurface {
    std::vector<glm::dvec3> positions;
    std::vector<glm::dvec3> normals;
    std::vector<std::uint32_t> indices; // three per triangle
    // Grid edge of each vertex, 3 * node + axis (the edge from that node
    // towards +axis), sorted ascending: positions[i] lies on edge edges[i].
    std::vector<std::uint64_t> edges;

    [[nodiscard]] std::size_t triangle_count() const noexcept { return indices.size() / 3; }
};

// Empty for a 2D grid or one without values.
[[nodiscard]] IsoSurface marching_cubes(const ScalarGrid& field, double level);

// The same surface from the GPU kernel: identical vertices (edges) and
// triangles, positions in binary32. Fails when the triangles exceed
// max_triangles; `backend` must provide kernel_marching_cubes.
[[nodiscard]] core::Result<IsoSurface, std::string>
marching_cubes_gpu(aero::compute::IComputeBackend& backend, const ScalarGrid& field, double level,
                   std::size_t max_triangles = std::size_t{1} << 21U);

// Sum of the triangle areas.
[[nodiscard]] double surface_area(const IsoSurface& surface);

// Edge use of an indexed triangle mesh: a closed, consistently oriented
// surface has every edge in exactly two triangles, once in each direction.
struct MeshTopology {
    std::size_t edges = 0;
    std::size_t boundary_edges = 0;       // in one triangle
    std::size_t non_manifold_edges = 0;   // in more than two
    std::size_t misoriented_edges = 0;    // in two triangles with the same direction
    std::size_t degenerate_triangles = 0; // a repeated vertex index

    [[nodiscard]] bool closed() const noexcept {
        return boundary_edges == 0 && non_manifold_edges == 0 && misoriented_edges == 0;
    }
};
[[nodiscard]] MeshTopology topology(const IsoSurface& surface);

// The corner configuration table: for each of the 256 cases (bit c set when
// corner c is inside, corner c at offset (c & 1, (c >> 1) & 1, (c >> 2) & 1)),
// the cell edges of each triangle (edge e: 0-3 along x, 4-7 along y, 8-11
// along z, see aero_isosurface.cpp), three per triangle.
[[nodiscard]] const std::vector<std::vector<std::uint8_t>>& marching_cubes_table();

} // namespace orrery::render
