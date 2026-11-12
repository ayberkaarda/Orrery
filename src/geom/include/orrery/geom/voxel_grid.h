#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "orrery/geom/triangle_mesh.h"

namespace orrery::geom {

// Regular grid of cubic cells; cell (i, j, k) has its centre at
// origin + spacing (i + 1/2, j + 1/2, k + 1/2) and the linear index
// i + nx (j + ny k).
struct GridSpec {
    Vector3 origin;
    double spacing = 1.0;
    std::size_t nx = 0;
    std::size_t ny = 0;
    std::size_t nz = 0;

    [[nodiscard]] std::size_t cells() const noexcept { return nx * ny * nz; }
    [[nodiscard]] std::size_t index(std::size_t i, std::size_t j, std::size_t k) const noexcept {
        return i + nx * (j + ny * k);
    }
    [[nodiscard]] Vector3 centre(std::size_t i, std::size_t j, std::size_t k) const noexcept;
};

// Largest grid accepted (2^27 cells, 128 MB per byte field).
inline constexpr std::size_t max_grid_cells = std::size_t{1} << 27U;

// Grid covering `box` with `padding` extra cells on every side. Throws
// std::invalid_argument for a non-positive or non-finite spacing, bounds that
// are not finite or not ordered (min <= max), or a grid above max_grid_cells.
[[nodiscard]] GridSpec grid_around(const Bounds& box, double spacing, std::size_t padding);

// Solid/fluid classification: a cell is solid when the generalized winding
// number of the mesh at its centre is at least `threshold` (1/2), which also
// classifies meshes with small holes or overlaps sensibly.
// APPROXIMATION: one sample per cell (the centre) - the staircase wall is
// what an LBM solver with bounce-back sees (A4 in docs/physics.md).
struct VoxelGrid {
    GridSpec spec;
    std::vector<std::uint8_t> solid;
    std::vector<double> winding;

    [[nodiscard]] std::size_t solid_count() const noexcept;
};

// Throws std::invalid_argument unless every dimension is at least 1, the
// cell count (computed without wrapping) is at most max_grid_cells, the origin
// is finite, the spacing positive and finite, and the threshold finite.
[[nodiscard]] VoxelGrid voxelize(const TriangleMesh& mesh, const GridSpec& spec,
                                 double threshold = 0.5);

// Narrow-band signed distance (negative inside): cells within `band_cells`
// cells of the surface hold the exact distance to the closest triangle with
// the sign of the voxel classification; the others hold -band or +band and
// in_band = 0. Throws std::invalid_argument for a grid spec voxelize would
// reject, voxel vectors that do not match it, or a band that is not positive
// and finite (also after scaling by the spacing).
struct SignedDistanceField {
    GridSpec spec;
    double band = 0.0;
    std::vector<double> distance;
    std::vector<std::uint8_t> in_band;
};

[[nodiscard]] SignedDistanceField
signed_distance_band(const TriangleMesh& mesh, const VoxelGrid& voxels, double band_cells = 3.0);

} // namespace orrery::geom
