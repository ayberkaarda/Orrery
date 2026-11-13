#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include "orrery/geom/voxel_grid.h"

// Solid/fluid classification of a 3D lattice. Node (x, y, z) sits at lattice
// position (x, y, z) and has the linear index x + nx (y + ny z), the index of
// geom::GridSpec. Masks come from a signed distance function evaluated at the
// nodes, from analytic shapes (sphere, box), or from an orrery_geom VoxelGrid
// (generalized winding-number voxelization of a triangle mesh, Phase 6).
// APPROXIMATION A4: one sample per node gives a staircase wall. A mask may
// also carry the signed distance to the wall at its nodes (lattice units,
// negative inside), which the interpolated bounce-back uses to place the
// wall on each link (SolidWall::interpolated in lbm3d.h).
namespace orrery::aero::lbm {

// nx * ny * nz computed without wrapping; empty when a dimension is zero or
// the product exceeds geom::max_grid_cells.
[[nodiscard]] std::optional<std::size_t> lattice_cells_3d(std::size_t nx, std::size_t ny,
                                                          std::size_t nz) noexcept;

struct SolidMask3D {
    std::size_t nx = 0;
    std::size_t ny = 0;
    std::size_t nz = 0;
    std::vector<std::uint8_t> solid; // 1 = solid, 0 = fluid
    // Signed distance to the wall at every node, lattice units, negative
    // inside; empty when the mask has none.
    std::vector<double> distance;

    [[nodiscard]] std::size_t index(std::size_t x, std::size_t y, std::size_t z) const noexcept {
        return x + nx * (y + ny * z);
    }
    [[nodiscard]] bool at(std::size_t x, std::size_t y, std::size_t z) const noexcept {
        return solid[index(x, y, z)] != 0;
    }
    [[nodiscard]] std::size_t count() const noexcept;
};

// All-fluid mask. Throws std::invalid_argument for a zero dimension or more
// than geom::max_grid_cells nodes (product checked without wrapping).
[[nodiscard]] SolidMask3D fluid_mask_3d(std::size_t nx, std::size_t ny, std::size_t nz);

// A node is solid where sdf(x, y, z) <= 0 (negative inside); the values are
// kept as the mask's wall distance, so `sdf` should be a signed distance in
// lattice units.
[[nodiscard]] SolidMask3D
mask_from_sdf_3d(std::size_t nx, std::size_t ny, std::size_t nz,
                 const std::function<double(double, double, double)>& sdf);

// Sphere of radius r (lattice units) centred at `centre`, with its exact
// signed distance.
[[nodiscard]] SolidMask3D sphere_mask(std::size_t nx, std::size_t ny, std::size_t nz,
                                      const std::array<double, 3>& centre, double radius);

// Nodes with lo <= position <= hi on every axis (a flat plate, a duct
// wall), with the exact signed distance of the box.
[[nodiscard]] SolidMask3D box_mask(std::size_t nx, std::size_t ny, std::size_t nz,
                                   const std::array<double, 3>& lo,
                                   const std::array<double, 3>& hi);

// Marks the solid voxels of `voxels` in `mask`, voxel (i, j, k) at node
// (i + offset[0], j + offset[1], k + offset[2]). Throws std::invalid_argument
// for a grid whose cell count (checked without wrapping) is zero or above
// geom::max_grid_cells, a voxel vector that does not match its spec, or a
// placement that does not fit in the mask.
void place_voxels(SolidMask3D& mask, const geom::VoxelGrid& voxels,
                  const std::array<std::size_t, 3>& offset);

// Copies the narrow-band signed distance `sdf` (metres, its grid placed like
// place_voxels at `offset`) into the mask's wall distance in lattice units
// (divided by the grid spacing). Nodes outside the grid, and a mask without
// distances, start at +far_distance (fluid far from the wall). Throws
// std::invalid_argument under the conditions of place_voxels and for
// distance vectors that do not match the grid.
inline constexpr double far_distance = 1e9;
void place_distance(SolidMask3D& mask, const geom::SignedDistanceField& sdf,
                    const std::array<std::size_t, 3>& offset);

} // namespace orrery::aero::lbm
