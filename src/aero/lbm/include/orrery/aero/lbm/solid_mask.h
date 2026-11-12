#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "orrery/geom/naca.h"
#include "orrery/geom/voxel_grid.h"

// Solid/fluid classification of a 2D lattice. Node (x, y) sits at lattice
// position (x, y) and has the linear index x + nx y. Masks come from a signed
// distance function evaluated at the nodes, from a polygon (the NACA section
// of orrery_geom), or from a slice of an orrery_geom VoxelGrid (generalized
// winding-number voxelization of a triangle mesh).
// APPROXIMATION A4: one sample per node gives a staircase wall.
namespace orrery::aero::lbm {

struct SolidMask2D {
    std::size_t nx = 0;
    std::size_t ny = 0;
    std::vector<std::uint8_t> solid; // 1 = solid, 0 = fluid

    [[nodiscard]] bool at(std::size_t x, std::size_t y) const noexcept {
        return solid[x + nx * y] != 0;
    }
    [[nodiscard]] std::size_t count() const noexcept;
};

// All-fluid mask. Throws std::invalid_argument for a zero dimension or more
// than geom::max_grid_cells nodes.
[[nodiscard]] SolidMask2D fluid_mask(std::size_t nx, std::size_t ny);

// A node is solid where sdf(x, y) <= 0 (negative inside).
[[nodiscard]] SolidMask2D mask_from_sdf(std::size_t nx, std::size_t ny,
                                        const std::function<double(double, double)>& sdf);

// Circle of radius r (lattice units) centred at (cx, cy).
[[nodiscard]] SolidMask2D circle_mask(std::size_t nx, std::size_t ny, double cx, double cy,
                                      double radius);

// Nodes inside a closed polygon (even-odd rule) given in lattice coordinates.
[[nodiscard]] SolidMask2D polygon_mask(std::size_t nx, std::size_t ny,
                                       const std::vector<geom::SectionPoint>& polygon);

// NACA 4-digit section with its leading edge at (x, y), chord `chord` cells,
// pitched nose-up by `angle_of_attack` radians for a free stream along +x
// (the trailing edge moves to -y), sampled with geom::Naca4::section(points).
struct SectionPlacement {
    double leading_edge_x = 0.0;
    double leading_edge_y = 0.0;
    double chord = 40.0;
    double angle_of_attack = 0.0;
    int points = 200;
};
[[nodiscard]] std::vector<geom::SectionPoint> place_section(const geom::Naca4& airfoil,
                                                            const SectionPlacement& placement);
[[nodiscard]] SolidMask2D naca_mask(std::size_t nx, std::size_t ny, const geom::Naca4& airfoil,
                                    const SectionPlacement& placement);

// A plane of voxels: lattice x is grid i; lattice y is grid j (xy plane at
// k = index) or grid k (xz plane at j = index, the plane of the NACA wing
// sections of orrery_geom). Throws std::invalid_argument for an index outside
// the grid, a zero dimension, a cell count (computed without wrapping) above
// geom::max_grid_cells, or a voxel vector that does not match its spec.
enum class VoxelSlice : std::uint8_t { xy, xz };
[[nodiscard]] SolidMask2D mask_from_voxels(const geom::VoxelGrid& voxels, VoxelSlice plane,
                                           std::size_t index);

} // namespace orrery::aero::lbm
