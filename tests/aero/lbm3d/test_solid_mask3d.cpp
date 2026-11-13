#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <stdexcept>

#include "orrery/aero/lbm3d/solid_mask3d.h"
#include "orrery/geom/primitives.h"
#include "orrery/geom/voxel_grid.h"

using namespace orrery;
using namespace orrery::aero::lbm;
using Catch::Approx;

TEST_CASE("Lattice cell counts are checked without wrapping", "[aero][lbm3d]") {
    CHECK(lattice_cells_3d(4, 5, 6) == std::size_t{120});
    CHECK_FALSE(lattice_cells_3d(0, 5, 6));
    // 4 x 4 x (2^60 + 1) wraps to 16 in 64-bit arithmetic.
    const std::size_t huge = (std::size_t{1} << 60U) + 1;
    CHECK_FALSE(lattice_cells_3d(4, 4, huge));
    CHECK_FALSE(lattice_cells_3d(std::numeric_limits<std::size_t>::max(), 2, 1));
    CHECK_FALSE(lattice_cells_3d(1024, 1024, 1024)); // above geom::max_grid_cells
    CHECK_THROWS_AS(fluid_mask_3d(4, 4, huge), std::invalid_argument);
    CHECK_THROWS_AS(fluid_mask_3d(4, 0, 4), std::invalid_argument);
}

TEST_CASE("Sphere and box masks", "[aero][lbm3d]") {
    const SolidMask3D sphere = sphere_mask(40, 40, 40, {19.5, 19.5, 19.5}, 12.0);
    const double volume = 4.0 / 3.0 * std::numbers::pi * 12.0 * 12.0 * 12.0;
    CHECK(static_cast<double>(sphere.count()) == Approx(volume).epsilon(0.02));
    CHECK(sphere.at(19, 19, 19));
    CHECK_FALSE(sphere.at(0, 0, 0));

    const SolidMask3D plate = box_mask(10, 12, 14, {4.0, 2.0, 3.0}, {4.5, 9.0, 10.0});
    CHECK(plate.count() == std::size_t{64}); // 1 x 8 x 8 nodes
    CHECK(plate.at(4, 2, 3));
    CHECK_FALSE(plate.at(5, 2, 3));
}

TEST_CASE("Voxel grids are placed into a lattice with checked sizes", "[aero][lbm3d]") {
    const geom::TriangleMesh mesh = geom::sphere_profile(4.0).mesh({48, 0.1});
    geom::GridSpec spec;
    spec.origin = geom::Vector3{-5.0, -5.0, -5.0};
    spec.spacing = 1.0;
    spec.nx = 10;
    spec.ny = 10;
    spec.nz = 10;
    const geom::VoxelGrid voxels = geom::voxelize(mesh, spec);
    SolidMask3D mask = fluid_mask_3d(20, 16, 14);
    place_voxels(mask, voxels, {3, 2, 1});
    CHECK(mask.count() == voxels.solid_count());
    // Cell (5, 5, 5) of the grid has its centre at (0.5, 0.5, 0.5): inside.
    CHECK(mask.at(8, 7, 6));

    // A grid that does not fit at this offset.
    SolidMask3D small = fluid_mask_3d(12, 12, 12);
    CHECK_THROWS_AS(place_voxels(small, voxels, {3, 0, 0}), std::invalid_argument);
    // A spec whose cell count wraps (4 x 4 x (2^60 + 1) -> 16) with a
    // 16-byte voxel vector: refused before any index is formed.
    geom::VoxelGrid forged;
    forged.spec.nx = 4;
    forged.spec.ny = 4;
    forged.spec.nz = (std::size_t{1} << 60U) + 1;
    forged.solid.assign(16, std::uint8_t{1});
    CHECK_THROWS_AS(place_voxels(mask, forged, {0, 0, 0}), std::invalid_argument);
    // A vector that does not match its spec.
    geom::VoxelGrid short_vector = voxels;
    short_vector.solid.pop_back();
    CHECK_THROWS_AS(place_voxels(mask, short_vector, {0, 0, 0}), std::invalid_argument);
}
