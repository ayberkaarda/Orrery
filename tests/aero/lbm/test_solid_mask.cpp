#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <stdexcept>

#include "orrery/aero/lbm/lbm2d.h"
#include "orrery/aero/lbm/solid_mask.h"
#include "orrery/geom/naca.h"
#include "orrery/geom/primitives.h"
#include "orrery/geom/voxel_grid.h"

using namespace orrery;
using namespace orrery::aero::lbm;

TEST_CASE("A circle mask covers about pi r^2 nodes", "[aero][lbm][mask]") {
    const SolidMask2D mask = circle_mask(100, 80, 40.3, 39.7, 12.0);
    CHECK(mask.nx == 100);
    CHECK(mask.ny == 80);
    const double area = std::numbers::pi * 144.0;
    CHECK(static_cast<double>(mask.count()) == Catch::Approx(area).epsilon(0.02));
    CHECK(mask.at(40, 40));
    CHECK_FALSE(mask.at(0, 0));
    CHECK_FALSE(mask.at(53, 40));
}

TEST_CASE("A NACA section mask matches the section area", "[aero][lbm][mask]") {
    const geom::Naca4 airfoil = *geom::Naca4::from_designation("0012");
    SectionPlacement placement;
    placement.leading_edge_x = 20.0;
    placement.leading_edge_y = 40.0;
    placement.chord = 120.0;
    const SolidMask2D level = naca_mask(180, 80, airfoil, placement);
    const double area = airfoil.section_area() * 120.0 * 120.0;
    CHECK(static_cast<double>(level.count()) == Catch::Approx(area).epsilon(0.03));
    CHECK(level.at(50, 40));
    CHECK_FALSE(level.at(50, 50));

    // Nose-up pitch moves the trailing edge to -y.
    placement.angle_of_attack = 10.0 * std::numbers::pi / 180.0;
    const auto section = place_section(airfoil, placement);
    CHECK(section.front().z < 40.0 - 120.0 * std::sin(placement.angle_of_attack) + 1.0);
    const SolidMask2D pitched = naca_mask(180, 80, airfoil, placement);
    CHECK(static_cast<double>(pitched.count()) == Catch::Approx(area).epsilon(0.03));
    // Mid-chord: (79.1, 29.6) on the pitched chord line, 4 cells below the
    // level section.
    CHECK(pitched.at(79, 30));
    CHECK_FALSE(level.at(79, 30));
}

TEST_CASE("A voxelized cylinder slice agrees with the analytic circle", "[aero][lbm][mask]") {
    // Cylinder of radius 1 m along z through the voxel grid; one lattice node
    // per voxel (spacing 0.1 m).
    const geom::TriangleMesh mesh =
        geom::cylinder_profile(1.0, 1.0).mesh(geom::RevolutionResolution{128, 0.05});
    geom::GridSpec spec;
    spec.origin = geom::Vector3{-2.0, -2.0, 0.2};
    spec.spacing = 0.1;
    spec.nx = 40;
    spec.ny = 40;
    spec.nz = 3;
    const geom::VoxelGrid voxels = geom::voxelize(mesh, spec);
    const SolidMask2D slice = mask_from_voxels(voxels, VoxelSlice::xy, 1);
    REQUIRE(slice.nx == 40);
    REQUIRE(slice.ny == 40);
    // Node (i, j) of the slice is the voxel centre -2 + 0.1 (i + 1/2): the
    // circle of radius 10 nodes is centred at node 19.5.
    const SolidMask2D circle = circle_mask(40, 40, 19.5, 19.5, 10.0);
    std::size_t differ = 0;
    for (std::size_t c = 0; c < circle.solid.size(); ++c) {
        differ += circle.solid[c] != slice.solid[c] ? 1U : 0U;
    }
    CHECK(slice.count() > 290);
    CHECK(static_cast<double>(differ) < 0.03 * static_cast<double>(circle.count()));

    // xz slice: lattice y is the grid's z axis.
    const SolidMask2D side = mask_from_voxels(voxels, VoxelSlice::xz, 20);
    CHECK(side.ny == 3);
    CHECK(side.at(20, 1));
    CHECK_THROWS_AS(mask_from_voxels(voxels, VoxelSlice::xy, 3), std::invalid_argument);
    CHECK_THROWS_AS(mask_from_voxels(voxels, VoxelSlice::xz, 40), std::invalid_argument);
}

TEST_CASE("A voxel grid whose cell count wraps is rejected before indexing", "[aero][lbm][mask]") {
    // 4 x 4 x (2^60 + 1) cells: the unchecked product nx ny nz wraps to 16 on
    // a 64-bit size_t, which a 16-element vector would match.
    static_assert(sizeof(std::size_t) >= 8);
    geom::VoxelGrid voxels;
    voxels.spec.nx = 4;
    voxels.spec.ny = 4;
    voxels.spec.nz = (std::size_t{1} << 60U) + 1;
    voxels.solid.assign(16, std::uint8_t{0});
    CHECK_THROWS_AS(mask_from_voxels(voxels, VoxelSlice::xy, 0), std::invalid_argument);
    // Same with the wrap in the x-y product.
    voxels.spec.nx = (std::size_t{1} << 62U) + 4;
    voxels.spec.ny = 4;
    voxels.spec.nz = 1;
    CHECK_THROWS_AS(mask_from_voxels(voxels, VoxelSlice::xz, 0), std::invalid_argument);
    // A zero dimension.
    voxels.spec.nx = 0;
    voxels.solid.clear();
    CHECK_THROWS_AS(mask_from_voxels(voxels, VoxelSlice::xy, 0), std::invalid_argument);
}

TEST_CASE("Boundary links are the fluid-to-solid lattice links", "[aero][lbm][mask]") {
    Lbm2DConfig config;
    config.nx = 9;
    config.ny = 9;
    config.x_boundary = XBoundary::periodic;
    config.bottom = Wall{WallKind::periodic, 0.0};
    config.top = Wall{WallKind::periodic, 0.0};
    SolidMask2D mask = fluid_mask(9, 9);
    mask.solid[4 + 9 * 4] = 1; // one solid node: each of its 8 neighbours has one link
    const auto links = boundary_links(config, mask);
    REQUIRE(links.size() == 8);
    for (const BoundaryLink& link : links) {
        const std::size_t x = link.cell % 9;
        const std::size_t y = link.cell / 9;
        const auto dx = static_cast<int>(x) - 4;
        const auto dy = static_cast<int>(y) - 4;
        // The solid node lies at x - c_j.
        CHECK(dx == aero::lbm::d2q9::cx[link.direction]);
        CHECK(dy == aero::lbm::d2q9::cy[link.direction]);
    }
    CHECK_THROWS_AS(fluid_mask(0, 4), std::invalid_argument);
}
