#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <random>
#include <stdexcept>

#include "orrery/geom/primitives.h"
#include "orrery/geom/voxel_grid.h"
#include "orrery/geom/winding_number.h"

using namespace orrery::geom;

namespace {

constexpr double pi = std::numbers::pi;

} // namespace

TEST_CASE("solid angle of an octant triangle", "[geom][voxel]") {
    const Vector3 o{};
    CHECK(std::abs(solid_angle(o, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}) - pi / 2.0) < 1e-15);
    CHECK(std::abs(solid_angle(o, {1, 0, 0}, {0, 0, 1}, {0, 1, 0}) + pi / 2.0) < 1e-15);
    const Vector3 p{0.2, 0.3, 0.1};
    const Vector3 c = closest_point_on_triangle(p, {1, 0, 0}, {0, 1, 0}, {0, 0, 1});
    // The foot of the perpendicular onto x + y + z = 1.
    const double s = (1.0 - (p.x + p.y + p.z)) / 3.0;
    CHECK(std::abs(c.x - (p.x + s)) < 1e-15);
    CHECK(std::abs(c.y - (p.y + s)) < 1e-15);
    CHECK(std::abs(c.z - (p.z + s)) < 1e-15);
    CHECK(closest_point_on_triangle({-1, -1, -1}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}).x >= 0.0);
}

TEST_CASE("winding number of a closed mesh is 1 inside and 0 outside", "[geom][voxel]") {
    const TriangleMesh sphere = sphere_profile(1.0).mesh({48, pi / 24.0});
    CHECK(std::abs(winding_number(sphere, {0.1, -0.2, 0.3}) - 1.0) < 1e-12);
    CHECK(std::abs(winding_number(sphere, {1.5, 0.0, 0.2})) < 1e-12);
    CHECK(std::abs(winding_number(sphere, {0.0, 0.0, 30.0})) < 1e-12);

    // Hierarchical evaluation against the direct sum, near and far.
    const MeshTree tree(sphere);
    std::mt19937_64 rng(12345);
    std::uniform_real_distribution<double> coord(-2.0, 2.0);
    double worst = 0.0;
    int disagree = 0;
    for (int i = 0; i < 2000; ++i) {
        const Vector3 p{coord(rng), coord(rng), coord(rng)};
        const double fast = tree.winding_number(p);
        const double exact = winding_number(sphere, p);
        worst = std::max(worst, std::abs(fast - exact));
        disagree += (fast >= 0.5) != (exact >= 0.5) ? 1 : 0;
    }
    INFO("largest hierarchical error " << worst);
    // First-order far field with beta = 2: a few percent at worst, far from
    // the 1/2 threshold, and never a different classification.
    CHECK(worst < 0.05);
    CHECK(disagree == 0);

    // An open mesh (one cap removed) still classifies its interior.
    TriangleMesh open = sphere;
    open.triangles.resize(open.triangles.size() - 48);
    CHECK(winding_number(open, {0.0, 0.0, 0.0}) > 0.5);
    CHECK(winding_number(open, {0.0, 0.0, -1.5}) < 0.5);
}

TEST_CASE("sphere voxelization classifies at least 99.9 percent of cells", "[geom][voxel]") {
    const double radius = 1.0;
    const TriangleMesh sphere = sphere_profile(radius).mesh({128, pi / 64.0});
    const GridSpec spec = grid_around(bounds(sphere), 2.4 / 48.0, 2);
    const VoxelGrid grid = voxelize(sphere, spec);
    std::size_t agree = 0;
    std::size_t inside = 0;
    for (std::size_t k = 0; k < spec.nz; ++k) {
        for (std::size_t j = 0; j < spec.ny; ++j) {
            for (std::size_t i = 0; i < spec.nx; ++i) {
                const bool truth = norm(spec.centre(i, j, k)) < radius;
                const bool solid = grid.solid[spec.index(i, j, k)] != 0;
                agree += truth == solid ? 1U : 0U;
                inside += truth ? 1U : 0U;
            }
        }
    }
    const double accuracy = static_cast<double>(agree) / static_cast<double>(spec.cells());
    INFO("cells " << spec.cells() << ", inside " << inside << ", accuracy " << accuracy);
    CHECK(accuracy >= 0.999);
    CHECK(grid.solid_count() > 0);
    // Solid volume approaches the sphere's.
    const double voxel_volume = static_cast<double>(grid.solid_count()) * std::pow(spec.spacing, 3);
    CHECK(std::abs(voxel_volume / (4.0 / 3.0 * pi) - 1.0) < 0.02);
}

TEST_CASE("narrow-band signed distance of a sphere", "[geom][voxel]") {
    const double radius = 0.8;
    const RevolutionResolution res{96, pi / 48.0};
    const TriangleMesh sphere = sphere_profile(radius).mesh(res);
    const GridSpec spec = grid_around(bounds(sphere), 0.05, 4);
    const VoxelGrid grid = voxelize(sphere, spec);
    const SignedDistanceField sdf = signed_distance_band(sphere, grid, 3.0);
    // The facets lie inside the sphere by at most the sagitta of a facet
    // (vertex spacing 2 pi / 96 in both directions).
    const double sagitta = radius * (1.0 - std::cos(2.0 * pi / 96.0));
    std::size_t in_band = 0;
    double worst = 0.0;
    for (std::size_t k = 0; k < spec.nz; ++k) {
        for (std::size_t j = 0; j < spec.ny; ++j) {
            for (std::size_t i = 0; i < spec.nx; ++i) {
                const std::size_t c = spec.index(i, j, k);
                const double exact = norm(spec.centre(i, j, k)) - radius;
                if (sdf.in_band[c] != 0) {
                    ++in_band;
                    worst = std::max(worst, std::abs(sdf.distance[c] - exact));
                } else {
                    CHECK(std::abs(sdf.distance[c]) == sdf.band);
                    CHECK(std::abs(exact) > sdf.band - sagitta);
                }
            }
        }
    }
    INFO("in band " << in_band << ", worst error " << worst);
    CHECK(in_band > 0);
    CHECK(worst <= sagitta);
}

TEST_CASE("grid limits are enforced", "[geom][voxel]") {
    const Bounds box{{0, 0, 0}, {1, 1, 1}};
    CHECK_THROWS_AS(grid_around(box, 0.0, 0), std::invalid_argument);
    CHECK_THROWS_AS(grid_around(box, 1e-4, 0), std::invalid_argument);
    const GridSpec g = grid_around(box, 0.25, 1);
    CHECK(g.nx == 6);
    CHECK(std::abs(g.origin.x + 0.25) < 1e-15);
    VoxelGrid wrong;
    wrong.spec = g;
    CHECK_THROWS_AS(signed_distance_band(cube_mesh(1.0), wrong), std::invalid_argument);
}
