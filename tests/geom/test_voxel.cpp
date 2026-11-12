#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <random>
#include <stdexcept>
#include <utility>

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

TEST_CASE("signed distance rejects grids whose cell count overflows or is out of range",
          "[geom][voxel]") {
    const TriangleMesh cube = cube_mesh(1.0);
    // 2^63 * 2 * 1 wraps to zero cells: empty vectors would match the spec.
    VoxelGrid wrapped;
    wrapped.spec.nx = std::size_t{1} << 63U;
    wrapped.spec.ny = 2;
    wrapped.spec.nz = 1;
    CHECK_THROWS_AS(signed_distance_band(cube, wrapped), std::invalid_argument);
    // Zero cells along one axis: nothing to classify.
    VoxelGrid empty;
    empty.spec.nx = 0;
    empty.spec.ny = 4;
    empty.spec.nz = 4;
    CHECK_THROWS_AS(signed_distance_band(cube, empty), std::invalid_argument);
    // No overflow, but above max_grid_cells.
    VoxelGrid oversized;
    oversized.spec.nx = std::size_t{1} << 14U;
    oversized.spec.ny = std::size_t{1} << 14U;
    oversized.spec.nz = 1;
    CHECK_THROWS_AS(signed_distance_band(cube, oversized), std::invalid_argument);
    CHECK_THROWS_AS(voxelize(cube, wrapped.spec), std::invalid_argument);
}

TEST_CASE("grid geometry is validated before any arithmetic", "[geom][voxel]") {
    // Reversed bounds with no padding would give a negative cell count.
    CHECK_THROWS_AS(grid_around(Bounds{{1, 1, 1}, {0, 0, 0}}, 0.25, 0), std::invalid_argument);
    CHECK_THROWS_AS(grid_around(Bounds{{1e300, 0, 0}, {-1e300, 1, 1}}, 1.0, 0),
                    std::invalid_argument);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS(grid_around(Bounds{{nan, 0, 0}, {1, 1, 1}}, 0.25, 0), std::invalid_argument);

    const TriangleMesh cube = cube_mesh(1.0);
    GridSpec good;
    good.origin = {-1.5, -1.5, -1.5};
    good.spacing = 1.5;
    good.nx = 2;
    good.ny = 2;
    good.nz = 2;
    const VoxelGrid ok = voxelize(cube, good);
    CHECK(ok.solid_count() == 0); // cell centres at +-0.75, outside the unit cube
    for (double spacing : {0.0, -1.0, inf, nan}) {
        INFO("spacing " << spacing);
        GridSpec bad = good;
        bad.spacing = spacing;
        CHECK_THROWS_AS(voxelize(cube, bad), std::invalid_argument);
        VoxelGrid grid = ok;
        grid.spec = bad;
        CHECK_THROWS_AS(signed_distance_band(cube, grid), std::invalid_argument);
    }
    GridSpec nan_origin = good;
    nan_origin.origin.y = nan;
    CHECK_THROWS_AS(voxelize(cube, nan_origin), std::invalid_argument);
    VoxelGrid shifted = ok;
    shifted.spec = nan_origin;
    CHECK_THROWS_AS(signed_distance_band(cube, shifted), std::invalid_argument);
    CHECK_THROWS_AS(voxelize(cube, good, nan), std::invalid_argument);
    CHECK_THROWS_AS(voxelize(cube, good, inf), std::invalid_argument);
    // A band that overflows once scaled by the spacing.
    VoxelGrid wide = ok;
    wide.spec.spacing = 1e300;
    CHECK_THROWS_AS(signed_distance_band(cube, wide, 1e10), std::invalid_argument);
}

TEST_CASE("closest point on a degenerate triangle", "[geom][voxel]") {
    const Vector3 o{};
    const Vector3 x{1, 0, 0};
    auto distance_to = [](const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c) {
        return norm(closest_point_on_triangle(p, a, b, c) - p);
    };
    // Two corners at the same position: the triangle is the segment o-x.
    const Vector3 p{0.5, 1.0, 0.0};
    for (const auto& [a, b, c] : {std::array{o, o, x}, std::array{o, x, o}, std::array{x, o, o}}) {
        const Vector3 q = closest_point_on_triangle(p, a, b, c);
        CHECK(isfinite(q));
        CHECK(std::abs(distance_to(p, a, b, c) - 1.0) < 1e-15);
    }
    // All three corners equal.
    const Vector3 v{1, 2, 3};
    CHECK(std::abs(distance_to({1, 2, 5}, v, v, v) - 2.0) < 1e-15);
    // Collinear corners: the closest point is the far end of the segment.
    const Vector3 q = closest_point_on_triangle({3, 1, 0}, o, {2, 0, 0}, x);
    CHECK(std::abs(q.x - 2.0) < 1e-15);
    CHECK(std::abs(q.y) < 1e-15);
    CHECK(std::abs(distance_to({0.5, -2, 0}, {2, 0, 0}, o, x) - 2.0) < 1e-15);

    // The same through the tree, which must not fall back to its cap.
    TriangleMesh m;
    m.vertices = {o, o, x};
    m.triangles = {{0, 1, 2}};
    const MeshTree tree(m);
    CHECK(std::abs(tree.distance(p, 10.0) - 1.0) < 1e-15);
}

TEST_CASE("solid angle and winding number are scale invariant", "[geom][voxel]") {
    const Vector3 o{};
    for (double s : {1.0, 1e-110, 1e-150, 1e100}) {
        INFO("scale " << s);
        CHECK(std::abs(solid_angle(o, {s, 0, 0}, {0, s, 0}, {0, 0, s}) - pi / 2.0) < 1e-14);
        CHECK(std::abs(solid_angle(o, {s, 0, 0}, {0, 0, s}, {0, s, 0}) + pi / 2.0) < 1e-14);
    }
    // A point on a corner sees no solid angle (and no NaN).
    CHECK(solid_angle({1, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}) == 0.0);

    for (double s : {1e-110, 1e100}) {
        INFO("scale " << s);
        TriangleMesh sphere = sphere_profile(1.0).mesh({48, pi / 24.0});
        for (Vector3& v : sphere.vertices) {
            v = v * s;
        }
        const MeshTree tree(sphere);
        for (const auto& [point, expected] :
             {std::pair{Vector3{0.1, -0.2, 0.3}, 1.0}, std::pair{Vector3{1.5, 0.0, 0.2}, 0.0},
              std::pair{Vector3{0.0, 0.0, 30.0}, 0.0}, std::pair{Vector3{4.0, -3.0, 2.0}, 0.0}}) {
            INFO("point " << point.x << ' ' << point.y << ' ' << point.z);
            CHECK(std::abs(winding_number(sphere, point * s) - expected) < 1e-12);
            CHECK(std::abs(tree.winding_number(point * s) - expected) < 0.05);
        }
    }
}
