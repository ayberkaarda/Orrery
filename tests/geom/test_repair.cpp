#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <numbers>
#include <vector>

#include "geom_test_support.h"
#include "orrery/geom/mesh_import.h"
#include "orrery/geom/mesh_repair.h"
#include "orrery/geom/primitives.h"

using namespace orrery::geom;
namespace t = orrery::geom::test;

// tests/geom/fixtures/broken_box.stl is the unit cube [0, 1]^3 (12 outward
// triangles) with deliberate defects:
//   - one triangle of the x = 0 face removed: a hole of three edges;
//   - one triangle of the z = 0 face written with reversed winding: its
//     three edges are used twice in the same direction;
//   - one top triangle written twice: a duplicate;
//   - a zero-area triangle along the edge y = z = 0: degenerate;
//   - a fin (1,1,0) (1,1,1) (2,2,0.5) on the cube edge x = y = 1: that edge
//     has three triangles, and the fin adds two free edges.
TEST_CASE("broken STL fixture is reported correctly", "[geom][repair]") {
    const auto imported = import_mesh_file(t::fixture_dir() / "broken_box.stl");
    REQUIRE(imported.has_value());
    const TriangleMesh& mesh = imported.value().mesh;

    const WatertightReport r = analyze_mesh(mesh);
    CHECK(r.triangles == 14);
    CHECK(r.degenerate_triangles == 1);
    CHECK(r.duplicate_triangles == 1);
    CHECK(r.edges == 20);
    CHECK(r.boundary_edges == 5);
    CHECK(r.non_manifold_edges == 1);
    CHECK(r.inconsistent_edges == 3);
    CHECK(r.boundary_loops == 1);
    CHECK(r.open_boundary_chains == 1);
    CHECK(r.components == 1);
    CHECK_FALSE(r.watertight());

    const RepairResult fixed = repair_mesh(mesh);
    CHECK(fixed.removed_degenerate == 1);
    CHECK(fixed.removed_duplicates == 1);
    CHECK(fixed.removed_fins == 1);
    CHECK(fixed.filled_holes == 1);
    CHECK(fixed.added_triangles == 1);
    CHECK_FALSE(fixed.orientation_conflict);
    CHECK(fixed.after.watertight());
    CHECK(fixed.after.outward());
    CHECK(fixed.after.triangles == 12);
    CHECK(fixed.after.duplicate_vertices == 0);
    CHECK(fixed.after.unreferenced_vertices == 0);
    CHECK(fixed.mesh.vertices.size() == 8);
    CHECK(std::abs(signed_volume(fixed.mesh) - 1.0) < 1e-15);
    CHECK(std::abs(surface_area(fixed.mesh) - 6.0) < 1e-14);
}

TEST_CASE("clean primitives report watertight", "[geom][repair]") {
    for (const TriangleMesh& m :
         {cube_mesh(1.0), sphere_profile(1.0).mesh({24, 0.3}),
          capsule_profile(CapsuleShape::orion()).mesh({32, 0.1}), Wedge{}.mesh()}) {
        const WatertightReport r = analyze_mesh(m);
        CHECK(r.watertight());
        CHECK(r.boundary_edges == 0);
        CHECK(r.non_manifold_edges == 0);
        CHECK(r.inconsistent_edges == 0);
        CHECK(r.boundary_loops == 0);
        // Euler characteristic of a sphere: V - E + F = 2.
        CHECK(static_cast<long long>(r.vertices) - static_cast<long long>(r.edges) +
                  static_cast<long long>(r.triangles) ==
              2);
        const RepairResult again = repair_mesh(m);
        CHECK(again.flipped_triangles == 0);
        CHECK(again.added_triangles == 0);
        CHECK(again.mesh.triangles == m.triangles);
    }
}

TEST_CASE("an inside-out mesh is turned outward and a large hole is fanned", "[geom][repair]") {
    TriangleMesh m = sphere_profile(2.0).mesh({32, 0.1});
    for (Triangle& tri : m.triangles) {
        std::swap(tri[1], tri[2]);
    }
    CHECK_FALSE(analyze_mesh(m).outward());
    // Remove the top cap (the fan around the top pole): a 32-edge hole.
    m.triangles.resize(m.triangles.size() - 32);
    const WatertightReport before = analyze_mesh(m);
    CHECK(before.boundary_loops == 1);
    CHECK(before.boundary_edges == 32);
    CHECK(before.unreferenced_vertices == 1);
    const RepairResult fixed = repair_mesh(m);
    CHECK(fixed.filled_holes == 1);
    CHECK(fixed.added_triangles == 32);
    CHECK(fixed.after.watertight());
    CHECK(fixed.after.outward());
    // The fan from the ring centroid replaces the cap by a flat disk.
    CHECK(signed_volume(fixed.mesh) > 0.0);
}

TEST_CASE("duplicate vertices are welded, within a tolerance if asked", "[geom][repair]") {
    // Two triangles of a square with their shared corners stored twice.
    TriangleMesh m;
    m.vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 0, 0}, {1, 1, 1e-9}, {0, 1, 0}};
    m.triangles = {{0, 1, 2}, {3, 4, 5}};
    const WatertightReport exact = analyze_mesh(m);
    CHECK(exact.duplicate_vertices == 1);
    CHECK(exact.boundary_edges == 6);
    const WatertightReport welded = analyze_mesh(m, 1e-6);
    CHECK(welded.duplicate_vertices == 2);
    CHECK(welded.boundary_edges == 4);
    CHECK(welded.boundary_loops == 1);
    CHECK_THROWS(analyze_mesh(m, -1.0));
}

TEST_CASE("a Moebius strip cannot be oriented", "[geom][repair]") {
    TriangleMesh m;
    constexpr std::uint32_t n = 24;
    for (std::uint32_t i = 0; i < n; ++i) {
        const double u = 2.0 * std::numbers::pi * i / n;
        for (double s : {-0.3, 0.3}) {
            const double r = 1.0 + s * std::cos(u / 2.0);
            m.vertices.push_back({r * std::cos(u), r * std::sin(u), s * std::sin(u / 2.0)});
        }
    }
    for (std::uint32_t i = 0; i < n; ++i) {
        const std::uint32_t a = 2 * i;
        const std::uint32_t b = 2 * i + 1;
        // The last strip joins the first with its two rails swapped.
        const bool last = i + 1 == n;
        const std::uint32_t c = last ? 1 : 2 * (i + 1);
        const std::uint32_t d = last ? 0 : 2 * (i + 1) + 1;
        m.triangles.push_back({a, c, b});
        m.triangles.push_back({b, c, d});
    }
    const RepairResult r = repair_mesh(m);
    CHECK(r.orientation_conflict);
    // One boundary curve, twice around; the seam breaks its direction.
    CHECK(r.before.boundary_edges == std::size_t{2} * n);
    CHECK(r.before.non_manifold_edges == 0);
}
