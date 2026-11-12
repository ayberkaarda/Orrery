#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>
#include <stdexcept>

#include "orrery/geom/naca.h"
#include "orrery/geom/primitives.h"
#include "orrery/geom/reference_quantities.h"

using namespace orrery::geom;

namespace {

constexpr double pi = std::numbers::pi;

} // namespace

TEST_CASE("reference quantities of a cube", "[geom][reference]") {
    TriangleMesh cube = cube_mesh(2.0);
    for (Vector3& v : cube.vertices) {
        v += Vector3{3.0, -1.0, 0.5};
    }
    const ReferenceQuantities q = reference_quantities(cube);
    CHECK(q.wetted_area == 24.0);
    CHECK(q.volume == 8.0);
    CHECK(q.frontal_area == 4.0);
    CHECK(q.planform_area == 4.0);
    CHECK(q.frontal_area_bound == 4.0);
    CHECK(q.reference_length == 2.0);
    CHECK(q.span == 2.0);
    CHECK(norm(q.centre_of_mass - Vector3{3.0, -1.0, 0.5}) < 1e-15);
    CHECK(reference_area(q, ReferenceAreaKind::wetted) == 24.0);
    CHECK(reference_area(q, ReferenceAreaKind::frontal) == 4.0);
    CHECK(reference_area(q, ReferenceAreaKind::planform) == 4.0);
}

TEST_CASE("sphere and cylinder silhouettes", "[geom][reference]") {
    const double r = 1.0;
    const TriangleMesh sphere = sphere_profile(r).mesh({128, pi / 64.0});
    const ReferenceQuantities q = reference_quantities(sphere);
    // Convex: the bound is the exact silhouette of the polyhedron, close to
    // pi r^2; the raster is within perimeter x cell size of it.
    const double cell = 2.0 * r / 1024.0;
    CHECK(std::abs(q.frontal_area_bound - pi * r * r) < 1e-3 * pi);
    CHECK(std::abs(q.frontal_area - q.frontal_area_bound) < 2.0 * pi * r * cell);
    CHECK(std::abs(q.planform_area - q.planform_area_bound) < 2.0 * pi * r * cell);
    CHECK(norm(q.centre_of_mass) < 1e-14);

    // Cylinder along z seen along x: a 2r x h rectangle (vertices at
    // multiples of 90 degrees lie on the silhouette).
    const double h = 3.0;
    const TriangleMesh cylinder = cylinder_profile(r, h).mesh({64, 0.1});
    const double bound = projected_area_bound(cylinder, {1.0, 0.0, 0.0});
    CHECK(std::abs(bound - 2.0 * r * h) < 1e-12);
    const double raster = projected_area(cylinder, {1.0, 0.0, 0.0}, 2048);
    CHECK(std::abs(raster - 2.0 * r * h) < 2.0 * (2.0 * r + h) * (h / 2048.0));
    const ReferenceQuantities c = reference_quantities(cylinder);
    CHECK(std::abs(c.centre_of_mass.z - 0.5 * h) < 1e-14);
}

TEST_CASE("wing planform, chord and span", "[geom][reference]") {
    const double chord = 1.2;
    const double span = 5.0;
    const NacaWing wing = naca_wing(*Naca4::from_designation("0012"), chord, span, 60, 10);
    const ReferenceQuantities q = reference_quantities(wing.mesh);
    // A symmetric section spans exactly [0, chord].
    CHECK(std::abs(q.reference_length - chord) < 1e-15);
    CHECK(std::abs(q.span - span) < 1e-15);
    // The planform bound is exact for a wing whose surfaces are single valued
    // over the planform.
    CHECK(std::abs(q.planform_area_bound - chord * span) < 1e-12);
    CHECK(std::abs(q.planform_area - chord * span) < 2.0 * (chord + span) * (span / 1024.0));
    CHECK(q.frontal_area < 0.13 * chord * span);
    CHECK(std::abs(q.volume - wing.polyhedral_volume) < 1e-12);
    CHECK(reference_area(q, ReferenceAreaKind::planform) == q.planform_area);
}

TEST_CASE("reference options are validated", "[geom][reference]") {
    const TriangleMesh cube = cube_mesh(1.0);
    ReferenceOptions o;
    o.lift_direction = o.flow_direction;
    CHECK_THROWS_AS(reference_quantities(cube, o), std::invalid_argument);
    o = {};
    o.flow_direction = {};
    CHECK_THROWS_AS(reference_quantities(cube, o), std::invalid_argument);
    o = {};
    o.raster_resolution = 1;
    CHECK_THROWS_AS(reference_quantities(cube, o), std::invalid_argument);
}
