#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "geom_test_support.h"
#include "orrery/geom/mesh_repair.h"
#include "orrery/geom/naca.h"

using namespace orrery::geom;
using orrery::geom::test::relative_error;

TEST_CASE("NACA designations parse", "[geom][naca]") {
    const auto a = Naca4::from_designation("2412");
    REQUIRE(a.has_value());
    CHECK(a->max_camber == 0.02);
    CHECK(a->camber_position == 0.4);
    CHECK(a->thickness == 0.12);
    CHECK(Naca4::from_designation("0012").has_value());
    CHECK_FALSE(Naca4::from_designation("12a4").has_value());
    CHECK_FALSE(Naca4::from_designation("2012").has_value()); // camber without position
    CHECK_FALSE(Naca4::from_designation("0000").has_value());
    CHECK_FALSE(Naca4::from_designation("00120").has_value());
}

TEST_CASE("NACA 0012 section area matches the closed form", "[geom][naca]") {
    const Naca4 a = *Naca4::from_designation("0012");
    const double exact =
        10.0 * 0.12 *
        (0.2969 * 2.0 / 3.0 - 0.1260 / 2.0 - 0.3516 / 3.0 + 0.2843 / 4.0 - 0.1036 / 5.0);
    CHECK(relative_error(a.section_area(), exact) < 1e-13);
    // Closed trailing edge; maximum thickness 12 percent near 30 percent chord.
    CHECK(std::abs(a.half_thickness(1.0)) < 1e-15);
    double thickest = 0.0;
    for (int i = 0; i <= 1000; ++i) {
        thickest = std::max(thickest, 2.0 * a.half_thickness(i / 1000.0));
    }
    CHECK(std::abs(thickest - 0.12) < 2e-4);
    // The polygon converges to the exact area at second order.
    double previous = 1.0;
    for (int n : {100, 200, 400, 800}) {
        const double error = relative_error(polygon_area(a.section(n)), exact);
        CHECK(error < 0.3 * previous);
        previous = error;
    }
    CHECK(previous < 1e-5);
}

TEST_CASE("cambered NACA 2412 area matches a fine polygon", "[geom][naca]") {
    const Naca4 a = *Naca4::from_designation("2412");
    const double exact = a.section_area();
    // Richardson extrapolation of the polygon areas (second order).
    const double coarse = polygon_area(a.section(4000));
    const double fine = polygon_area(a.section(8000));
    const double extrapolated = fine + (fine - coarse) / 3.0;
    CHECK(relative_error(extrapolated, exact) < 1e-9);
    // Camber line: zero at both ends, maximum m at p.
    CHECK(a.camber(0.0) == 0.0);
    CHECK(std::abs(a.camber(1.0)) < 1e-17);
    CHECK(std::abs(a.camber(0.4) - 0.02) < 1e-15);
    // Counter-clockwise polygon.
    CHECK(polygon_area(a.section(64)) > 0.0);
}

TEST_CASE("extruded NACA wing is a closed prism", "[geom][naca]") {
    for (const char* digits : {"0012", "2412", "4415"}) {
        const Naca4 a = *Naca4::from_designation(digits);
        const double chord = 1.5;
        const double span = 6.0;
        const NacaWing wing = naca_wing(a, chord, span, 80, 12);
        const WatertightReport r = analyze_mesh(wing.mesh);
        CHECK(r.watertight());
        CHECK(r.outward());
        CHECK(r.degenerate_triangles == 0);
        CHECK(relative_error(signed_volume(wing.mesh), wing.polyhedral_volume) < 1e-9);
        CHECK(relative_error(surface_area(wing.mesh), wing.polyhedral_area) < 1e-9);
        CHECK(relative_error(wing.polyhedral_volume, a.section_area() * chord * chord * span) <
              5e-4);
        REQUIRE(wing.trailing_edge.size() == 13);
        for (std::size_t j = 0; j < wing.trailing_edge.size(); ++j) {
            const Vector3& v = wing.mesh.vertices[wing.trailing_edge[j]];
            CHECK(v.x == chord);
            CHECK(v.z == 0.0);
            CHECK(std::abs(v.y - span * (static_cast<double>(j) / 12.0 - 0.5)) < 1e-12);
        }
    }
    CHECK_THROWS_AS(naca_wing(Naca4{}, -1.0, 1.0, 10, 1), std::invalid_argument);
    CHECK_THROWS_AS(naca_wing(Naca4{}, 1.0, 1.0, 2, 1), std::invalid_argument);
    CHECK_THROWS_AS(naca_wing(Naca4{}, 1.0, 1.0, 10, 0), std::invalid_argument);
}
