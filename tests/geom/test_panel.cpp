#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

#include "orrery/geom/naca.h"
#include "orrery/geom/panel_mesh.h"
#include "orrery/geom/primitives.h"

using namespace orrery::geom;

TEST_CASE("sphere panels close and face outward", "[geom][panel]") {
    const TriangleMesh m = sphere_profile(1.5).mesh({32, 0.1});
    const PanelMesh p = build_panels(m);
    REQUIRE(p.panels.size() == m.triangles.size());
    Vector3 closure;
    double area = 0.0;
    for (const Panel& panel : p.panels) {
        CHECK(std::abs(norm(panel.normal) - 1.0) < 1e-14);
        CHECK(dot(panel.normal, panel.collocation) > 0.0);
        closure += panel.normal * panel.area;
        area += panel.area;
    }
    // The area vectors of a closed surface sum to zero.
    CHECK(norm(closure) < 1e-12 * area);
    CHECK(std::abs(area - surface_area(m)) < 1e-12 * area);
    CHECK(p.wake.empty());
}

TEST_CASE("a degenerate triangle cannot become a panel", "[geom][panel]") {
    TriangleMesh m;
    m.vertices = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}};
    m.triangles = {{0, 1, 2}};
    CHECK_THROWS_AS(build_panels(m), std::invalid_argument);
}

TEST_CASE("wing trailing edge is detected and sheds wake strips", "[geom][panel]") {
    const Naca4 airfoil = *Naca4::from_designation("2412");
    const double chord = 1.0;
    const double span = 4.0;
    const int strips = 8;
    const NacaWing wing = naca_wing(airfoil, chord, span, 40, strips);

    const std::vector<std::uint32_t> te = detect_trailing_edge(wing.mesh);
    CHECK(te == wing.trailing_edge);

    PanelMesh p = build_panels(wing.mesh);
    WakeOptions options;
    options.length = 20.0;
    options.rows = 5;
    add_wake(p, wing.mesh, te, options);
    REQUIRE(p.wake.size() == static_cast<std::size_t>(strips * options.rows));
    for (const WakePanel& w : p.wake) {
        CHECK(std::abs(w.area - (span / strips) * (options.length / options.rows)) < 1e-12);
        CHECK(std::abs(std::abs(w.normal.z) - 1.0) < 1e-14);
        // The upper panel faces up and the lower panel down.
        CHECK(p.panels[w.upper_panel].normal.z > 0.0);
        CHECK(p.panels[w.lower_panel].normal.z < 0.0);
        if (w.row == 0) {
            CHECK(w.corners[0].x == chord);
            CHECK(w.corners[1].x == chord);
        }
    }
    const WakePanel& last = p.wake.back();
    CHECK(std::abs(last.corners[2].x - (chord + options.length)) < 1e-12);

    // No trailing edge on bodies without a sharp downstream edge.
    CHECK(detect_trailing_edge(sphere_profile(1.0).mesh({16, 0.3})).empty());
    CHECK(detect_trailing_edge(cube_mesh(1.0)).empty());

    // Invalid wake requests.
    options.rows = 0;
    CHECK_THROWS_AS(add_wake(p, wing.mesh, te, options), std::invalid_argument);
    options.rows = 1;
    CHECK_THROWS_AS(add_wake(p, wing.mesh, {0, 5}, options), std::invalid_argument);
}

TEST_CASE("a wake that would have no usable panels is rejected", "[geom][panel]") {
    const NacaWing wing = naca_wing(*Naca4::from_designation("0012"), 1.0, 4.0, 40, 4);
    PanelMesh p = build_panels(wing.mesh);
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    // An infinite direction normalizes to NaN; a direction along the
    // trailing edge (the span, +y) sweeps panels of zero area.
    for (const Vector3& direction : {Vector3{inf, 0.0, 0.0}, Vector3{nan, 0.0, 0.0},
                                     Vector3{0.0, 1.0, 0.0}, Vector3{0.0, -2.0, 0.0}}) {
        INFO("direction " << direction.x << ' ' << direction.y << ' ' << direction.z);
        WakeOptions options;
        options.direction = direction;
        CHECK_THROWS_AS(add_wake(p, wing.mesh, wing.trailing_edge, options), std::invalid_argument);
        CHECK(p.wake.empty());
    }
    WakeOptions options;
    options.up = {0.0, nan, 1.0};
    CHECK_THROWS_AS(add_wake(p, wing.mesh, wing.trailing_edge, options), std::invalid_argument);
    options.up = {0.0, 0.0, 1.0};
    options.length = inf;
    CHECK_THROWS_AS(add_wake(p, wing.mesh, wing.trailing_edge, options), std::invalid_argument);
    CHECK(p.wake.empty());
    // A valid request still works.
    options.length = 10.0;
    add_wake(p, wing.mesh, wing.trailing_edge, options);
    CHECK(p.wake.size() == 4);
    for (const WakePanel& w : p.wake) {
        CHECK(isfinite(w.normal));
        CHECK(w.area > 0.0);
    }
}
