#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <glm/geometric.hpp>

#include "aero_test_support.h"
#include "orrery/render/aero_field.h"
#include "orrery/render/aero_surface.h"

// Flow fields of the aero visualisation: grids, solver adapters, derived
// quantities, interpolation and section sampling; and the surface builders.

using namespace orrery;
using namespace orrery::render;
using orrery::render::test::scalar_grid;
using orrery::render::test::vector_grid;

TEST_CASE("Grid geometry and validation", "[render][aero][field]") {
    GridGeometry g;
    g.dims = {4, 3, 2};
    g.origin = {1.0, 2.0, 3.0};
    g.spacing = 0.5;
    CHECK(g.node_count() == 24);
    CHECK(g.index(1, 2, 1) == 1 + 4 * (2 + 3 * 1));
    CHECK(glm::length(g.node_position(1, 2, 1) - glm::dvec3(1.5, 3.0, 3.5)) == 0.0);
    CHECK(glm::length(g.upper_corner() - glm::dvec3(2.5, 3.0, 3.5)) == 0.0);
    CHECK_FALSE(validate(g).has_value());
    GridGeometry flat = g;
    flat.dims[2] = 1;
    CHECK(flat.is_2d());
    CHECK_FALSE(validate(flat).has_value());
    GridGeometry bad = g;
    bad.spacing = 0.0;
    CHECK(validate(bad).has_value());
    bad = g;
    bad.dims = {1, 3, 2};
    CHECK(validate(bad).has_value());
    bad = g;
    bad.dims = {1024, 1024, 1024};
    CHECK(validate(bad).has_value());

    FlowField field;
    field.velocity.grid = g;
    field.velocity.values.assign(23, glm::dvec3(0.0));
    CHECK(validate(field).has_value());
    field.velocity.values.assign(24, glm::dvec3(0.0));
    CHECK_FALSE(validate(field).has_value());
    field.solid.assign(5, std::uint8_t{0});
    CHECK(validate(field).has_value());
    field.solid.clear();
    field.pressure = ScalarGrid{flat, std::vector<double>(12, 0.0)};
    CHECK(validate(field).has_value()); // pressure on another grid
}

TEST_CASE("Trilinear sampling reproduces linear fields", "[render][aero][field]") {
    const auto linear = [](const glm::dvec3& p) { return 2.0 * p.x - 3.0 * p.y + 0.5 * p.z + 1.0; };
    const ScalarGrid f = scalar_grid(5, 6, 7, {-1.0, -2.0, 0.5}, 0.25, linear);
    for (const glm::dvec3 p : {glm::dvec3(-0.9, -1.7, 0.6), glm::dvec3(-0.13, -0.81, 1.9),
                               glm::dvec3(0.0, -0.75, 2.0)}) {
        const auto v = sample(f, p);
        REQUIRE(v.has_value());
        CHECK(*v == Catch::Approx(linear(p)).margin(1.0e-12));
    }
    CHECK_FALSE(sample(f, {-1.1, -1.7, 0.6}).has_value());
    CHECK_FALSE(sample(f, {0.0, 0.0, 0.6}).has_value()); // y beyond -2 + 5 * 0.25
    // A 2D grid ignores z.
    const ScalarGrid flat = scalar_grid(5, 6, 1, {0.0, 0.0, 0.0}, 1.0, linear);
    const auto v = sample(flat, {1.5, 2.25, 40.0});
    REQUIRE(v.has_value());
    CHECK(*v == Catch::Approx(linear({1.5, 2.25, 0.0})));
}

TEST_CASE("Vorticity, speed and Q of analytic flows", "[render][aero][field]") {
    // Solid-body rotation u = (-w y, w x, 0): vorticity 2 w along z, Q = w^2.
    const double w = 1.5;
    const VectorGrid rotation =
        vector_grid(9, 9, 5, {-2.0, -2.0, -1.0}, 0.5,
                    [w](const glm::dvec3& p) { return glm::dvec3(-w * p.y, w * p.x, 0.0); });
    const VectorGrid omega = vorticity(rotation, {});
    const ScalarGrid q = q_criterion(rotation, {});
    for (std::size_t n = 0; n < omega.values.size(); ++n) {
        CHECK(omega.values[n].z == Catch::Approx(2.0 * w).margin(1.0e-12));
        CHECK(q.values[n] == Catch::Approx(w * w).margin(1.0e-12));
    }
    // Pure strain u = (a x, -a y, 0): no vorticity, Q = -a^2.
    const double a = 0.8;
    const VectorGrid strain = vector_grid(6, 6, 6, {0.0, 0.0, 0.0}, 0.2, [a](const glm::dvec3& p) {
        return glm::dvec3(a * p.x, -a * p.y, 0.0);
    });
    const ScalarGrid qs = q_criterion(strain, {});
    const VectorGrid os = vorticity(strain, {});
    for (std::size_t n = 0; n < qs.values.size(); ++n) {
        CHECK(qs.values[n] == Catch::Approx(-a * a).margin(1.0e-12));
        CHECK(glm::length(os.values[n]) < 1.0e-12);
    }
    const ScalarGrid s = speed(rotation);
    CHECK(s.values[rotation.grid.index(8, 4, 2)] == Catch::Approx(w * 2.0));

    FlowField field;
    field.velocity = rotation;
    const ScalarGrid wz = derive(field, FieldQuantity::vorticity, Axis::z);
    const ScalarGrid wx = derive(field, FieldQuantity::vorticity, Axis::x);
    const ScalarGrid wm = derive(field, FieldQuantity::vorticity_magnitude);
    CHECK(wz.values[10] == Catch::Approx(2.0 * w));
    CHECK(wx.values[10] == Catch::Approx(0.0).margin(1.0e-12));
    CHECK(wm.values[10] == Catch::Approx(2.0 * w));
    CHECK(derive(field, FieldQuantity::pressure).values.empty());
    CHECK(is_signed(FieldQuantity::vorticity));
    CHECK_FALSE(is_signed(FieldQuantity::speed));
    CHECK(quantity_label(FieldQuantity::pressure, "Cp") == "Cp");
}

TEST_CASE("Derivatives are one-sided next to solid nodes", "[render][aero][field]") {
    // u = (y^2, 0, 0) on a 2D grid with a solid row at j = 2: the node at
    // j = 3 must not difference across it.
    const VectorGrid u = vector_grid(4, 6, 1, {0.0, 0.0, 0.0}, 1.0, [](const glm::dvec3& p) {
        return glm::dvec3(p.y * p.y, 0.0, 0.0);
    });
    std::vector<std::uint8_t> solid(u.values.size(), 0);
    for (std::size_t i = 0; i < 4; ++i) {
        solid[u.grid.index(i, 2, 0)] = 1;
    }
    const VectorGrid w = vorticity(u, solid);
    // omega_z = -du/dy; one-sided forward at j = 3: (16 - 9) = 7.
    CHECK(w.values[u.grid.index(1, 3, 0)].z == Catch::Approx(-7.0));
    CHECK(w.values[u.grid.index(1, 2, 0)].z == 0.0);                 // solid
    CHECK(w.values[u.grid.index(1, 4, 0)].z == Catch::Approx(-8.0)); // central (25 - 9) / 2
}

TEST_CASE("Section sampling interpolates along the normal", "[render][aero][field]") {
    const auto f = [](const glm::dvec3& p) { return p.x + 10.0 * p.y + 100.0 * p.z; };
    const ScalarGrid grid = scalar_grid(4, 5, 6, {0.0, 0.0, 0.0}, 1.0, f);
    std::vector<std::uint8_t> solid(grid.values.size(), 0);
    solid[grid.grid.index(2, 3, 2)] = 1;

    const SectionImage z = sample_section(grid, solid, Axis::z, 2.25);
    CHECK(z.width == 4);
    CHECK(z.height == 5);
    CHECK(z.position == 2.25);
    CHECK(z.values[1 + 4 * 2] == Catch::Approx(f({1.0, 2.0, 2.25})));
    CHECK(z.solid[2 + 4 * 3] == 1); // nearer node k = 2
    CHECK(glm::length(z.corner - glm::dvec3(0.0, 0.0, 2.25)) == 0.0);
    CHECK(glm::length(z.u_step - glm::dvec3(1.0, 0.0, 0.0)) == 0.0);
    CHECK(glm::length(z.v_step - glm::dvec3(0.0, 1.0, 0.0)) == 0.0);

    const SectionImage x = sample_section(grid, solid, Axis::x, 1.5);
    CHECK(x.width == 5);  // y
    CHECK(x.height == 6); // z
    CHECK(x.values[2 + 5 * 3] == Catch::Approx(f({1.5, 2.0, 3.0})));
    const SectionImage y = sample_section(grid, solid, Axis::y, 99.0); // clamped
    CHECK(y.position == 4.0);
    CHECK(y.width == 4);
    CHECK(y.height == 6);
    CHECK(y.values[3 + 4 * 5] == Catch::Approx(f({3.0, 4.0, 5.0})));

    // Every point of a section matches trilinear sampling at its position.
    for (std::size_t v = 0; v < z.height; ++v) {
        for (std::size_t u = 0; u < z.width; ++u) {
            const glm::dvec3 p =
                z.corner + z.u_step * static_cast<double>(u) + z.v_step * static_cast<double>(v);
            CHECK(z.values[u + z.width * v] == Catch::Approx(*sample(grid, p)).margin(1.0e-12));
        }
    }
    // A 2D grid has only the z section.
    const ScalarGrid flat = scalar_grid(4, 5, 1, {0.0, 0.0, 0.0}, 1.0, f);
    CHECK(sample_section(flat, {}, Axis::z, 7.0).values.size() == 20);
    CHECK(sample_section(flat, {}, Axis::x, 1.0).values.empty());
}

TEST_CASE("LBM and velocity-field adapters", "[render][aero][field]") {
    aero::lbm::MacroFields2D macro;
    macro.nx = 3;
    macro.ny = 2;
    macro.density = {1.0, 1.003, 0.997, 1.0, 1.0, 1.0};
    macro.velocity_x = {0.05, 0.04, 0.0, 0.05, 0.05, 0.05};
    macro.velocity_y = {0.0, 0.01, 0.0, 0.0, 0.0, 0.0};
    macro.solid = {0, 0, 1, 0, 0, 0};
    LatticeMapping mapping;
    mapping.origin = {-1.0, 0.0, 0.0};
    mapping.spacing = 0.01;
    mapping.velocity_scale = 200.0;
    mapping.reference_speed = 0.05;
    const FlowField field = flow_field_from(macro, mapping);
    CHECK_FALSE(validate(field).has_value());
    CHECK(field.velocity.grid.is_2d());
    CHECK(field.velocity.values[1].x == Catch::Approx(8.0));
    CHECK(field.velocity.values[1].y == Catch::Approx(2.0));
    REQUIRE(field.pressure.has_value());
    CHECK(field.pressure_label == "Cp");
    // Cp = c_s^2 (rho - 1) / (U^2 / 2) = (0.003 / 3) / 0.00125 = 0.8.
    CHECK(field.pressure->values[1] == Catch::Approx(0.8));
    CHECK(field.solid[2] == 1);

    aero::lbm::MacroFields3D macro3;
    macro3.nx = 2;
    macro3.ny = 2;
    macro3.nz = 2;
    macro3.density.assign(8, 1.0);
    macro3.velocity_x.assign(8, 0.1);
    macro3.velocity_y.assign(8, 0.0);
    macro3.velocity_z.assign(8, -0.1);
    macro3.solid.assign(8, std::uint8_t{0});
    const FlowField field3 = flow_field_from(macro3);
    CHECK_FALSE(validate(field3).has_value());
    CHECK(field3.pressure_label == "p (lattice)");
    CHECK(field3.velocity.values[7].z == Catch::Approx(-0.1));

    aero::VelocityField v;
    v.dims = {2, 2, 1};
    v.spacing = 0.5;
    v.origin = {1.0, 1.0, 0.0};
    v.values.assign(4, geom::Vector3{3.0, 0.0, 0.0});
    const FlowField plain = flow_field_from(v);
    CHECK_FALSE(validate(plain).has_value());
    CHECK_FALSE(plain.pressure.has_value());
    CHECK(plain.velocity.values[3].x == 3.0);
}

TEST_CASE("Extruded sections carry one Cp per panel and close at the ends",
          "[render][aero][surface]") {
    const std::vector<geom::SectionPoint> square{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
    const std::vector<double> cp{-1.0, 0.0, 0.5, 1.0};
    auto made = extrude_section(square, cp, 2.0, 3);
    REQUIRE(made.has_value());
    const AeroSurface& s = made.value();
    CHECK(s.mesh.triangles.size() == 2 * 4 * 3 + 2 * 4);
    CHECK(s.cp.size() == s.mesh.triangles.size());
    CHECK(s.cp[0] == -1.0);
    CHECK(s.cp[2] == 0.0);
    CHECK(std::isnan(s.cp.back()));
    // Closed and outward: positive volume equal to the prism.
    CHECK(geom::signed_volume(s.mesh) == Catch::Approx(2.0));
    CHECK(geom::surface_area(s.mesh) == Catch::Approx(4.0 * 2.0 + 2.0));
    // Smoothing averages the two panels around each section vertex.
    const std::vector<double> smooth = vertex_values(s.mesh, s.cp);
    CHECK(smooth[1] == Catch::Approx(-0.5)); // between panel 0 (-1) and 1 (0)
    CHECK(smooth[0] == Catch::Approx(0.0));  // between panel 3 (1) and 0 (-1)

    CHECK_FALSE(extrude_section(square, std::vector<double>{1.0}, 1.0, 1).has_value());
    CHECK_FALSE(extrude_section(square, cp, -1.0, 1).has_value());
    CHECK_FALSE(extrude_section(square, cp, 1.0, 0).has_value());
}

TEST_CASE("Surfaces and forces from solver results", "[render][aero][surface]") {
    geom::TriangleMesh mesh;
    mesh.vertices = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
    mesh.triangles = {{0, 1, 2}};
    aero::AeroResult result;
    result.surface.cp = {0.25};
    result.cd = 0.02;
    result.cl = 0.5;
    result.force = {2.0, 0.0, 50.0};
    auto surface = surface_from_result(mesh, result);
    REQUIRE(surface.has_value());
    CHECK(surface.value().cp == std::vector<double>{0.25});
    result.surface.cp.push_back(1.0);
    CHECK_FALSE(surface_from_result(mesh, result).has_value());

    aero::FlowConditions conditions;
    conditions.speed = 30.0;
    conditions.angle_of_attack = 0.0;
    const AeroForces forces = forces_from_result(result, conditions, {0.25, 0.0, 0.0});
    CHECK(forces.drag.coefficient == 0.02);
    CHECK(forces.lift.coefficient == 0.5);
    CHECK(forces.drag.force == Catch::Approx(2.0));
    CHECK(forces.lift.force == Catch::Approx(50.0));
    CHECK(forces.lift.direction.z == Catch::Approx(1.0));
    CHECK(forces.application_point.x == 0.25);
}
