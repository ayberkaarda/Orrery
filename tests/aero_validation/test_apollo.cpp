#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <iostream>
#include <numbers>

#include "orrery/aero/newtonian.h"
#include "orrery/geom/primitives.h"

using namespace orrery::aero;
using orrery::geom::Vector3;

namespace {

constexpr double pi = std::numbers::pi;
constexpr double degree = pi / 180.0;

// The capsule primitive has its axis along +z with the heat shield facing -z;
// body axes have the free stream along +x. Rotate (x, y, z) -> (z, y, -x), a
// proper rotation, so the heat shield faces the flow.
orrery::geom::TriangleMesh heat_shield_forward(orrery::geom::TriangleMesh mesh) {
    for (Vector3& v : mesh.vertices) {
        v = Vector3{v.z, v.y, -v.x};
    }
    return mesh;
}

FlowConditions mach20(double alpha_deg) {
    FlowConditions c;
    // About 60 km altitude (US Standard Atmosphere 1976).
    c.temperature = 247.0;
    c.density = 3.1e-4;
    c.speed = 20.0 * c.speed_of_sound();
    c.angle_of_attack = alpha_deg * degree;
    return c;
}

} // namespace

// Apollo command module at Mach 20, modified Newtonian theory.
// Reference: hypersonic Apollo command module drag coefficient about 1.3 at
// its trim attitude, about 20 degrees between the free stream and the axis
// (160 degrees in the Apollo convention), with L/D about 0.3; from wind-tunnel
// and Apollo 4 flight data (Hillje, NASA TN D-5399, 1969; Moseley, Moore and
// Hughes, NASA TN D-3890, 1967). The criterion is Cd = 1.3 within 10 percent.
TEST_CASE("Apollo capsule at Mach 20: Cd about 1.3", "[aero_validation][newtonian]") {
    const orrery::geom::CapsuleShape shape = orrery::geom::CapsuleShape::apollo();
    AeroGeometry g;
    g.mesh = heat_shield_forward(
        orrery::geom::capsule_profile(shape).mesh(orrery::geom::RevolutionResolution{128, 0.01}));
    const double radius = 0.5 * shape.max_diameter;
    g.reference = {pi * radius * radius, shape.max_diameter, {}};

    NewtonianOptions options;
    options.nose_radius = shape.nose_radius;
    NewtonianSolver solver(options);
    for (const double alpha : {0.0, 10.0, 20.0, 25.0}) {
        solver.setup(g, mach20(alpha));
        REQUIRE(solver.step());
        const AeroResult& r = solver.result();
        std::cout << "Apollo Mach 20, alpha " << alpha << " deg: Cd " << r.cd << ", Cl " << r.cl
                  << ", L/D " << r.cl / r.cd << ", q_s "
                  << r.stagnation_heat_flux.value_or(0.0) / 1e4 << " W/cm^2\n";
        CHECK(r.warnings.empty());
    }
    solver.setup(g, mach20(20.0));
    solver.step();
    const AeroResult& trim = solver.result();
    CHECK(std::abs(trim.cd - 1.3) < 0.10 * 1.3);
    // Lift to drag ratio about 0.3 in magnitude. A blunt body's force is mostly
    // axial, so its lift opposes the angle of attack (negative cl here).
    CHECK(std::abs(std::abs(trim.cl / trim.cd) - 0.3) < 0.05);
}
