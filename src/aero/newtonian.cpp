#include "orrery/aero/newtonian.h"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>

namespace orrery::aero {

using geom::Vector3;

double newtonian_cp_max(double mach, double gamma) {
    if (!std::isfinite(mach) || !(mach > 0.0) || !std::isfinite(gamma) || !(gamma > 1.0)) {
        throw std::invalid_argument("newtonian_cp_max: Mach must be positive and gamma above 1");
    }
    const double m2 = mach * mach;
    const double exponent = gamma / (gamma - 1.0);
    double ratio = 0.0; // stagnation over free-stream static pressure
    if (mach >= 1.0) {
        const double g1 = gamma + 1.0;
        ratio = std::pow(g1 * g1 * m2 / (4.0 * gamma * m2 - 2.0 * (gamma - 1.0)), exponent) *
                (1.0 - gamma + 2.0 * gamma * m2) / g1;
    } else {
        ratio = std::pow(1.0 + 0.5 * (gamma - 1.0) * m2, exponent);
    }
    return 2.0 / (gamma * m2) * (ratio - 1.0);
}

double sutton_graves_heat_flux(double density, double speed, double nose_radius, double k) {
    if (!std::isfinite(density) || density < 0.0 || !std::isfinite(speed) ||
        !std::isfinite(nose_radius) || !(nose_radius > 0.0) || !std::isfinite(k) || k < 0.0) {
        throw std::invalid_argument(
            "sutton_graves_heat_flux: density, k >= 0, nose radius > 0, all finite");
    }
    // APPROXIMATION (A8): convective, cold-wall stagnation-point correlation.
    return k * std::sqrt(density / nose_radius) * std::abs(speed * speed * speed);
}

void NewtonianSolver::setup(const AeroGeometry& geometry, const FlowConditions& conditions) {
    if (geometry.mesh.triangles.empty()) {
        throw std::invalid_argument("NewtonianSolver: the mesh has no triangles");
    }
    for (const Vector3& v : geometry.mesh.vertices) {
        if (!geom::isfinite(v)) {
            throw std::invalid_argument("NewtonianSolver: mesh vertex is not finite");
        }
    }
    for (const geom::Triangle& t : geometry.mesh.triangles) {
        for (const std::uint32_t idx : t) {
            if (idx >= geometry.mesh.vertices.size()) {
                throw std::invalid_argument("NewtonianSolver: triangle index out of range");
            }
        }
    }
    if (!std::isfinite(conditions.speed) || !(conditions.speed > 0.0) ||
        !std::isfinite(conditions.density) || !(conditions.density > 0.0) ||
        !std::isfinite(conditions.mach()) || !(conditions.gamma > 1.0) ||
        !std::isfinite(conditions.angle_of_attack) || !std::isfinite(conditions.sideslip)) {
        throw std::invalid_argument("NewtonianSolver: speed, density and Mach must be positive "
                                    "and finite, gamma above 1, angles finite");
    }
    const ReferenceValues& ref = geometry.reference;
    if (!std::isfinite(ref.area) || !(ref.area > 0.0) || !std::isfinite(ref.length) ||
        !(ref.length > 0.0) || !geom::isfinite(ref.moment_centre)) {
        throw std::invalid_argument(
            "NewtonianSolver: reference area and length must be positive and finite");
    }
    if (!std::isfinite(options_.nose_radius) || options_.nose_radius < 0.0) {
        throw std::invalid_argument("NewtonianSolver: nose radius must be >= 0");
    }
    mesh_ = geometry.mesh;
    reference_ = ref;
    conditions_ = conditions;
    result_ = AeroResult{};
    ready_ = true;
}

bool NewtonianSolver::step() {
    if (!ready_) {
        throw std::logic_error("NewtonianSolver::step called before setup");
    }
    const Vector3 v_hat = conditions_.drag_direction();
    const double cp_max = newtonian_cp_max(conditions_.mach(), conditions_.gamma);
    const double q = conditions_.dynamic_pressure();

    AeroResult r;
    const std::size_t count = mesh_.triangles.size();
    r.surface.points.reserve(count);
    r.surface.normals.reserve(count);
    r.surface.areas.reserve(count);
    r.surface.cp.reserve(count);
    for (const geom::Triangle& t : mesh_.triangles) {
        const Vector3& a = mesh_.vertices[t[0]];
        const Vector3& b = mesh_.vertices[t[1]];
        const Vector3& c = mesh_.vertices[t[2]];
        const Vector3 twice = geom::cross(b - a, c - a);
        const double twice_area = geom::norm(twice);
        const Vector3 centre = (a + b + c) / 3.0;
        if (!(twice_area > 0.0)) {
            // A degenerate triangle carries no force.
            r.surface.points.push_back(centre);
            r.surface.normals.push_back({});
            r.surface.areas.push_back(0.0);
            r.surface.cp.push_back(0.0);
            continue;
        }
        const Vector3 n = twice / twice_area;
        const double area = 0.5 * twice_area;
        // APPROXIMATION (A7): impact pressure on windward faces, free-stream
        // pressure in the shadow.
        const double sin_theta = -geom::dot(n, v_hat);
        const double cp = sin_theta > 0.0 ? cp_max * sin_theta * sin_theta : 0.0;
        const Vector3 f = n * (-cp * q * area);
        r.force += f;
        r.moment += geom::cross(centre - reference_.moment_centre, f);
        r.surface.points.push_back(centre);
        r.surface.normals.push_back(n);
        r.surface.areas.push_back(area);
        r.surface.cp.push_back(cp);
    }
    set_coefficients(r, conditions_, reference_);
    r.cd_pressure = r.cd;
    if (options_.nose_radius > 0.0) {
        r.stagnation_heat_flux =
            sutton_graves_heat_flux(conditions_.density, conditions_.speed, options_.nose_radius,
                                    options_.sutton_graves_constant);
    }
    r.iterations = 1;
    r.converged = true;
    r.warnings = check_validity(SolverKind::newtonian, conditions_, reference_.length);
    result_ = std::move(r);
    return true;
}

} // namespace orrery::aero
