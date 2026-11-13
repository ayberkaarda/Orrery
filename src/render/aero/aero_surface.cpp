#include "orrery/render/aero_surface.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace orrery::render {

namespace {

constexpr int kMaxDivisions = 4096;

glm::dvec3 to_glm(const geom::Vector3& v) noexcept {
    return {v.x, v.y, v.z};
}

} // namespace

core::Result<AeroSurface, std::string> surface_from_result(const geom::TriangleMesh& mesh,
                                                           const aero::AeroResult& result) {
    if (result.surface.cp.size() != mesh.triangles.size()) {
        return core::failure("aero surface: " + std::to_string(result.surface.cp.size()) +
                             " Cp samples for " + std::to_string(mesh.triangles.size()) +
                             " triangles");
    }
    AeroSurface surface;
    surface.mesh = mesh;
    surface.cp = result.surface.cp;
    return surface;
}

core::Result<AeroSurface, std::string> extrude_section(std::span<const geom::SectionPoint> section,
                                                       std::span<const double> cp, double span,
                                                       int divisions) {
    const std::size_t n = section.size();
    if (n < 3) {
        return core::failure(std::string("aero surface: a section needs at least 3 points"));
    }
    if (cp.size() != n) {
        return core::failure("aero surface: " + std::to_string(cp.size()) + " Cp samples for " +
                             std::to_string(n) + " panels");
    }
    if (!(span > 0.0) || !std::isfinite(span)) {
        return core::failure(std::string("aero surface: span must be positive and finite"));
    }
    if (divisions < 1 || divisions > kMaxDivisions) {
        return core::failure(std::string("aero surface: divisions must be in [1, 4096]"));
    }
    const auto rows = static_cast<std::size_t>(divisions) + 1;
    AeroSurface surface;
    geom::TriangleMesh& mesh = surface.mesh;
    mesh.vertices.reserve(rows * n + 2);
    for (std::size_t r = 0; r < rows; ++r) {
        const double y =
            -0.5 * span + span * static_cast<double>(r) / static_cast<double>(rows - 1);
        for (const geom::SectionPoint& p : section) {
            mesh.vertices.push_back({p.x, y, p.z});
        }
    }
    const auto vertex = [n](std::size_t row, std::size_t i) {
        return static_cast<std::uint32_t>(row * n + (i % n));
    };
    mesh.triangles.reserve(2 * n * (rows - 1) + 2 * n);
    surface.cp.reserve(2 * n * (rows - 1) + 2 * n);
    // The section is counter-clockwise seen with x right and z up (looking
    // along +y), so (a, c, b) below winds counter-clockwise seen from outside.
    for (std::size_t r = 0; r + 1 < rows; ++r) {
        for (std::size_t i = 0; i < n; ++i) {
            const std::uint32_t a = vertex(r, i);
            const std::uint32_t b = vertex(r, i + 1);
            const std::uint32_t c = vertex(r + 1, i + 1);
            const std::uint32_t d = vertex(r + 1, i);
            mesh.triangles.push_back({a, c, b});
            mesh.triangles.push_back({a, d, c});
            surface.cp.push_back(cp[i]);
            surface.cp.push_back(cp[i]);
        }
    }
    // End caps: fans around the section centroid (airfoil sections are star
    // shaped about it), facing -y at the first row and +y at the last.
    double cx = 0.0;
    double cz = 0.0;
    for (const geom::SectionPoint& p : section) {
        cx += p.x;
        cz += p.z;
    }
    cx /= static_cast<double>(n);
    cz /= static_cast<double>(n);
    const auto first_centre = static_cast<std::uint32_t>(mesh.vertices.size());
    mesh.vertices.push_back({cx, -0.5 * span, cz});
    const auto last_centre = static_cast<std::uint32_t>(mesh.vertices.size());
    mesh.vertices.push_back({cx, 0.5 * span, cz});
    constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t i = 0; i < n; ++i) {
        mesh.triangles.push_back({first_centre, vertex(0, i), vertex(0, i + 1)});
        mesh.triangles.push_back({last_centre, vertex(rows - 1, i + 1), vertex(rows - 1, i)});
        surface.cp.push_back(kNan);
        surface.cp.push_back(kNan);
    }
    return surface;
}

// APPROXIMATION (V8): Cp smoothing - per-panel Cp averaged to the vertices
// (angle-weighted) and interpolated across each triangle - display only; the
// solver's integrated loads are unaffected; across sharp edges the two sides
// are averaged at shared vertices.
std::vector<double> vertex_values(const geom::TriangleMesh& mesh,
                                  std::span<const double> triangle_values) {
    std::vector<double> sum(mesh.vertices.size(), 0.0);
    std::vector<double> weight(mesh.vertices.size(), 0.0);
    const std::size_t count = std::min(mesh.triangles.size(), triangle_values.size());
    for (std::size_t t = 0; t < count; ++t) {
        const double value = triangle_values[t];
        if (!std::isfinite(value)) {
            continue;
        }
        const geom::Triangle& tri = mesh.triangles[t];
        if (tri[0] >= sum.size() || tri[1] >= sum.size() || tri[2] >= sum.size()) {
            continue;
        }
        for (std::size_t c = 0; c < 3; ++c) {
            const std::uint32_t v = tri[c];
            const geom::Vector3 e1 = mesh.vertices[tri[(c + 1) % 3]] - mesh.vertices[v];
            const geom::Vector3 e2 = mesh.vertices[tri[(c + 2) % 3]] - mesh.vertices[v];
            const double angle = std::atan2(geom::norm(geom::cross(e1, e2)), geom::dot(e1, e2));
            sum[v] += angle * value;
            weight[v] += angle;
        }
    }
    std::vector<double> result(mesh.vertices.size(), std::numeric_limits<double>::quiet_NaN());
    for (std::size_t v = 0; v < result.size(); ++v) {
        if (weight[v] > 0.0) {
            result[v] = sum[v] / weight[v];
        }
    }
    return result;
}

AeroForces forces_from_result(const aero::AeroResult& result,
                              const aero::FlowConditions& conditions,
                              const glm::dvec3& application_point) {
    const glm::dvec3 force = to_glm(result.force);
    const auto component = [&](const geom::Vector3& direction, double coefficient) {
        const glm::dvec3 d = to_glm(direction);
        return ForceComponent{d, coefficient, force.x * d.x + force.y * d.y + force.z * d.z};
    };
    AeroForces forces;
    forces.application_point = application_point;
    forces.drag = component(conditions.drag_direction(), result.cd);
    forces.lift = component(conditions.lift_direction(), result.cl);
    forces.side = component(conditions.side_direction(), result.cy);
    return forces;
}

} // namespace orrery::render
