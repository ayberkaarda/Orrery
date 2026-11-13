#include "orrery/app/aero_scene.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>
#include <variant>

namespace orrery::app {

namespace {

glm::dvec3 to_glm(const geom::Vector3& v) noexcept {
    return {v.x, v.y, v.z};
}

double largest_extent(const geom::Bounds& bounds) noexcept {
    const geom::Vector3 e = bounds.extent();
    return std::max({e.x, e.y, e.z, 1.0e-6});
}

glm::dvec3 view_direction(const glm::dvec3& body, const sim::ViewFrame& frame) {
    return to_glm(sim::to_view({body.x, body.y, body.z}, frame));
}

double clamp_into(double value, double lo, double hi) noexcept {
    return std::clamp(value, std::min(lo, hi), std::max(lo, hi));
}

} // namespace

geom::Bounds view_bounds(const geom::TriangleMesh& mesh) {
    return geom::bounds(mesh);
}

core::Result<render::AeroSurface, std::string>
view_surface(const sim::PreparedGeometry& geometry, const std::vector<double>& cp,
             const sim::ViewFrame& frame) {
    if (geometry.two_dimensional) {
        std::vector<geom::SectionPoint> wind;
        wind.reserve(geometry.section.size());
        for (const geom::SectionPoint& p : geometry.section) {
            wind.push_back(sim::section_to_wind(p, frame));
        }
        std::vector<double> values = cp;
        if (values.size() != wind.size()) {
            values.assign(wind.size(), std::numeric_limits<double>::quiet_NaN());
        }
        auto extruded = render::extrude_section(
            wind, values, kSectionViewSpan * geometry.ref.dimensions.chord_m, 1);
        if (!extruded) {
            return core::failure(std::string(extruded.error()));
        }
        render::AeroSurface surface = std::move(extruded).value();
        // extrude_section spans y with the section in x-z; the 2D view keeps
        // the section in its x-y plane (a proper rotation: winding is kept).
        for (geom::Vector3& v : surface.mesh.vertices) {
            v = geom::Vector3{v.x, v.z, -v.y};
        }
        return surface;
    }
    render::AeroSurface surface;
    surface.mesh = geometry.mesh;
    for (geom::Vector3& v : surface.mesh.vertices) {
        v = sim::to_view(v, frame);
    }
    if (cp.size() == surface.mesh.triangles.size()) {
        surface.cp = cp;
    }
    return surface;
}

render::FlowField view_flow(const sim::FlowSnapshot& snapshot) {
    render::LatticeMapping mapping;
    mapping.origin = to_glm(snapshot.origin);
    mapping.spacing = snapshot.spacing;
    mapping.velocity_scale = snapshot.velocity_scale;
    mapping.reference_speed = snapshot.reference_speed;
    return std::visit([&](const auto& fields) { return render::flow_field_from(fields, mapping); },
                      snapshot.fields);
}

render::AeroForces view_forces(const aero::AeroResult& result,
                               const aero::FlowConditions& conditions,
                               const sim::PreparedGeometry& geometry,
                               const sim::ViewFrame& frame) {
    aero::FlowConditions c = conditions;
    if (frame.two_dimensional) {
        c.sideslip = 0.0;
    }
    render::AeroForces forces = render::forces_from_result(result, c, to_glm(geometry.moment_centre));
    forces.application_point = view_direction(forces.application_point, frame);
    forces.drag.direction = view_direction(forces.drag.direction, frame);
    forces.lift.direction = view_direction(forces.lift.direction, frame);
    forces.side.direction = view_direction(forces.side.direction, frame);
    return forces;
}

render::AeroViewOptions view_options(const AeroLayers& layers, bool two_dimensional,
                                     const geom::Bounds& body_bounds,
                                     const std::optional<render::GridGeometry>& grid) {
    render::AeroViewOptions options;
    const double size = largest_extent(body_bounds);
    const geom::Vector3 centre = body_bounds.centre();
    if (grid && layers.section) {
        render::SectionPlane plane;
        plane.axis = two_dimensional ? render::Axis::z : render::Axis::y;
        const glm::dvec3 lo = grid->origin;
        const glm::dvec3 hi = grid->upper_corner();
        const double fraction = std::clamp(layers.section_fraction, 0.0, 1.0);
        plane.position = two_dimensional ? lo.z : lo.y + fraction * (hi.y - lo.y);
        plane.quantity = layers.section_quantity;
        plane.scale.mode = render::is_signed(layers.section_quantity) ? render::ScaleMode::symmetric
                                                                       : render::ScaleMode::automatic;
        plane.opacity = 0.92;
        options.sections.push_back(plane);
    }
    if (grid) {
        const glm::dvec3 lo = grid->origin;
        const glm::dvec3 hi = grid->upper_corner();
        const double x = clamp_into(body_bounds.min.x - 0.75 * size, lo.x, hi.x);
        options.streamlines.enabled = layers.streamlines;
        options.particles.enabled = layers.particles;
        if (two_dimensional) {
            constexpr int kSeeds = 28;
            for (int k = 0; k < kSeeds; ++k) {
                const double t = static_cast<double>(k) / static_cast<double>(kSeeds - 1);
                const double y = clamp_into(centre.y + (t - 0.5) * 3.0 * size, lo.y, hi.y);
                options.streamlines.seeds.emplace_back(x, y, lo.z);
            }
        } else {
            constexpr int kRows = 7;
            constexpr int kColumns = 9;
            for (int j = 0; j < kColumns; ++j) {
                for (int k = 0; k < kRows; ++k) {
                    const double u = static_cast<double>(j) / static_cast<double>(kColumns - 1);
                    const double v = static_cast<double>(k) / static_cast<double>(kRows - 1);
                    const double y = clamp_into(centre.y + (u - 0.5) * 1.6 * size, lo.y, hi.y);
                    const double z = clamp_into(centre.z + (v - 0.5) * 1.2 * size, lo.z, hi.z);
                    options.streamlines.seeds.emplace_back(x, y, z);
                }
            }
        }
        options.iso.enabled = layers.q_iso && !two_dimensional;
        options.iso.level = layers.q_level;
    }
    options.forces.enabled = layers.forces;
    options.forces.length_per_coefficient = 0.8 * size;
    options.forces.side = !two_dimensional;
    return options;
}

render::TurntableMode view_camera(const geom::Bounds& body_bounds, bool two_dimensional,
                                  double aspect, const render::CameraLens& lens) {
    const double size = largest_extent(body_bounds);
    // Frame the body with some of its wake: more downstream than upstream.
    geom::Bounds framed = body_bounds;
    framed.min.x -= 0.6 * size;
    framed.max.x += 1.4 * size;
    if (two_dimensional) {
        framed.min.y -= 0.5 * size;
        framed.max.y += 0.5 * size;
        // Looking down on the section plane, flow left to right, lift up.
        return render::frame_bounds(framed, aspect, lens, -0.5 * std::numbers::pi, 1.45);
    }
    framed.min.y -= 0.3 * size;
    framed.max.y += 0.3 * size;
    return render::frame_bounds(framed, aspect, lens, -1.15, 0.38);
}

} // namespace orrery::app
