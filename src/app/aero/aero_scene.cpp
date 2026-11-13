#include "orrery/app/aero_scene.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
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

core::Result<render::AeroSurface, std::string> view_surface(const sim::PreparedGeometry& geometry,
                                                            const std::vector<double>& cp,
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
        // The section in the wind frame's x-z plane, extruded along y: the
        // frame of the 2D view (aero_view_data.h).
        auto extruded = render::extrude_section(
            wind, values, kSectionViewSpan * geometry.ref.dimensions.chord_m, 4);
        if (!extruded) {
            return core::failure(std::string(extruded.error()));
        }
        return std::move(extruded).value();
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

namespace {

// A 2D lattice in the view's x-z plane: two identical layers of nodes at
// y = -dx/2 and +dx/2, so that the field is an ordinary 3D grid around the
// plane y = 0 that the section plane, the streamlines and the particles use
// like any other (derivatives along y are zero). Pressure is the pressure
// coefficient c_s^2 (rho - 1) / (u_ref^2 / 2), as flow_field_from reports it.
render::FlowField section_flow(const aero::lbm::MacroFields2D& fields,
                               const sim::FlowSnapshot& snapshot) {
    constexpr double kCs2 = 1.0 / 3.0;
    render::FlowField field;
    render::GridGeometry grid;
    grid.dims = {fields.nx, 2, fields.ny};
    grid.origin = {snapshot.origin.x, -0.5 * snapshot.spacing, snapshot.origin.z};
    grid.spacing = snapshot.spacing;
    const std::size_t plane = fields.nx * fields.ny;
    field.velocity.grid = grid;
    field.velocity.values.reserve(2 * plane);
    render::ScalarGrid pressure;
    pressure.grid = grid;
    pressure.values.reserve(2 * plane);
    field.solid.reserve(2 * plane);
    const double q = snapshot.reference_speed > 0.0
                         ? 0.5 * snapshot.reference_speed * snapshot.reference_speed
                         : 0.0;
    // Layout i + nx (j + 2 k): row j = 0 and j = 1 of each lattice row k.
    for (std::size_t k = 0; k < fields.ny; ++k) {
        for (std::size_t j = 0; j < 2; ++j) {
            for (std::size_t i = 0; i < fields.nx; ++i) {
                const std::size_t n = i + fields.nx * k;
                field.velocity.values.emplace_back(snapshot.velocity_scale * fields.velocity_x[n],
                                                   0.0,
                                                   snapshot.velocity_scale * fields.velocity_y[n]);
                const double gauge = kCs2 * (fields.density[n] - 1.0);
                pressure.values.push_back(q > 0.0 ? gauge / q : gauge);
                field.solid.push_back(n < fields.solid.size() ? fields.solid[n] : 0);
            }
        }
    }
    field.pressure = std::move(pressure);
    field.pressure_label = q > 0.0 ? "Cp" : "p";
    return field;
}

} // namespace

render::FlowField view_flow(const sim::FlowSnapshot& snapshot) {
    if (const auto* section = std::get_if<aero::lbm::MacroFields2D>(&snapshot.fields)) {
        return section_flow(*section, snapshot);
    }
    render::LatticeMapping mapping;
    mapping.origin = to_glm(snapshot.origin);
    mapping.spacing = snapshot.spacing;
    mapping.velocity_scale = snapshot.velocity_scale;
    mapping.reference_speed = snapshot.reference_speed;
    return render::flow_field_from(std::get<aero::lbm::MacroFields3D>(snapshot.fields), mapping);
}

render::AeroForces view_forces(const aero::AeroResult& result,
                               const aero::FlowConditions& conditions,
                               const sim::PreparedGeometry& geometry, const sim::ViewFrame& frame) {
    aero::FlowConditions c = conditions;
    if (frame.two_dimensional) {
        c.sideslip = 0.0;
    }
    render::AeroForces forces =
        render::forces_from_result(result, c, to_glm(geometry.moment_centre));
    forces.application_point = view_direction(forces.application_point, frame);
    forces.drag.direction = view_direction(forces.drag.direction, frame);
    forces.lift.direction = view_direction(forces.lift.direction, frame);
    forces.side.direction = view_direction(forces.side.direction, frame);
    return forces;
}

double cp_range(const std::vector<double>& cp) {
    std::vector<double> magnitude;
    magnitude.reserve(cp.size());
    for (const double value : cp) {
        if (std::isfinite(value)) {
            magnitude.push_back(std::abs(value));
        }
    }
    if (magnitude.empty()) {
        return 1.0;
    }
    const auto at = static_cast<std::ptrdiff_t>(0.98 * static_cast<double>(magnitude.size() - 1));
    std::nth_element(magnitude.begin(), magnitude.begin() + at, magnitude.end());
    return std::max(1.0, magnitude[static_cast<std::size_t>(at)]);
}

render::AeroViewOptions view_options(const AeroLayers& layers, bool two_dimensional,
                                     const geom::Bounds& body_bounds,
                                     const std::optional<render::GridGeometry>& grid,
                                     const std::vector<double>& cp) {
    render::AeroViewOptions options;
    const double range = cp_range(cp);
    options.surface.scale.mode = render::ScaleMode::fixed;
    options.surface.scale.min = -range;
    options.surface.scale.max = range;
    const double size = largest_extent(body_bounds);
    const geom::Vector3 centre = body_bounds.centre();
    if (grid && layers.section) {
        // A light sheet normal to the span: at y = 0 for a 2D run (between
        // its two node layers), anywhere across the span for a 3D run.
        render::SectionPlane plane;
        plane.axis = render::Axis::y;
        const glm::dvec3 lo = grid->origin;
        const glm::dvec3 hi = grid->upper_corner();
        const double fraction = std::clamp(layers.section_fraction, 0.0, 1.0);
        plane.position = two_dimensional ? 0.0 : lo.y + fraction * (hi.y - lo.y);
        plane.quantity = layers.section_quantity;
        plane.scale.mode = render::is_signed(layers.section_quantity)
                               ? render::ScaleMode::symmetric
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
            // A vertical rake in the light sheet.
            constexpr int kSeeds = 28;
            for (int k = 0; k < kSeeds; ++k) {
                const double t = static_cast<double>(k) / static_cast<double>(kSeeds - 1);
                const double z = clamp_into(centre.z + (t - 0.5) * 1.6 * size, lo.z, hi.z);
                options.streamlines.seeds.emplace_back(x, 0.0, z);
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
                                  bool with_wake, double aspect, const render::CameraLens& lens) {
    const double size = largest_extent(body_bounds);
    // Frame the body with some of its wake: more downstream than upstream.
    geom::Bounds framed = body_bounds;
    framed.min.x -= (with_wake ? 0.6 : 0.15) * size;
    framed.max.x += (with_wake ? 1.4 : 0.15) * size;
    if (two_dimensional && with_wake) {
        // Nearly square-on to the light sheet: the section picture, with the
        // short wing's suction side still in view.
        framed.min.z -= 0.3 * size;
        framed.max.z += 0.3 * size;
        return render::frame_bounds(framed, aspect, lens, -0.5 * std::numbers::pi - 0.35, 0.45);
    }
    framed.min.y -= 0.3 * size;
    framed.max.y += 0.3 * size;
    // From upstream, to the side and above: the windward faces and the
    // suction side show, the free stream still runs left to right.
    return render::frame_bounds(framed, aspect, lens, -2.0, 0.45);
}

} // namespace orrery::app
