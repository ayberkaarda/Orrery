#include "orrery/sim/aero_geometry.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "orrery/aero/panel_method.h"
#include "orrery/geom/panel_mesh.h"
#include "orrery/geom/primitives.h"

namespace orrery::sim {

namespace {

constexpr double kDegree = std::numbers::pi / 180.0;
// Raster of the silhouette areas (reference_quantities): fine enough for a
// fraction of a percent on the primitives, fast on large user models.
constexpr std::size_t kReferenceRaster = 512;
// Span of the extruded preview of a 2D section, in chords.
constexpr double kSectionPreviewSpan = 0.6;

std::string metres(double value) {
    std::ostringstream out;
    out.precision(3);
    out << value << " m";
    return out.str();
}

// Rotations used to face the solids of revolution into the free stream.
// Each is a proper rotation, so outward triangles stay outward.
geom::Vector3 axis_z_to_y(const geom::Vector3& v) noexcept {
    return {v.x, v.z, -v.y}; // -90 degrees about x
}
geom::Vector3 axis_z_to_minus_x(const geom::Vector3& v) noexcept {
    return {-v.z, v.y, v.x}; // -90 degrees about y
}
geom::Vector3 axis_z_to_x(const geom::Vector3& v) noexcept {
    return {v.z, v.y, -v.x}; // +90 degrees about y
}

template <typename F> void transform(geom::TriangleMesh& mesh, F&& f) {
    for (geom::Vector3& v : mesh.vertices) {
        v = f(v);
    }
}

void centre_on_bounds(geom::TriangleMesh& mesh) {
    const geom::Vector3 centre = geom::bounds(mesh).centre();
    for (geom::Vector3& v : mesh.vertices) {
        v -= centre;
    }
}

geom::RevolutionResolution revolution() {
    return geom::RevolutionResolution{kRevolutionAround, kRevolutionArcStep};
}

geom::CapsuleShape scaled_capsule(const std::string& name, double diameter) {
    geom::CapsuleShape shape =
        name == "orion" ? geom::CapsuleShape::orion() : geom::CapsuleShape::apollo();
    const double k = diameter / shape.max_diameter;
    shape.max_diameter *= k;
    shape.nose_radius *= k;
    shape.shoulder_radius *= k;
    shape.top_diameter *= k;
    return shape;
}

geom::Naca4 airfoil_of(const scene::GeometryRef& ref) {
    const auto airfoil = geom::Naca4::from_designation(ref.naca);
    if (!airfoil) {
        throw std::invalid_argument("'" + ref.naca + "' is not a NACA 4-digit designation");
    }
    return *airfoil;
}

geom::ReferenceQuantities quantities_of(const geom::TriangleMesh& mesh) {
    geom::ReferenceOptions options;
    options.raster_resolution = kReferenceRaster;
    return geom::reference_quantities(mesh, options);
}

PreparedGeometry build(const scene::GeometryRef& ref) {
    PreparedGeometry out;
    out.ref = ref;
    const scene::ShapeDimensions& d = ref.dimensions;
    std::string label(scene::display_name(ref.shape));
    bool naca = false;
    switch (ref.shape) {
        case scene::AeroShape::sphere:
            out.mesh = geom::sphere_profile(0.5 * d.diameter_m).mesh(revolution());
            label += ", d " + metres(d.diameter_m);
            break;
        case scene::AeroShape::cube:
            out.mesh = geom::cube_mesh(d.side_m);
            label += ", side " + metres(d.side_m);
            break;
        case scene::AeroShape::cylinder:
            out.mesh = geom::cylinder_profile(0.5 * d.diameter_m, d.length_m).mesh(revolution());
            transform(out.mesh, axis_z_to_y);
            label += ", d " + metres(d.diameter_m) + ", span " + metres(d.length_m);
            break;
        case scene::AeroShape::cone:
            out.mesh = geom::cone_profile(0.5 * d.diameter_m, d.length_m).mesh(revolution());
            transform(out.mesh, axis_z_to_minus_x);
            label += ", d " + metres(d.diameter_m) + ", length " + metres(d.length_m);
            break;
        case scene::AeroShape::ellipsoid:
            out.mesh = geom::Ellipsoid{0.5 * d.length_m, 0.5 * d.width_m, 0.5 * d.height_m}.mesh(
                revolution());
            label += ", " + metres(d.length_m) + " long";
            break;
        case scene::AeroShape::flat_plate:
            out.mesh = geom::flat_plate_mesh(d.chord_m, d.span_m, d.thickness_m);
            label += ", " + metres(d.chord_m) + " x " + metres(d.span_m);
            break;
        case scene::AeroShape::wedge:
            out.mesh = geom::Wedge{d.length_m, d.half_angle_deg * kDegree, d.span_m}.mesh();
            label += ", " + metres(d.length_m) + " long";
            break;
        case scene::AeroShape::capsule:
            out.mesh = geom::capsule_profile(scaled_capsule(ref.capsule, d.diameter_m))
                           .mesh(revolution());
            transform(out.mesh, axis_z_to_x);
            label = (ref.capsule == "orion" ? "Orion-type capsule, d " : "Apollo-type capsule, d ") +
                    metres(d.diameter_m);
            break;
        case scene::AeroShape::naca_wing: {
            geom::NacaWing wing = geom::naca_wing(airfoil_of(ref), d.chord_m, d.span_m,
                                                  kNacaStations, kNacaSpanDivisions);
            out.mesh = std::move(wing.mesh);
            out.trailing_edge = std::move(wing.trailing_edge);
            label = "NACA " + ref.naca + " wing, chord " + metres(d.chord_m) + ", span " +
                    metres(d.span_m);
            naca = true;
            break;
        }
        case scene::AeroShape::naca_section: {
            const geom::Naca4 airfoil = airfoil_of(ref);
            const aero::AeroGeometry section =
                aero::naca_section_geometry(airfoil, d.chord_m, kNacaStations);
            out.section = section.section;
            out.two_dimensional = true;
            out.moment_centre = section.reference.moment_centre;
            geom::NacaWing preview = geom::naca_wing(airfoil, d.chord_m,
                                                     kSectionPreviewSpan * d.chord_m,
                                                     kNacaStations, 1);
            out.mesh = std::move(preview.mesh);
            label = "NACA " + ref.naca + " section, chord " + metres(d.chord_m);
            break;
        }
    }
    if (!out.two_dimensional) {
        const geom::Bounds before = geom::bounds(out.mesh);
        centre_on_bounds(out.mesh);
        if (naca) {
            // Quarter chord on the chord line, after centring.
            out.moment_centre = {before.min.x - before.centre().x + 0.25 * d.chord_m, 0.0, 0.0};
        }
    }
    out.label = std::move(label);
    out.report = geom::analyze_mesh(out.mesh);
    out.quantities = quantities_of(out.mesh);
    if (!naca && !out.two_dimensional) {
        out.moment_centre = out.quantities.centre_of_mass;
    }
    return out;
}

} // namespace

core::Result<PreparedGeometry, GeometryError> build_primitive(const scene::GeometryRef& ref) {
    if (ref.source != scene::GeometrySource::primitive) {
        return core::failure(
            GeometryError{"Not a primitive", "this geometry is a user model; import it instead"});
    }
    try {
        return build(ref);
    } catch (const std::exception& error) {
        return core::failure(GeometryError{
            "Cannot build the " + std::string(scene::display_name(ref.shape)), error.what()});
    }
}

core::Result<PreparedGeometry, GeometryError>
prepare_user_model(const ImportedModel& model, double unit_scale, geom::UpAxis up) {
    if (!(unit_scale > 0.0) || !std::isfinite(unit_scale)) {
        return core::failure(
            GeometryError{"Invalid unit scale", "the scale from file units to metres must be "
                                                "positive and finite"});
    }
    try {
        geom::TriangleMesh mesh = model.imported.mesh;
        geom::apply_unit_scale(mesh, unit_scale);
        geom::convert_to_z_up(mesh, up);
        centre_on_bounds(mesh);
        const geom::RepairResult repaired = geom::repair_mesh(mesh);

        PreparedGeometry out;
        out.ref = model.ref;
        out.ref.unit_scale = unit_scale;
        out.ref.up_axis = up;
        out.label = model.file_name.empty() ? std::string("User model") : model.file_name;
        out.mesh = repaired.mesh;
        out.report = repaired.after;
        RepairSummary summary;
        summary.before = repaired.before;
        summary.after = repaired.after;
        summary.welded_vertices = repaired.welded_vertices;
        summary.removed_degenerate = repaired.removed_degenerate;
        summary.removed_duplicates = repaired.removed_duplicates;
        summary.removed_fins = repaired.removed_fins;
        summary.flipped_triangles = repaired.flipped_triangles;
        summary.filled_holes = repaired.filled_holes;
        summary.orientation_conflict = repaired.orientation_conflict;
        out.repair = summary;
        if (out.mesh.triangles.empty()) {
            return core::failure(GeometryError{
                "Nothing left after repair",
                "every triangle of " + out.label +
                    " was degenerate or duplicated; the model encloses no surface"});
        }
        out.trailing_edge = geom::detect_trailing_edge(out.mesh);
        out.quantities = quantities_of(out.mesh);
        out.moment_centre = out.quantities.centre_of_mass;
        out.unvalidated = true;
        return out;
    } catch (const std::exception& error) {
        return core::failure(GeometryError{"Cannot prepare " + model.file_name, error.what()});
    }
}

std::vector<UnitChoice> unit_choices() {
    return {{"metres", 1.0},     {"centimetres", 0.01}, {"millimetres", 0.001},
            {"inches", 0.0254},  {"feet", 0.3048}};
}

aero::ReferenceValues reference_values(const PreparedGeometry& geometry,
                                       const scene::AeroSolverSettings& settings) {
    aero::ReferenceValues reference;
    if (geometry.two_dimensional) {
        // Per unit span: chord x 1 m, the chord as the length.
        const double chord = geometry.ref.dimensions.chord_m;
        reference.area = chord;
        reference.length = settings.reference_length_m.value_or(chord);
    } else {
        reference.area = geom::reference_area(geometry.quantities, settings.reference_area);
        reference.length =
            settings.reference_length_m.value_or(geometry.quantities.reference_length);
    }
    reference.moment_centre = geometry.moment_centre;
    return reference;
}

aero::AeroGeometry to_aero_geometry(const PreparedGeometry& geometry,
                                    const scene::AeroSolverSettings& settings) {
    aero::AeroGeometry out;
    if (geometry.two_dimensional) {
        out.section = geometry.section;
    } else {
        out.mesh = geometry.mesh;
        out.trailing_edge = geometry.trailing_edge;
    }
    out.reference = reference_values(geometry, settings);
    return out;
}

} // namespace orrery::sim
