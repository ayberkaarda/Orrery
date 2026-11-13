#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "orrery/aero/aero_solver.h"
#include "orrery/core/result.h"
#include "orrery/geom/mesh_import.h"
#include "orrery/geom/mesh_repair.h"
#include "orrery/geom/naca.h"
#include "orrery/geom/reference_quantities.h"
#include "orrery/geom/triangle_mesh.h"
#include "orrery/scene/aero_experiment.h"

// Geometry of the Aero workspace (docs/aero-workspace.md, "Geometry"): a
// GeometryRef of an aero experiment turned into the body the solvers and the
// view use, with its watertightness report and reference quantities.
//
// Body axes are those of orrery_aero (x downstream, y span, z up); every
// body is placed with its centre of volume near the origin. Primitive meshes
// come from orrery_geom and are watertight by construction; user models go
// through import, the confirmed unit scale and up axis, and repair_mesh.
namespace orrery::sim {

// Tessellation used for the primitives (fine enough for the panel method's
// dense solve, which stays below about 4000 panels).
inline constexpr int kRevolutionAround = 48;
inline constexpr double kRevolutionArcStep = 0.08;
inline constexpr int kNacaStations = 48;
inline constexpr int kNacaSpanDivisions = 12;

// What repair did to a user model, and the report before and after.
struct RepairSummary {
    geom::WatertightReport before;
    geom::WatertightReport after;
    std::size_t welded_vertices = 0;
    std::size_t removed_degenerate = 0;
    std::size_t removed_duplicates = 0;
    std::size_t removed_fins = 0;
    std::size_t flipped_triangles = 0;
    std::size_t filled_holes = 0;
    bool orientation_conflict = false;
};

struct PreparedGeometry {
    // Short description for panels and logs ("Sphere, d 0.2 m").
    std::string label;
    scene::GeometryRef ref;
    // A section in the x-z plane (2D solvers); the mesh is then a short
    // extruded preview for the view only.
    bool two_dimensional = false;
    geom::TriangleMesh mesh;
    std::vector<std::uint32_t> trailing_edge;
    std::vector<geom::SectionPoint> section;
    // Of the mesh as used (after repair for user models).
    geom::WatertightReport report;
    std::optional<RepairSummary> repair;
    geom::ReferenceQuantities quantities;
    // Point the moments are taken about: the quarter chord of a NACA body,
    // the centre of volume of anything else.
    geom::Vector3 moment_centre;
    // User models carry no validation case ("unvalidated geometry").
    bool unvalidated = false;
};

struct GeometryError {
    std::string title;  // one line, for the modal's heading
    std::string detail; // the full explanation
};

// Builds a primitive (GeometrySource::primitive). Fails with the reason for
// dimensions the generators reject.
[[nodiscard]] core::Result<PreparedGeometry, GeometryError>
build_primitive(const scene::GeometryRef& ref);

// A user model after import (raw file units and axes) with the importer's
// unit and up-axis estimates, waiting for the user's confirmation.
struct ImportedModel {
    geom::ImportedMesh imported;
    std::string file_name;
    scene::GeometryRef ref; // source, path, url, sha256 and licence filled in
};

// Scales, rotates to Z up, repairs and analyses an imported model with the
// confirmed `unit_scale` (file units to metres) and `up` axis.
[[nodiscard]] core::Result<PreparedGeometry, GeometryError>
prepare_user_model(const ImportedModel& model, double unit_scale, geom::UpAxis up);

// Common unit choices for the confirmation step: name and metres per unit.
struct UnitChoice {
    const char* name;
    double scale;
};
[[nodiscard]] std::vector<UnitChoice> unit_choices();

// The body the solvers receive: mesh or section plus the reference values
// for the chosen area kind (a 2D section always uses chord x 1 m) and the
// reference length (the override, else the body's extent along x; the chord
// of a section).
[[nodiscard]] aero::AeroGeometry to_aero_geometry(const PreparedGeometry& geometry,
                                                  const scene::AeroSolverSettings& settings);

// Reference area and length that to_aero_geometry would use.
[[nodiscard]] aero::ReferenceValues reference_values(const PreparedGeometry& geometry,
                                                     const scene::AeroSolverSettings& settings);

} // namespace orrery::sim
