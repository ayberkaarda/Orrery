#pragma once

#include <span>
#include <string>
#include <vector>

#include <glm/ext/vector_double3.hpp>

#include "orrery/aero/aero_result.h"
#include "orrery/aero/flow_conditions.h"
#include "orrery/core/result.h"
#include "orrery/geom/naca.h"
#include "orrery/geom/triangle_mesh.h"

// Body surfaces and force vectors of the aero visualisation
// (docs/aero-visualisation.md, "Surface pressure", "Force vectors"): plain
// data built from orrery_geom meshes and orrery_aero results, in body axes
// (x downstream, y span, z up), metres.
namespace orrery::render {

// A body to draw: the mesh and, optionally, the pressure coefficient of each
// triangle (same order as mesh.triangles). Without cp the body is drawn in
// the neutral body colour.
struct AeroSurface {
    geom::TriangleMesh mesh;
    std::vector<double> cp;
};

// The surface of a 3D solver result: triangle i of `mesh` carries
// result.surface.cp[i] (the panel method and Newtonian theory report one
// sample per triangle of the mesh they were given). Fails when the counts
// differ.
[[nodiscard]] core::Result<AeroSurface, std::string>
surface_from_result(const geom::TriangleMesh& mesh, const aero::AeroResult& result);

// The surface of a 2D section result, extruded along y from -span/2 to
// +span/2 in `divisions` strips: panel i (from section point i to i + 1,
// the last one closing back to point 0, as PanelSolver2D numbers them)
// becomes a strip of quads carrying cp[i]; the two ends are closed with
// flat caps in the neutral colour (cp of a cap triangle is NaN). Fails for
// fewer than 3 points, a cp count different from the point count, a
// non-positive span or divisions outside [1, 4096].
[[nodiscard]] core::Result<AeroSurface, std::string>
extrude_section(std::span<const geom::SectionPoint> section, std::span<const double> cp,
                double span, int divisions);

// Per-vertex values for smooth shading: the area-weighted mean of the
// finite values of the triangles around each vertex (NaN where none is
// finite). triangle_values has one value per triangle.
[[nodiscard]] std::vector<double> vertex_values(const geom::TriangleMesh& mesh,
                                                std::span<const double> triangle_values);

// One force component: unit direction, coefficient and force (N, or N per
// metre of span for a 2D result).
struct ForceComponent {
    glm::dvec3 direction{0.0};
    double coefficient = 0.0;
    double force = 0.0;
};

// Drag, lift and side force of a result, drawn as arrows from
// `application_point` (body axes, m).
struct AeroForces {
    glm::dvec3 application_point{0.0};
    ForceComponent drag;
    ForceComponent lift;
    ForceComponent side;
};

// Components along FlowConditions::drag_direction, lift_direction and
// side_direction: coefficients from result.cd, cl, cy and forces from
// result.force projected on those directions.
[[nodiscard]] AeroForces forces_from_result(const aero::AeroResult& result,
                                            const aero::FlowConditions& conditions,
                                            const glm::dvec3& application_point);

} // namespace orrery::render
