#pragma once

#include <cstddef>

#include "orrery/geom/triangle_mesh.h"

namespace orrery::geom {

// Which area normalises the aerodynamic coefficients; the user chooses.
enum class ReferenceAreaKind { frontal, planform, wetted };

struct ReferenceOptions {
    Vector3 flow_direction{1.0, 0.0, 0.0};
    Vector3 lift_direction{0.0, 0.0, 1.0};
    // Raster cells along the larger side of a projection (2 .. 8192).
    std::size_t raster_resolution = 1024;
};

struct ReferenceQuantities {
    // Total surface area.
    double wetted_area = 0.0;
    // Silhouette areas seen along the flow and along the lift direction
    // (union of the projected triangles, rasterised).
    // APPROXIMATION (G6): cell-centre coverage on a raster_resolution grid - the
    // error is below the silhouette perimeter times the cell size.
    double frontal_area = 0.0;
    double planform_area = 0.0;
    // Half the sum of |n . d| A: the exact silhouette area of a closed convex
    // mesh and an upper bound for any other closed mesh.
    double frontal_area_bound = 0.0;
    double planform_area_bound = 0.0;
    // Extent along the flow (body length, or chord of an unswept wing) and
    // along the span axis lift x flow.
    double reference_length = 0.0;
    double span = 0.0;
    double volume = 0.0;
    // Centre of the enclosed volume.
    // APPROXIMATION: uniform density (A1 in docs/physics.md) - hollow or
    // layered bodies have a different centre of mass.
    Vector3 centre_of_mass;
};

// The mesh must be closed and outward for volume and centre of mass to be
// meaningful (see analyze_mesh). Throws std::invalid_argument for zero or
// parallel directions or a resolution outside 2 .. 8192.
[[nodiscard]] ReferenceQuantities reference_quantities(const TriangleMesh& mesh,
                                                       const ReferenceOptions& options = {});

[[nodiscard]] double reference_area(const ReferenceQuantities& q, ReferenceAreaKind kind) noexcept;

// Rasterised silhouette area along `direction` (see frontal_area).
[[nodiscard]] double projected_area(const TriangleMesh& mesh, const Vector3& direction,
                                    std::size_t resolution);

// Half the sum of |n . d| A over all triangles (see frontal_area_bound).
[[nodiscard]] double projected_area_bound(const TriangleMesh& mesh,
                                          const Vector3& direction) noexcept;

} // namespace orrery::geom
