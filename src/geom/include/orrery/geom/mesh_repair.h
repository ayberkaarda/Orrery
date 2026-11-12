#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "orrery/geom/triangle_mesh.h"

namespace orrery::geom {

// Watertightness and quality report of a triangle mesh (docs/geometry-import.md).
//
// Vertices at the same position (within the weld tolerance) count as one
// vertex. Degenerate triangles (two equal welded corners, or area below 1e-12
// of the squared longest edge) and duplicate triangles (the same three welded
// corners as an earlier triangle, in either orientation) are counted and then
// left out: every edge and orientation figure describes the remaining
// triangles.
struct WatertightReport {
    std::size_t vertices = 0;
    std::size_t triangles = 0;
    std::size_t unreferenced_vertices = 0;
    // Vertices merged into an earlier vertex at the same position.
    std::size_t duplicate_vertices = 0;
    std::size_t degenerate_triangles = 0;
    std::size_t duplicate_triangles = 0;
    std::size_t edges = 0;
    // Edges used by exactly one triangle.
    std::size_t boundary_edges = 0;
    // Edges used by three or more triangles.
    std::size_t non_manifold_edges = 0;
    // Edges used by two triangles in the same direction (one of them flipped).
    std::size_t inconsistent_edges = 0;
    // Closed loops of boundary edges (holes) and boundary chains that do not
    // close (for example the free edges of a fin on a non-manifold edge).
    std::size_t boundary_loops = 0;
    std::size_t open_boundary_chains = 0;
    // Edge-connected components.
    std::size_t components = 0;
    // Divergence-theorem volume of the remaining triangles: positive when a
    // closed surface faces outward.
    double signed_volume = 0.0;

    [[nodiscard]] bool watertight() const noexcept {
        return triangles > degenerate_triangles + duplicate_triangles && boundary_edges == 0 &&
               non_manifold_edges == 0 && inconsistent_edges == 0;
    }
    [[nodiscard]] bool outward() const noexcept { return signed_volume > 0.0; }
};

// weld_tolerance: absolute distance (mesh units); 0 merges only identical
// positions. A positive tolerance merges points in the same cube cell of that
// size (points up to sqrt(3) times the tolerance apart). Throws
// std::invalid_argument for a negative or non-finite tolerance, or one so
// small that a coordinate divided by it is not finite.
[[nodiscard]] WatertightReport analyze_mesh(const TriangleMesh& mesh, double weld_tolerance = 0.0);

struct RepairOptions {
    double weld_tolerance = 0.0;
    // Remove fins: triangles with two or more boundary edges attached to the
    // rest through a non-manifold edge.
    bool remove_fins = true;
    bool fix_orientation = true;
    bool fill_holes = true;
    // Holes with more boundary edges are left open.
    std::size_t max_hole_edges = 10'000;
};

struct RepairResult {
    TriangleMesh mesh;
    WatertightReport before;
    WatertightReport after;
    std::size_t welded_vertices = 0;
    std::size_t removed_degenerate = 0;
    std::size_t removed_duplicates = 0;
    std::size_t removed_fins = 0;
    std::size_t flipped_triangles = 0;
    std::size_t filled_holes = 0;
    std::size_t added_triangles = 0;
    // A component whose orientation cannot be made consistent (for example a
    // Moebius strip or a non-manifold junction).
    bool orientation_conflict = false;
};

// Repairs in a fixed order: weld, drop degenerate and duplicate triangles,
// drop fins, make the orientation consistent per component, fill holes,
// turn each closed component to face away from the solid (outward for an
// outer wall or an island, into the void for a cavity wall, decided by how
// many other closed components enclose it), drop unreferenced vertices. The
// input must have passed validate_mesh. Throws std::invalid_argument for a
// weld tolerance analyze_mesh rejects.
// APPROXIMATION (G4): holes are filled by a fan from the loop centroid -
// simple and always closes the loop - fine for small, nearly planar holes;
// large or curved holes get a flat or self-intersecting patch.
[[nodiscard]] RepairResult repair_mesh(const TriangleMesh& mesh, const RepairOptions& options = {});

} // namespace orrery::geom
