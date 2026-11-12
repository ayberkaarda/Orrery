#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

#include "orrery/core/result.h"
#include "orrery/geom/triangle_mesh.h"

namespace orrery::geom {

// Bounds on untrusted mesh data. Counts are checked before anything is
// allocated or read, so a hostile header cannot force a large allocation.
struct MeshLimits {
    std::size_t max_vertices = 10'000'000;
    std::size_t max_triangles = 10'000'000;
    // Largest coordinate magnitude accepted (raw file units): keeps every
    // product of two coordinates far from overflow.
    double max_abs_coordinate = 1e12;
};

enum class MeshErrorCode {
    empty,
    too_many_vertices,
    too_many_triangles,
    malformed_array,
    non_finite_coordinate,
    coordinate_out_of_range,
    index_out_of_range,
};

struct MeshValidationError {
    MeshErrorCode code = MeshErrorCode::empty;
    // Offending vertex or triangle index, when there is one.
    std::size_t element = 0;
    std::string message;
};

// Checks a mesh before any other code uses it: counts within the limits,
// every coordinate finite and within range, every index below the vertex
// count. Empty if the mesh is acceptable.
[[nodiscard]] std::optional<MeshValidationError> validate_mesh(const TriangleMesh& mesh,
                                                               const MeshLimits& limits);

// Builds a mesh from flat untrusted arrays (x, y, z per vertex; three indices
// per triangle) after checking them with the same rules.
[[nodiscard]] core::Result<TriangleMesh, MeshValidationError>
mesh_from_arrays(std::span<const double> xyz, std::span<const std::uint32_t> indices,
                 const MeshLimits& limits);

} // namespace orrery::geom
