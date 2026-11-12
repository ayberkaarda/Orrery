#include "orrery/geom/mesh_validation.h"

#include <cmath>
#include <string>
#include <utility>

namespace orrery::geom {

namespace {

MeshValidationError error(MeshErrorCode code, std::size_t element, std::string message) {
    return MeshValidationError{code, element, std::move(message)};
}

std::optional<MeshValidationError> check_counts(std::size_t vertices, std::size_t triangles,
                                                const MeshLimits& limits) {
    if (vertices == 0 || triangles == 0) {
        return error(MeshErrorCode::empty, 0, "mesh has no vertices or no triangles");
    }
    if (vertices > limits.max_vertices) {
        return error(MeshErrorCode::too_many_vertices, vertices,
                     "vertex count " + std::to_string(vertices) + " exceeds the limit " +
                         std::to_string(limits.max_vertices));
    }
    if (triangles > limits.max_triangles) {
        return error(MeshErrorCode::too_many_triangles, triangles,
                     "triangle count " + std::to_string(triangles) + " exceeds the limit " +
                         std::to_string(limits.max_triangles));
    }
    return std::nullopt;
}

std::optional<MeshValidationError> check_coordinate(double value, std::size_t vertex,
                                                    const MeshLimits& limits) {
    if (!std::isfinite(value)) {
        return error(MeshErrorCode::non_finite_coordinate, vertex,
                     "vertex " + std::to_string(vertex) + " has a NaN or infinite coordinate");
    }
    if (std::abs(value) > limits.max_abs_coordinate) {
        return error(MeshErrorCode::coordinate_out_of_range, vertex,
                     "vertex " + std::to_string(vertex) + " coordinate exceeds the limit");
    }
    return std::nullopt;
}

std::optional<MeshValidationError> check_index(std::uint32_t index, std::size_t vertices,
                                               std::size_t triangle) {
    if (index >= vertices) {
        return error(MeshErrorCode::index_out_of_range, triangle,
                     "triangle " + std::to_string(triangle) + " references vertex " +
                         std::to_string(index) + " of " + std::to_string(vertices));
    }
    return std::nullopt;
}

} // namespace

std::optional<MeshValidationError> validate_mesh(const TriangleMesh& mesh,
                                                 const MeshLimits& limits) {
    if (auto e = check_counts(mesh.vertices.size(), mesh.triangles.size(), limits)) {
        return e;
    }
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
        const Vector3& v = mesh.vertices[i];
        for (double c : {v.x, v.y, v.z}) {
            if (auto e = check_coordinate(c, i, limits)) {
                return e;
            }
        }
    }
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        for (std::uint32_t index : mesh.triangles[t]) {
            if (auto e = check_index(index, mesh.vertices.size(), t)) {
                return e;
            }
        }
    }
    return std::nullopt;
}

core::Result<TriangleMesh, MeshValidationError>
mesh_from_arrays(std::span<const double> xyz, std::span<const std::uint32_t> indices,
                 const MeshLimits& limits) {
    if (xyz.size() % 3 != 0 || indices.size() % 3 != 0) {
        return core::failure(
            error(MeshErrorCode::malformed_array, 0, "array lengths must be multiples of three"));
    }
    const std::size_t vertices = xyz.size() / 3;
    const std::size_t triangles = indices.size() / 3;
    if (auto e = check_counts(vertices, triangles, limits)) {
        return core::failure(std::move(*e));
    }
    for (std::size_t i = 0; i < xyz.size(); ++i) {
        if (auto e = check_coordinate(xyz[i], i / 3, limits)) {
            return core::failure(std::move(*e));
        }
    }
    for (std::size_t i = 0; i < indices.size(); ++i) {
        if (auto e = check_index(indices[i], vertices, i / 3)) {
            return core::failure(std::move(*e));
        }
    }
    TriangleMesh mesh;
    mesh.vertices.reserve(vertices);
    for (std::size_t i = 0; i < vertices; ++i) {
        mesh.vertices.push_back({xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]});
    }
    mesh.triangles.reserve(triangles);
    for (std::size_t t = 0; t < triangles; ++t) {
        mesh.triangles.push_back({indices[3 * t], indices[3 * t + 1], indices[3 * t + 2]});
    }
    return mesh;
}

} // namespace orrery::geom
