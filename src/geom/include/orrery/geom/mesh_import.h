#pragma once

#include <cstddef>
#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "orrery/core/result.h"
#include "orrery/geom/mesh_validation.h"
#include "orrery/geom/triangle_mesh.h"

namespace orrery::geom {

// Model import through assimp (docs/geometry-import.md). Every file is treated
// as untrusted: only the post-processing steps Triangulate,
// JoinIdenticalVertices and GenNormals run, and the scene is validated
// (counts, NaN/inf, coordinate range, index range) while it is converted,
// before any other code sees the mesh.

enum class ModelFormat { obj, stl, gltf, glb, ply, fbx };

// Lower-case name ("obj", "stl", "gltf", "glb", "ply", "fbx").
[[nodiscard]] std::string_view format_name(ModelFormat format) noexcept;

enum class UpAxis { y_up, z_up };

struct ImportLimits {
    MeshLimits mesh;
    // 200 MB, as for downloaded models (ADR-010).
    std::size_t max_file_bytes = 200ULL * 1024ULL * 1024ULL;
    std::size_t max_nodes = 100'000;
    std::size_t max_node_depth = 128;
};

// Suggested scale from file units to metres. It is a suggestion: the caller
// confirms it (or chooses another) and applies it with apply_unit_scale.
struct UnitEstimate {
    double scale_to_metres = 1.0;
    std::string unit = "m";
    // True when the format defines the unit (glTF: metres; FBX: its
    // UnitScaleFactor), false for the bounding-box heuristic.
    bool from_format = false;
    // Largest bounding-box extent in file units.
    double raw_extent = 0.0;
};

struct UpAxisEstimate {
    UpAxis axis = UpAxis::z_up;
    // True when the format defines it (glTF: +Y; FBX: its UpAxis setting),
    // false for the per-format convention.
    bool from_format = false;
};

struct ImportedMesh {
    // Raw file units and axes; all node transforms applied.
    TriangleMesh mesh;
    ModelFormat format = ModelFormat::obj;
    UnitEstimate unit;
    UpAxisEstimate up;
    std::size_t source_meshes = 0;
    // Point and line primitives dropped (they enclose nothing).
    std::size_t skipped_primitives = 0;
};

enum class ImportErrorCode {
    unsupported_format,
    file_not_found,
    file_too_large,
    parse_failed,
    too_many_nodes,
    invalid_mesh,
};

struct ImportError {
    ImportErrorCode code = ImportErrorCode::parse_failed;
    std::string message;
    // Set for invalid_mesh.
    MeshValidationError mesh_error;
};

// Format from a file name extension (case-insensitive).
[[nodiscard]] core::Result<ModelFormat, ImportError> format_from_extension(std::string_view ext);

// Imports a local file. Files referenced by the model (glTF buffers, OBJ
// materials) are opened only from the model's own directory.
[[nodiscard]] core::Result<ImportedMesh, ImportError>
import_mesh_file(const std::filesystem::path& path, const ImportLimits& limits = {});

// Imports a model held in memory. External references are refused: only
// self-contained data (glb, embedded glTF buffers, OBJ without materials)
// loads.
[[nodiscard]] core::Result<ImportedMesh, ImportError>
import_mesh_memory(std::span<const std::byte> data, ModelFormat format,
                   const ImportLimits& limits = {});

// Bounding-box unit heuristic (largest extent e in file units): e > 500 is
// read as millimetres, 50 < e <= 500 as centimetres, otherwise metres.
// APPROXIMATION (G3): model size prior - aerodynamic test bodies are 5 cm to
// 500 m; a wrong guess is caught by the caller's confirmation.
[[nodiscard]] UnitEstimate estimate_unit(const TriangleMesh& mesh);

// Multiplies every coordinate by `scale` (positive, finite).
void apply_unit_scale(TriangleMesh& mesh, double scale);

// Rotates a mesh from `source` into the project frame (+Z up, right-handed):
// Y-up (x, y, z) becomes (x, -z, y), a +90 degree rotation about x.
void convert_to_z_up(TriangleMesh& mesh, UpAxis source) noexcept;

} // namespace orrery::geom
