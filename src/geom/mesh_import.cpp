#include "orrery/geom/mesh_import.h"

#include <assimp/DefaultIOSystem.h>
#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>
#include <assimp/Importer.hpp>
#include <assimp/metadata.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include "model_prescan.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <exception>
#include <fstream>
#include <ios>
#include <limits>
#include <memory>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace orrery::geom {

namespace {

// The only post-processing applied to untrusted files (spec 4.3, ADR-010).
constexpr unsigned int import_steps =
    aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_GenNormals;

ImportError import_error(ImportErrorCode code, std::string message) {
    ImportError e;
    e.code = code;
    e.message = std::move(message);
    return e;
}

std::filesystem::path path_from_utf8(const char* text) {
    std::u8string u8;
    for (const char* c = text; *c != '\0'; ++c) {
        u8.push_back(static_cast<char8_t>(*c));
    }
    return std::filesystem::path(u8);
}

std::string utf8_from_path(const std::filesystem::path& path) {
    std::string out;
    for (char8_t c : path.u8string()) {
        out.push_back(static_cast<char>(c));
    }
    return out;
}

bool read_only_mode(const char* mode) {
    const std::string_view m(mode == nullptr ? "rb" : mode);
    return m.find_first_of("wa+") == std::string_view::npos;
}

// Opens files only inside one directory (the model's own), read-only.
class DirectoryIOSystem final : public Assimp::IOSystem {
public:
    explicit DirectoryIOSystem(std::filesystem::path root) : root_(std::move(root)) {}

    using Assimp::IOSystem::Exists;
    using Assimp::IOSystem::Open;

    bool Exists(const char* file) const override { return allowed(file) && inner_.Exists(file); }
    char getOsSeparator() const override { return inner_.getOsSeparator(); }
    Assimp::IOStream* Open(const char* file, const char* mode) override {
        if (!read_only_mode(mode) || !allowed(file)) {
            return nullptr;
        }
        return inner_.Open(file, mode);
    }
    void Close(Assimp::IOStream* stream) override { inner_.Close(stream); }

private:
    bool allowed(const char* file) const {
        if (file == nullptr) {
            return false;
        }
        std::error_code ec;
        const std::filesystem::path p = std::filesystem::weakly_canonical(
            std::filesystem::absolute(path_from_utf8(file), ec), ec);
        if (ec) {
            return false;
        }
        const std::filesystem::path rel = p.lexically_relative(root_);
        return !rel.empty() && *rel.begin() != ".." && rel.is_relative();
    }

    std::filesystem::path root_;
    mutable Assimp::DefaultIOSystem inner_;
};

// Refuses every file: memory imports may not reach the file system.
class DenyIOSystem final : public Assimp::IOSystem {
public:
    using Assimp::IOSystem::Exists;
    using Assimp::IOSystem::Open;

    bool Exists(const char* /*file*/) const override { return false; }
    char getOsSeparator() const override { return '/'; }
    Assimp::IOStream* Open(const char* /*file*/, const char* /*mode*/) override { return nullptr; }
    void Close(Assimp::IOStream* /*stream*/) override {}
};

// assimp stores single-precision values (ai_real); widen on read.
double widen(ai_real value) {
    return static_cast<double>(value);
}

using Matrix = std::array<double, 16>; // row-major 4 x 4 affine transform

Matrix to_matrix(const aiMatrix4x4& m) {
    return {widen(m.a1), widen(m.a2), widen(m.a3), widen(m.a4), widen(m.b1), widen(m.b2),
            widen(m.b3), widen(m.b4), widen(m.c1), widen(m.c2), widen(m.c3), widen(m.c4),
            widen(m.d1), widen(m.d2), widen(m.d3), widen(m.d4)};
}

Matrix multiply(const Matrix& a, const Matrix& b) {
    Matrix r{};
    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
            double sum = 0.0;
            for (std::size_t k = 0; k < 4; ++k) {
                sum += a[4 * i + k] * b[4 * k + j];
            }
            r[4 * i + j] = sum;
        }
    }
    return r;
}

constexpr Matrix identity{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                          0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};

bool finite(const Matrix& m) {
    return std::all_of(m.begin(), m.end(), [](double v) { return std::isfinite(v); });
}

// Validating conversion of one assimp mesh instance (transform applied).
std::optional<ImportError> append_mesh(TriangleMesh& out, const aiMesh& mesh, const Matrix& t,
                                       const ImportLimits& limits, std::size_t& skipped) {
    const std::size_t base = out.vertices.size();
    // Indices are 32-bit: never accept more vertices than they can address.
    const std::size_t max_vertices =
        std::min<std::size_t>(limits.mesh.max_vertices, std::numeric_limits<std::uint32_t>::max());
    if (mesh.mNumVertices > max_vertices - std::min(base, max_vertices)) {
        ImportError e = import_error(ImportErrorCode::invalid_mesh, "too many vertices");
        e.mesh_error = {MeshErrorCode::too_many_vertices, base + mesh.mNumVertices,
                        "vertex count exceeds the limit " + std::to_string(max_vertices)};
        return e;
    }
    if (mesh.mNumFaces >
        limits.mesh.max_triangles - std::min(out.triangles.size(), limits.mesh.max_triangles)) {
        ImportError e = import_error(ImportErrorCode::invalid_mesh, "too many triangles");
        e.mesh_error = {MeshErrorCode::too_many_triangles, out.triangles.size() + mesh.mNumFaces,
                        "triangle count exceeds the limit " +
                            std::to_string(limits.mesh.max_triangles)};
        return e;
    }
    if (mesh.mNumVertices > 0 && mesh.mVertices == nullptr) {
        return import_error(ImportErrorCode::parse_failed, "mesh without vertex array");
    }
    for (unsigned int i = 0; i < mesh.mNumVertices; ++i) {
        const aiVector3D& v = mesh.mVertices[i];
        const double x = widen(v.x);
        const double y = widen(v.y);
        const double z = widen(v.z);
        const Vector3 p{t[0] * x + t[1] * y + t[2] * z + t[3],
                        t[4] * x + t[5] * y + t[6] * z + t[7],
                        t[8] * x + t[9] * y + t[10] * z + t[11]};
        const bool raw_finite = std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
        if (!raw_finite || !isfinite(p)) {
            ImportError e = import_error(ImportErrorCode::invalid_mesh, "non-finite vertex");
            e.mesh_error = {MeshErrorCode::non_finite_coordinate, base + i,
                            "vertex " + std::to_string(base + i) +
                                " has a NaN or infinite coordinate"};
            return e;
        }
        out.vertices.push_back(p);
    }
    if (mesh.mNumFaces > 0 && mesh.mFaces == nullptr) {
        return import_error(ImportErrorCode::parse_failed, "mesh without face array");
    }
    for (unsigned int f = 0; f < mesh.mNumFaces; ++f) {
        const aiFace& face = mesh.mFaces[f];
        if (face.mNumIndices != 3) {
            ++skipped;
            continue;
        }
        if (face.mIndices == nullptr) {
            return import_error(ImportErrorCode::parse_failed, "face without index array");
        }
        Triangle tri{};
        for (std::size_t k = 0; k < 3; ++k) {
            const unsigned int index = face.mIndices[k];
            if (index >= mesh.mNumVertices) {
                ImportError e = import_error(ImportErrorCode::invalid_mesh, "index out of range");
                e.mesh_error = {MeshErrorCode::index_out_of_range, out.triangles.size(),
                                "face " + std::to_string(f) + " references vertex " +
                                    std::to_string(index) + " of " +
                                    std::to_string(mesh.mNumVertices)};
                return e;
            }
            tri[k] = static_cast<std::uint32_t>(base + index);
        }
        out.triangles.push_back(tri);
    }
    return std::nullopt;
}

template <typename T> bool metadata(const aiScene& scene, const char* key, T& value) {
    return scene.mMetaData != nullptr && scene.mMetaData->Get(key, value);
}

// FBX stores its unit (UnitScaleFactor, centimetres per file unit) and its up
// axis (UpAxis 0/1/2 = X/Y/Z) as scene metadata.
void fbx_conventions(const aiScene& scene, ImportedMesh& out) {
    double scale = 0.0;
    bool found = metadata(scene, "UnitScaleFactor", scale);
    if (!found) {
        ai_real stored{};
        found = metadata(scene, "UnitScaleFactor", stored);
        scale = widen(stored);
    }
    if (found && std::isfinite(scale) && scale > 0.0) {
        out.unit.scale_to_metres = scale / 100.0;
        out.unit.unit = scale == 1.0 ? "cm" : "fbx units";
        out.unit.from_format = true;
    }
    std::int32_t up = -1;
    if (metadata(scene, "UpAxis", up) && (up == 1 || up == 2)) {
        out.up = {up == 1 ? UpAxis::y_up : UpAxis::z_up, true};
    }
}

core::Result<ImportedMesh, ImportError> convert_scene(const aiScene* scene, ModelFormat format,
                                                      const ImportLimits& limits,
                                                      const std::string& error_text) {
    if (scene == nullptr) {
        return core::failure(
            import_error(ImportErrorCode::parse_failed, "parser rejected the file: " + error_text));
    }
    if (scene->mRootNode == nullptr) {
        return core::failure(import_error(ImportErrorCode::parse_failed, "scene has no root"));
    }
    ImportedMesh out;
    out.format = format;

    struct Item {
        const aiNode* node;
        Matrix world;
        std::size_t depth;
    };
    std::vector<Item> stack{{scene->mRootNode, identity, 0}};
    std::size_t visited = 0;
    while (!stack.empty()) {
        const Item item = stack.back();
        stack.pop_back();
        if (++visited > limits.max_nodes || item.depth > limits.max_node_depth) {
            return core::failure(
                import_error(ImportErrorCode::too_many_nodes, "node hierarchy exceeds the limits"));
        }
        const Matrix world = multiply(item.world, to_matrix(item.node->mTransformation));
        if (!finite(world)) {
            ImportError e = import_error(ImportErrorCode::invalid_mesh, "non-finite transform");
            e.mesh_error = {MeshErrorCode::non_finite_coordinate, 0,
                            "node transform has a NaN or infinite entry"};
            return core::failure(std::move(e));
        }
        for (unsigned int i = 0; i < item.node->mNumMeshes; ++i) {
            const unsigned int index = item.node->mMeshes[i];
            if (index >= scene->mNumMeshes || scene->mMeshes[index] == nullptr) {
                return core::failure(
                    import_error(ImportErrorCode::parse_failed, "node references a missing mesh"));
            }
            if (auto e = append_mesh(out.mesh, *scene->mMeshes[index], world, limits,
                                     out.skipped_primitives)) {
                return core::failure(std::move(*e));
            }
            ++out.source_meshes;
        }
        for (unsigned int c = item.node->mNumChildren; c > 0; --c) {
            const aiNode* child = item.node->mChildren[c - 1];
            if (child != nullptr) {
                stack.push_back({child, world, item.depth + 1});
            }
        }
    }
    if (auto e = validate_mesh(out.mesh, limits.mesh)) {
        ImportError err = import_error(ImportErrorCode::invalid_mesh, e->message);
        err.mesh_error = std::move(*e);
        return core::failure(std::move(err));
    }

    out.unit = estimate_unit(out.mesh);
    switch (format) {
        case ModelFormat::gltf:
        case ModelFormat::glb:
            // glTF 2.0: metres, +Y up (specification section 3.4).
            out.unit.scale_to_metres = 1.0;
            out.unit.unit = "m";
            out.unit.from_format = true;
            out.up = {UpAxis::y_up, true};
            break;
        case ModelFormat::fbx:
            out.up = {UpAxis::y_up, false};
            fbx_conventions(*scene, out);
            break;
        case ModelFormat::obj:
            // Exported by most modelling tools with +Y up.
            out.up = {UpAxis::y_up, false};
            break;
        case ModelFormat::stl:
        case ModelFormat::ply:
            // CAD and scanner convention: +Z up.
            out.up = {UpAxis::z_up, false};
            break;
    }
    return out;
}

std::size_t file_size_or_max(const std::filesystem::path& path, std::error_code& ec) {
    const std::uintmax_t size = std::filesystem::file_size(path, ec);
    if (ec || size > SIZE_MAX) {
        return SIZE_MAX;
    }
    return static_cast<std::size_t>(size);
}

ImportError mesh_failure(MeshErrorCode code, std::size_t element, std::string message) {
    ImportError e = import_error(ImportErrorCode::invalid_mesh, message);
    e.mesh_error = {code, element, std::move(message)};
    return e;
}

// Validation of the parsed scene before any post-processing step reads it:
// node hierarchy, mesh references, counts (with the triangles that
// triangulation will create), coordinates and every face index.
std::optional<ImportError> validate_raw_scene(const aiScene& scene, const ImportLimits& limits) {
    if (scene.mRootNode == nullptr || (scene.mNumMeshes > 0 && scene.mMeshes == nullptr)) {
        return import_error(ImportErrorCode::parse_failed, "scene without root or mesh table");
    }
    std::vector<std::pair<const aiNode*, std::size_t>> stack{{scene.mRootNode, 0}};
    std::size_t visited = 0;
    while (!stack.empty()) {
        const auto [node, depth] = stack.back();
        stack.pop_back();
        if (++visited > limits.max_nodes || depth > limits.max_node_depth) {
            return import_error(ImportErrorCode::too_many_nodes,
                                "node hierarchy exceeds the limits");
        }
        if ((node->mNumMeshes > 0 && node->mMeshes == nullptr) ||
            (node->mNumChildren > 0 && node->mChildren == nullptr)) {
            return import_error(ImportErrorCode::parse_failed, "node without its arrays");
        }
        for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
            if (node->mMeshes[i] >= scene.mNumMeshes) {
                return import_error(ImportErrorCode::parse_failed,
                                    "node references a missing mesh");
            }
        }
        for (unsigned int c = 0; c < node->mNumChildren; ++c) {
            if (node->mChildren[c] == nullptr) {
                return import_error(ImportErrorCode::parse_failed, "node without its child");
            }
            stack.emplace_back(node->mChildren[c], depth + 1);
        }
    }
    std::size_t vertices = 0;
    std::size_t triangles = 0;
    for (unsigned int m = 0; m < scene.mNumMeshes; ++m) {
        const aiMesh* mesh = scene.mMeshes[m];
        if (mesh == nullptr || (mesh->mNumVertices > 0 && mesh->mVertices == nullptr) ||
            (mesh->mNumFaces > 0 && mesh->mFaces == nullptr)) {
            return import_error(ImportErrorCode::parse_failed, "mesh without its arrays");
        }
        vertices += mesh->mNumVertices;
        if (vertices > limits.mesh.max_vertices) {
            return mesh_failure(MeshErrorCode::too_many_vertices, vertices,
                                "vertex count exceeds the limit " +
                                    std::to_string(limits.mesh.max_vertices));
        }
        for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
            const aiVector3D& v = mesh->mVertices[i];
            for (double c : {widen(v.x), widen(v.y), widen(v.z)}) {
                if (!std::isfinite(c)) {
                    return mesh_failure(MeshErrorCode::non_finite_coordinate, i,
                                        "vertex " + std::to_string(i) +
                                            " has a NaN or infinite coordinate");
                }
                if (std::abs(c) > limits.mesh.max_abs_coordinate) {
                    return mesh_failure(MeshErrorCode::coordinate_out_of_range, i,
                                        "vertex " + std::to_string(i) +
                                            " coordinate exceeds the limit");
                }
            }
        }
        for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
            const aiFace& face = mesh->mFaces[f];
            if (face.mNumIndices > 0 && face.mIndices == nullptr) {
                return import_error(ImportErrorCode::parse_failed, "face without its indices");
            }
            for (unsigned int k = 0; k < face.mNumIndices; ++k) {
                if (face.mIndices[k] >= mesh->mNumVertices) {
                    return mesh_failure(MeshErrorCode::index_out_of_range, f,
                                        "face " + std::to_string(f) + " references vertex " +
                                            std::to_string(face.mIndices[k]) + " of " +
                                            std::to_string(mesh->mNumVertices));
                }
            }
            // A polygon of n corners becomes n - 2 triangles.
            triangles += face.mNumIndices >= 3 ? face.mNumIndices - 2 : 0;
            if (triangles > limits.mesh.max_triangles) {
                return mesh_failure(MeshErrorCode::too_many_triangles, triangles,
                                    "triangle count exceeds the limit " +
                                        std::to_string(limits.mesh.max_triangles));
            }
        }
    }
    return std::nullopt;
}

// Parse (no post-processing), validate, then run the restricted steps.
core::Result<ImportedMesh, ImportError> parse_validated(Assimp::Importer& importer,
                                                        const aiScene* parsed, ModelFormat format,
                                                        const ImportLimits& limits) {
    if (parsed == nullptr) {
        return core::failure(
            import_error(ImportErrorCode::parse_failed,
                         std::string("parser rejected the file: ") + importer.GetErrorString()));
    }
    if (auto e = validate_raw_scene(*parsed, limits)) {
        return core::failure(std::move(*e));
    }
    const aiScene* scene = importer.ApplyPostProcessing(import_steps);
    return convert_scene(scene, format, limits, importer.GetErrorString());
}

core::Result<std::vector<std::byte>, ImportError> read_file(const std::filesystem::path& path,
                                                            std::size_t max_bytes) {
    std::error_code ec;
    const std::size_t size = file_size_or_max(path, ec);
    if (size > max_bytes) {
        return core::failure(
            import_error(ImportErrorCode::file_too_large, "model file exceeds the size limit"));
    }
    std::ifstream in(path, std::ios::binary);
    std::vector<char> chars(size);
    if (!in.read(chars.data(), static_cast<std::streamsize>(size))) {
        return core::failure(
            import_error(ImportErrorCode::file_not_found, "model file cannot be read"));
    }
    std::vector<std::byte> bytes(size);
    std::transform(chars.begin(), chars.end(), bytes.begin(),
                   [](char c) { return static_cast<std::byte>(c); });
    return bytes;
}

// Formats whose files may reference others (glTF buffers, OBJ materials) are
// read through the directory-confined file system; the rest from memory.
bool needs_directory(ModelFormat format) {
    return format == ModelFormat::gltf || format == ModelFormat::glb || format == ModelFormat::obj;
}

} // namespace

std::string_view format_name(ModelFormat format) noexcept {
    switch (format) {
        case ModelFormat::obj:
            return "obj";
        case ModelFormat::stl:
            return "stl";
        case ModelFormat::gltf:
            return "gltf";
        case ModelFormat::glb:
            return "glb";
        case ModelFormat::ply:
            return "ply";
        case ModelFormat::fbx:
            return "fbx";
    }
    return "obj";
}

core::Result<ModelFormat, ImportError> format_from_extension(std::string_view ext) {
    std::string lower;
    for (char c : ext) {
        if (c != '.' || !lower.empty()) {
            lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
        }
    }
    for (ModelFormat f : {ModelFormat::obj, ModelFormat::stl, ModelFormat::gltf, ModelFormat::glb,
                          ModelFormat::ply, ModelFormat::fbx}) {
        if (lower == format_name(f)) {
            return f;
        }
    }
    return core::failure(import_error(ImportErrorCode::unsupported_format,
                                      "unsupported model format '" + std::string(ext) + "'"));
}

core::Result<ImportedMesh, ImportError> import_mesh_file(const std::filesystem::path& path,
                                                         const ImportLimits& limits) {
    auto format = format_from_extension(utf8_from_path(path.extension()));
    if (!format) {
        return core::failure(format.error());
    }
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec)) {
        return core::failure(import_error(ImportErrorCode::file_not_found,
                                          "model file not found: " + utf8_from_path(path)));
    }
    const std::filesystem::path absolute = std::filesystem::weakly_canonical(path, ec);
    if (ec) {
        return core::failure(import_error(ImportErrorCode::file_not_found,
                                          "model path cannot be resolved: " + ec.message()));
    }
    try {
        auto bytes = read_file(absolute, limits.max_file_bytes);
        if (!bytes) {
            return core::failure(bytes.error());
        }
        if (!needs_directory(format.value())) {
            return import_mesh_memory(bytes.value(), format.value(), limits);
        }
        if (auto reason = detail::prescan_model(bytes.value(), format.value(), limits)) {
            return core::failure(import_error(ImportErrorCode::parse_failed, *reason));
        }
        Assimp::Importer importer;
        // Ownership passes to the importer (assimp's C++ API takes a raw
        // pointer and deletes it with the importer).
        importer.SetIOHandler(
            std::make_unique<DirectoryIOSystem>(absolute.parent_path()).release());
        const aiScene* parsed = importer.ReadFile(utf8_from_path(absolute), 0U);
        return parse_validated(importer, parsed, format.value(), limits);
    } catch (const std::exception& e) {
        return core::failure(import_error(ImportErrorCode::parse_failed, e.what()));
    }
}

core::Result<ImportedMesh, ImportError> import_mesh_memory(std::span<const std::byte> data,
                                                           ModelFormat format,
                                                           const ImportLimits& limits) {
    if (data.size() > limits.max_file_bytes) {
        return core::failure(
            import_error(ImportErrorCode::file_too_large, "model data exceeds the size limit"));
    }
    if (data.empty()) {
        return core::failure(import_error(ImportErrorCode::parse_failed, "model data is empty"));
    }
    if (auto reason = detail::prescan_model(data, format, limits)) {
        return core::failure(import_error(ImportErrorCode::parse_failed, *reason));
    }
    try {
        Assimp::Importer importer;
        // Ownership passes to the importer, which wraps it for the memory
        // read: any file the model references is refused.
        importer.SetIOHandler(std::make_unique<DenyIOSystem>().release());
        const std::string hint(format_name(format));
        const aiScene* parsed =
            importer.ReadFileFromMemory(data.data(), data.size(), 0U, hint.c_str());
        return parse_validated(importer, parsed, format, limits);
    } catch (const std::exception& e) {
        return core::failure(import_error(ImportErrorCode::parse_failed, e.what()));
    }
}

UnitEstimate estimate_unit(const TriangleMesh& mesh) {
    const Vector3 e = bounds(mesh).extent();
    UnitEstimate u;
    u.raw_extent = std::max({e.x, e.y, e.z});
    if (u.raw_extent > 500.0) {
        u.scale_to_metres = 0.001;
        u.unit = "mm";
    } else if (u.raw_extent > 50.0) {
        u.scale_to_metres = 0.01;
        u.unit = "cm";
    }
    return u;
}

void apply_unit_scale(TriangleMesh& mesh, double scale) {
    if (!std::isfinite(scale) || !(scale > 0.0)) {
        throw std::invalid_argument("unit scale must be positive and finite");
    }
    for (Vector3& v : mesh.vertices) {
        v = v * scale;
    }
}

void convert_to_z_up(TriangleMesh& mesh, UpAxis source) noexcept {
    if (source == UpAxis::z_up) {
        return;
    }
    for (Vector3& v : mesh.vertices) {
        v = {v.x, -v.z, v.y};
    }
}

} // namespace orrery::geom
