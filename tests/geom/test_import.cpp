#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <string>
#include <vector>

#include "geom_test_support.h"
#include "orrery/geom/mesh_import.h"
#include "orrery/geom/mesh_repair.h"
#include "orrery/geom/mesh_validation.h"
#include "orrery/geom/primitives.h"

using namespace orrery::geom;
namespace t = orrery::geom::test;

namespace {

// A 1 x 2 x 3 box: every coordinate is exact in single precision, so each
// format must reproduce volume 6 and area 22 exactly.
TriangleMesh test_box() {
    TriangleMesh m = box_mesh(1.0, 2.0, 3.0);
    for (Vector3& v : m.vertices) {
        v += Vector3{0.5, 1.0, 1.5};
    }
    return m;
}

void check_box(const ImportedMesh& imported) {
    const WatertightReport r = analyze_mesh(imported.mesh);
    CHECK(r.watertight());
    CHECK(r.outward());
    CHECK(r.triangles == 12);
    CHECK(t::relative_error(signed_volume(imported.mesh), 6.0) < 1e-12);
    CHECK(t::relative_error(surface_area(imported.mesh), 22.0) < 1e-12);
}

ImportError expect_error(const orrery::core::Result<ImportedMesh, ImportError>& result) {
    REQUIRE_FALSE(result.has_value());
    return result.error();
}

} // namespace

TEST_CASE("every supported format imports the same box", "[geom][import]") {
    const auto dir = t::output_dir("import_formats");
    const TriangleMesh box = test_box();
    t::write_text(dir / "box.obj", t::obj_text(box));
    t::write_text(dir / "box_ascii.stl", t::ascii_stl(box));
    t::write_bytes(dir / "box_binary.stl", t::binary_stl(box));
    t::write_text(dir / "box.ply", t::ply_text(box));
    t::write_text(dir / "box.gltf", t::gltf_embedded(box));
    t::write_bytes(dir / "box.glb", t::glb(box));

    for (const char* name :
         {"box.obj", "box_ascii.stl", "box_binary.stl", "box.ply", "box.gltf", "box.glb"}) {
        INFO(name);
        const auto result = import_mesh_file(dir / name);
        if (!result) {
            FAIL(result.error().message);
        }
        check_box(result.value());
        CHECK(result.value().unit.raw_extent == 3.0);
        CHECK(result.value().unit.scale_to_metres == 1.0);
    }
    // glTF defines metres and +Y up; STL has no unit and is read as +Z up.
    const ImportedMesh gltf = import_mesh_file(dir / "box.gltf").value();
    CHECK(gltf.format == ModelFormat::gltf);
    CHECK(gltf.unit.from_format);
    CHECK(gltf.up.axis == UpAxis::y_up);
    CHECK(gltf.up.from_format);
    const ImportedMesh stl = import_mesh_file(dir / "box_binary.stl").value();
    CHECK_FALSE(stl.unit.from_format);
    CHECK(stl.up.axis == UpAxis::z_up);
    CHECK_FALSE(stl.up.from_format);

    // The same bytes import from memory.
    const auto memory = import_mesh_memory(t::read_bytes(dir / "box.glb"), ModelFormat::glb);
    REQUIRE(memory.has_value());
    check_box(memory.value());
}

TEST_CASE("FBX unit and up axis come from the file", "[geom][import]") {
    const auto result = import_mesh_file(t::fixture_dir() / "cube_zup_metres.fbx");
    if (!result) {
        FAIL(result.error().message);
    }
    const ImportedMesh& m = result.value();
    CHECK(m.format == ModelFormat::fbx);
    CHECK(m.unit.from_format);
    CHECK(m.unit.scale_to_metres == 1.0);
    CHECK(m.up.from_format);
    CHECK(m.up.axis == UpAxis::z_up);
    const WatertightReport r = analyze_mesh(m.mesh, 0.0);
    CHECK(r.watertight());
    CHECK(std::abs(std::abs(signed_volume(m.mesh)) - 8.0) < 1e-12);
}

TEST_CASE("unit estimate and axis conversion", "[geom][import]") {
    TriangleMesh mm = box_mesh(1200.0, 300.0, 200.0);
    UnitEstimate u = estimate_unit(mm);
    CHECK(u.unit == "mm");
    CHECK(u.scale_to_metres == 0.001);
    CHECK(u.raw_extent == 1200.0);
    CHECK_FALSE(u.from_format);
    apply_unit_scale(mm, u.scale_to_metres);
    CHECK(std::abs(bounds(mm).extent().x - 1.2) < 1e-12);

    CHECK(estimate_unit(box_mesh(120.0, 10.0, 10.0)).unit == "cm");
    CHECK(estimate_unit(box_mesh(12.0, 1.0, 1.0)).unit == "m");
    CHECK_THROWS(apply_unit_scale(mm, 0.0));

    // Y-up box 1 x 2 x 3 (height 2 along +y) becomes 1 x 3 x 2 with height
    // along +z, still outward.
    TriangleMesh y_up = box_mesh(1.0, 2.0, 3.0);
    convert_to_z_up(y_up, UpAxis::y_up);
    const Vector3 e = bounds(y_up).extent();
    CHECK(e.x == 1.0);
    CHECK(e.y == 3.0);
    CHECK(e.z == 2.0);
    CHECK(signed_volume(y_up) == 6.0);
    TriangleMesh z_up = box_mesh(1.0, 2.0, 3.0);
    convert_to_z_up(z_up, UpAxis::z_up);
    CHECK(bounds(z_up).extent().z == 3.0);
}

TEST_CASE("hostile array data is rejected before use", "[geom][import][hostile]") {
    const std::vector<double> xyz{0, 0, 0, 1, 0, 0, 0, 1, 0};
    const std::vector<std::uint32_t> ok{0, 1, 2};
    CHECK(mesh_from_arrays(xyz, ok, {}).has_value());

    std::vector<double> nan_xyz = xyz;
    nan_xyz[4] = std::numeric_limits<double>::quiet_NaN();
    auto r = mesh_from_arrays(nan_xyz, ok, {});
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().code == MeshErrorCode::non_finite_coordinate);
    CHECK(r.error().element == 1);

    std::vector<double> inf_xyz = xyz;
    inf_xyz[8] = -std::numeric_limits<double>::infinity();
    CHECK(mesh_from_arrays(inf_xyz, ok, {}).error().code == MeshErrorCode::non_finite_coordinate);

    std::vector<double> huge_xyz = xyz;
    huge_xyz[0] = 1e300;
    CHECK(mesh_from_arrays(huge_xyz, ok, {}).error().code ==
          MeshErrorCode::coordinate_out_of_range);

    const std::vector<std::uint32_t> bad{0, 1, 3};
    r = mesh_from_arrays(xyz, bad, {});
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().code == MeshErrorCode::index_out_of_range);

    const std::vector<std::uint32_t> wrapped{0, 1, std::numeric_limits<std::uint32_t>::max()};
    CHECK(mesh_from_arrays(xyz, wrapped, {}).error().code == MeshErrorCode::index_out_of_range);

    CHECK(mesh_from_arrays(std::vector<double>{0, 0}, ok, {}).error().code ==
          MeshErrorCode::malformed_array);
    CHECK(mesh_from_arrays({}, {}, {}).error().code == MeshErrorCode::empty);

    MeshLimits tight;
    tight.max_vertices = 2;
    CHECK(mesh_from_arrays(xyz, ok, tight).error().code == MeshErrorCode::too_many_vertices);
    tight = {};
    tight.max_triangles = 0;
    CHECK(mesh_from_arrays(xyz, ok, tight).error().code == MeshErrorCode::too_many_triangles);

    // The same rules for an assembled mesh.
    TriangleMesh m = test_box();
    m.triangles[5][1] = 8;
    CHECK(validate_mesh(m, {})->code == MeshErrorCode::index_out_of_range);
}

TEST_CASE("hostile model files are rejected", "[geom][import][hostile]") {
    const auto dir = t::output_dir("import_hostile");
    const TriangleMesh box = test_box();

    // NaN coordinate in a binary STL (quiet NaN bit pattern).
    {
        std::vector<std::byte> stl = t::binary_stl(box);
        const std::size_t offset = 80 + 4 + 12; // first vertex x of facet 0
        for (std::size_t i = 0; i < 4; ++i) {
            stl[offset + i] = static_cast<std::byte>((0x7fc00000U >> (8 * i)) & 0xffU);
        }
        t::write_bytes(dir / "nan.stl", stl);
        const ImportError e = expect_error(import_mesh_file(dir / "nan.stl"));
        CHECK(e.code == ImportErrorCode::invalid_mesh);
        CHECK(e.mesh_error.code == MeshErrorCode::non_finite_coordinate);
    }
    // Infinity in an ASCII PLY.
    {
        std::string ply = t::ply_text(box);
        const std::size_t at = ply.find("end_header\n") + 11;
        ply.replace(at, ply.find('\n', at) - at, "inf 0 0");
        t::write_text(dir / "inf.ply", ply);
        CHECK_FALSE(import_mesh_file(dir / "inf.ply").has_value());
    }
    // Face index past the vertex array.
    {
        std::string ply = t::ply_text(box);
        ply.replace(ply.rfind("3 "), std::string::npos, "3 0 1 99\n");
        t::write_text(dir / "bad_index.ply", ply);
        CHECK_FALSE(import_mesh_file(dir / "bad_index.ply").has_value());
        std::string obj = t::obj_text(box) + "f 1 2 99\n";
        t::write_text(dir / "bad_index.obj", obj);
        CHECK_FALSE(import_mesh_file(dir / "bad_index.obj").has_value());
    }
    // Header claiming 4 294 967 295 facets in a 134-byte file.
    {
        std::vector<std::byte> stl(80, std::byte{0});
        t::put_u32(stl, 0xffffffffU);
        stl.resize(134, std::byte{0});
        t::write_bytes(dir / "huge_count.stl", stl);
        CHECK(expect_error(import_mesh_file(dir / "huge_count.stl")).code ==
              ImportErrorCode::parse_failed);
    }
    // PLY header claiming four billion vertices.
    {
        t::write_text(dir / "huge_count.ply",
                      "ply\nformat ascii 1.0\nelement vertex 4000000000\nproperty float x\n"
                      "property float y\nproperty float z\nelement face 1\n"
                      "property list uchar int vertex_indices\nend_header\n0 0 0\n");
        CHECK_FALSE(import_mesh_file(dir / "huge_count.ply").has_value());
    }
    // Binary PLY declaring 100000 vertices with 12 bytes of data.
    {
        t::write_text(dir / "short_binary.ply",
                      "ply\nformat binary_little_endian 1.0\nelement vertex 100000\n"
                      "property float x\nproperty float y\nproperty float z\nelement face 1\n"
                      "property list uchar int vertex_indices\nend_header\n" +
                          std::string(12, '\x01'));
        const ImportError e = expect_error(import_mesh_file(dir / "short_binary.ply"));
        CHECK(e.code == ImportErrorCode::parse_failed);
        CHECK(e.message.find("more entries than the file holds") != std::string::npos);
    }
    // ASCII FBX array declaring four billion values.
    {
        t::write_text(dir / "huge_array.fbx",
                      "; FBX 7.4.0 project file\nObjects:  {\n\tGeometry: 1, \"Geometry::g\", "
                      "\"Mesh\" {\n\t\tVertices: *4000000000 {\n\t\t\ta: 0,0,0\n\t\t}\n\t}\n}\n");
        const ImportError e = expect_error(import_mesh_file(dir / "huge_array.fbx"));
        CHECK(e.message.find("FBX array") != std::string::npos);
    }
    // Binary FBX whose deflated array claims 16 GB from 2 bytes of data.
    {
        std::vector<std::byte> fbx = t::bytes_of(std::string("Kaydara FBX Binary  ", 20));
        fbx.push_back(std::byte{0});
        fbx.push_back(std::byte{0x1a});
        fbx.push_back(std::byte{0});
        t::put_u32(fbx, 7400);
        const std::string name = "Vertices";
        const std::uint32_t property_bytes = 1 + 12 + 2;
        const auto end = static_cast<std::uint32_t>(fbx.size() + 13 + name.size() + property_bytes);
        t::put_u32(fbx, end);
        t::put_u32(fbx, 1);
        t::put_u32(fbx, property_bytes);
        fbx.push_back(static_cast<std::byte>(name.size()));
        for (char c : name) {
            fbx.push_back(static_cast<std::byte>(c));
        }
        fbx.push_back(static_cast<std::byte>('d'));
        t::put_u32(fbx, 0x7fffffffU); // array length
        t::put_u32(fbx, 1);           // deflate
        t::put_u32(fbx, 2);           // stored bytes
        fbx.push_back(std::byte{0x78});
        fbx.push_back(std::byte{0x01});
        fbx.resize(fbx.size() + 13 + 160, std::byte{0}); // null record and footer
        t::write_bytes(dir / "bomb.fbx", fbx);
        const ImportError e = expect_error(import_mesh_file(dir / "bomb.fbx"));
        CHECK(e.code == ImportErrorCode::parse_failed);
        CHECK(e.message.find("FBX") != std::string::npos);
    }
    // Counts above the configured limits.
    {
        t::write_bytes(dir / "box.stl", t::binary_stl(box));
        ImportLimits limits;
        limits.mesh.max_triangles = 11;
        const ImportError e = expect_error(import_mesh_file(dir / "box.stl", limits));
        CHECK(e.code == ImportErrorCode::invalid_mesh);
        CHECK(e.mesh_error.code == MeshErrorCode::too_many_triangles);
        limits = {};
        limits.mesh.max_vertices = 10;
        CHECK(expect_error(import_mesh_file(dir / "box.stl", limits)).mesh_error.code ==
              MeshErrorCode::too_many_vertices);
        limits = {};
        limits.max_file_bytes = 100;
        CHECK(expect_error(import_mesh_file(dir / "box.stl", limits)).code ==
              ImportErrorCode::file_too_large);
    }
    // Unknown or missing files.
    CHECK(expect_error(import_mesh_file(dir / "model.exe")).code ==
          ImportErrorCode::unsupported_format);
    CHECK(expect_error(import_mesh_file(dir / "missing.stl")).code ==
          ImportErrorCode::file_not_found);
    // Garbage with a valid extension.
    t::write_text(dir / "garbage.glb", "this is not a model");
    CHECK_FALSE(import_mesh_file(dir / "garbage.glb").has_value());
    CHECK(expect_error(import_mesh_memory({}, ModelFormat::stl)).code ==
          ImportErrorCode::parse_failed);
}

TEST_CASE("external references stay inside the model directory", "[geom][import][hostile]") {
    const auto dir = t::output_dir("import_references");
    const TriangleMesh box = test_box();
    std::filesystem::create_directories(dir / "model");
    t::write_bytes(dir / "outside.bin", t::gltf_buffer(box));
    t::write_bytes(dir / "model" / "inside.bin", t::gltf_buffer(box));

    // A buffer next to the model loads.
    t::write_text(dir / "model" / "inside.gltf", t::gltf_json(box, "inside.bin"));
    const auto inside = import_mesh_file(dir / "model" / "inside.gltf");
    REQUIRE(inside.has_value());
    check_box(inside.value());

    // A buffer outside the model's directory is refused.
    t::write_text(dir / "model" / "escape.gltf", t::gltf_json(box, "../outside.bin"));
    CHECK_FALSE(import_mesh_file(dir / "model" / "escape.gltf").has_value());

    // From memory, no file at all may be opened.
    const auto memory =
        import_mesh_memory(t::bytes_of(t::gltf_json(box, "inside.bin")), ModelFormat::gltf);
    CHECK_FALSE(memory.has_value());
}

TEST_CASE("format names and extensions", "[geom][import]") {
    CHECK(format_from_extension(".STL").value() == ModelFormat::stl);
    CHECK(format_from_extension("glb").value() == ModelFormat::glb);
    CHECK_FALSE(format_from_extension(".step").has_value());
    CHECK(format_name(ModelFormat::fbx) == "fbx");
}

// Project criterion: a 500k-triangle glTF imports, with its repair report, in
// under 5 s. Hidden (timing depends on the machine): run with
// orrery_test_geom "[benchmark]".
TEST_CASE("500k-triangle glb import with report", "[.benchmark][geom][import]") {
    const auto dir = t::output_dir("import_benchmark");
    const TriangleMesh sphere = sphere_profile(1.0).mesh({720, std::numbers::pi / 354.0});
    t::write_bytes(dir / "sphere.glb", t::glb(sphere));
    const auto start = std::chrono::steady_clock::now();
    const auto imported = import_mesh_file(dir / "sphere.glb");
    REQUIRE(imported.has_value());
    const WatertightReport r = analyze_mesh(imported.value().mesh);
    const double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    INFO(r.triangles << " triangles in " << seconds << " s");
    CHECK(r.watertight());
    CHECK(r.triangles >= 500000);
    CHECK(seconds < 5.0);
    WARN(r.triangles << " triangles imported and analysed in " << seconds << " s");
}
