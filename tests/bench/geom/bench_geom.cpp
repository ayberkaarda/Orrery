// Geometry benchmark (docs/performance.md, spec section 8): a local glTF
// model of about 500k triangles imported with its watertightness report in
// under 5 s. Measured twice: import plus the report (analyze_mesh, what the
// import panel shows), and import plus the full repair pass (repair_mesh,
// which reports before and after).
#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <filesystem>
#include <numbers>
#include <string>

#include "../../geom/geom_test_support.h"
#include "bench_record.h"
#include "orrery/geom/mesh_import.h"
#include "orrery/geom/mesh_repair.h"
#include "orrery/geom/primitives.h"

using namespace orrery;
using bench::Goal;
using bench::Measurement;

namespace {

std::filesystem::path model_file() {
    const std::filesystem::path dir = geom::test::output_dir("bench_import");
    const geom::TriangleMesh sphere =
        geom::sphere_profile(1.0).mesh({720, std::numbers::pi / 354.0});
    const std::filesystem::path file = dir / "sphere_500k.glb";
    geom::test::write_bytes(file, geom::test::glb(sphere));
    return file;
}

void record_import(const char* id, const char* title, double ms, std::size_t triangles,
                   const std::filesystem::path& file, const char* notes) {
    Measurement m;
    m.id = id;
    m.title = title;
    m.value = ms / 1000.0;
    m.unit = "s";
    m.goal = Goal::below;
    m.target = 5.0;
    m.notes = notes;
    m.context = {{"triangles", std::to_string(triangles)},
                 {"file_bytes", std::to_string(std::filesystem::file_size(file))}};
    bench::record(m);
}

} // namespace

TEST_CASE("500k-triangle glb import with the watertightness report", "[bench][geom][import]") {
    const std::filesystem::path file = model_file();
    std::size_t triangles = 0;
    bool watertight = false;
    const auto samples = bench::time_ms(
        [&] {
            const auto imported = geom::import_mesh_file(file);
            REQUIRE(imported.has_value());
            const geom::WatertightReport report = geom::analyze_mesh(imported.value().mesh);
            triangles = report.triangles;
            watertight = report.watertight();
        },
        1, 3);
    CHECK(watertight);
    CHECK(triangles >= 500000);
    record_import("geom.import.glb500k", "Model import, 500k triangles glb (with report)",
                  bench::median(samples), triangles, file,
                  "assimp import, validation and analyze_mesh; file in the OS cache");
}

TEST_CASE("500k-triangle glb import with the full repair pass", "[bench][geom][import]") {
    const std::filesystem::path file = model_file();
    std::size_t triangles = 0;
    bool watertight = false;
    const auto samples = bench::time_ms(
        [&] {
            const auto imported = geom::import_mesh_file(file);
            REQUIRE(imported.has_value());
            const geom::RepairResult repaired = geom::repair_mesh(imported.value().mesh);
            triangles = repaired.after.triangles;
            watertight = repaired.after.watertight();
        },
        1, 3);
    CHECK(watertight);
    record_import("geom.import_repair.glb500k", "Model import, 500k triangles glb (with repair)",
                  bench::median(samples), triangles, file,
                  "import followed by repair_mesh (weld, orientation, holes, reports before and "
                  "after)");
}
