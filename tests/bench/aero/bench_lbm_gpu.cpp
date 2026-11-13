// GPU lattice Boltzmann benchmarks (docs/performance.md, spec section 8):
// D3Q19 128^3 >= 60 steps/s on a GTX 1660-class GPU, D2Q9 2048 x 1024
// >= 200 steps/s. The OpenGL 4.6 context lives on a hidden window that is
// never shown. Without a context the measurements are recorded as skipped
// (ORRERY_REQUIRE_GL=1 makes that a failure).
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <optional>
#include <string>
#include <utility>

#include "../../aero/lbm/lbm_gl_support.h"
#include "../../aero/lbm/lbm_test_support.h"
#include "../../aero/lbm3d/lbm3d_test_support.h"
#include "bench_record.h"

using namespace orrery;
using aero::lbm::test::GlCompute;
using bench::Goal;
using bench::Measurement;

namespace {

// The compute backend on a hidden-window context, or nothing (recorded as
// skipped for `ids`) when no OpenGL 4.6 context can be created.
std::optional<GlCompute>
gl_compute(std::initializer_list<std::pair<const char*, const char*>> ids) {
    auto context = render::GlContext::create(render::OffscreenRenderer::context_options());
    if (!context) {
        if (render::test::environment_flag("ORRERY_REQUIRE_GL")) {
            FAIL("OpenGL 4.6 required (ORRERY_REQUIRE_GL=1) but unavailable: " << context.error());
        }
        for (const auto& [id, title] : ids) {
            bench::record_skipped(id, title,
                                  bench::cat({"no OpenGL 4.6 context: ", context.error()}));
        }
        return std::nullopt;
    }
    context.value().make_current();
    auto backend = render::GlComputeBackend::create();
    REQUIRE(backend.has_value());
    std::optional<GlCompute> made;
    made.emplace(GlCompute{std::move(context).value(), std::move(backend).value()});
    bench::record_device(made->backend->info().device);
    return made;
}

// Steps per second over `count` steps after a warm-up that compiles the
// kernels and touches the buffers; the GPU is drained around the timing.
template <typename Solver>
double gpu_steps_per_second(GlCompute& gl, Solver& solver, std::size_t count) {
    aero::lbm::test::require_ok(solver.step(50));
    gl.backend->finish();
    const auto start = std::chrono::steady_clock::now();
    aero::lbm::test::require_ok(solver.step(count));
    gl.backend->finish();
    const double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    return static_cast<double>(count) / seconds;
}

void record_gpu_rate(const GlCompute& gl, const char* id, const char* title, double rate,
                     double cells, Goal goal, double target, const std::string& notes) {
    Measurement m;
    m.id = id;
    m.title = title;
    m.value = rate;
    m.unit = "steps/s";
    m.goal = goal;
    m.target = target;
    m.notes = notes;
    m.context = {{"mlups", bench::number(rate * cells / 1e6)},
                 {"device", gl.backend->info().device}};
    bench::record(m);
}

} // namespace

TEST_CASE("GPU D3Q19 throughput at 128^3", "[bench][aero][lbm3d][gl]") {
    auto gl = gl_compute({{"aero.lbm3d.gpu.128", "LBM D3Q19 128^3 (GPU)"},
                          {"aero.lbm3d.gpu.128.les", "LBM D3Q19 128^3 (GPU, Smagorinsky)"}});
    if (!gl) {
        return;
    }
    for (const bool les : {false, true}) {
        auto setup = aero::lbm::test::sphere_channel(128, 128, 128, 24.0, 2.5, 0.05, 300.0);
        setup.config.smagorinsky = aero::lbm::Smagorinsky{les, 0.1};
        auto solver = aero::lbm::test::take(
            aero::lbm::make_gpu_solver(*gl->backend, setup.config, setup.mask));
        const double rate = gpu_steps_per_second(*gl, *solver, 1000);
        CHECK(std::isfinite(aero::lbm::test::take(solver->mass())));
        record_gpu_rate(*gl, les ? "aero.lbm3d.gpu.128.les" : "aero.lbm3d.gpu.128",
                        les ? "LBM D3Q19 128^3 (GPU, Smagorinsky)" : "LBM D3Q19 128^3 (GPU)", rate,
                        128.0 * 128.0 * 128.0, les ? Goal::info : Goal::at_least, 60.0,
                        "sphere channel D 24, Re 300, binary32 populations; target set for a "
                        "GTX 1660-class GPU");
    }
    CHECK(gl->context.debug_error_count() == 0);
}

TEST_CASE("GPU D2Q9 throughput at 2048 x 1024", "[bench][aero][lbm][gl]") {
    auto gl =
        gl_compute({{"aero.lbm2d.gpu.2048x1024", "LBM D2Q9 2048 x 1024 (GPU)"},
                    {"aero.lbm2d.gpu.2048x1024.les", "LBM D2Q9 2048 x 1024 (GPU, Smagorinsky)"}});
    if (!gl) {
        return;
    }
    for (const bool les : {false, true}) {
        auto setup = aero::lbm::test::cylinder_setup(2048, 1024, 64.0, 400.0, 512.5, 0.05, 1000.0);
        setup.config.smagorinsky = aero::lbm::Smagorinsky{les, 0.1};
        auto solver = aero::lbm::test::take(
            aero::lbm::make_gpu_solver(*gl->backend, setup.config, setup.mask));
        const double rate = gpu_steps_per_second(*gl, *solver, 1000);
        CHECK(std::isfinite(aero::lbm::test::take(solver->force()).x));
        record_gpu_rate(*gl, les ? "aero.lbm2d.gpu.2048x1024.les" : "aero.lbm2d.gpu.2048x1024",
                        les ? "LBM D2Q9 2048 x 1024 (GPU, Smagorinsky)"
                            : "LBM D2Q9 2048 x 1024 (GPU)",
                        rate, 2048.0 * 1024.0, les ? Goal::info : Goal::at_least, 200.0,
                        "cylinder D 64, Re 1000, binary32 populations");
    }
    CHECK(gl->context.debug_error_count() == 0);
}
