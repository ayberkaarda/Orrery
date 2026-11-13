// Aero benchmarks on the CPU (docs/performance.md): the 3D panel method at
// about 2000 panels (spec section 8: < 3 s), the Newtonian solver on 100k
// triangles (< 100 ms), and the CPU lattice Boltzmann solvers on the grids of
// the GPU targets (D2Q9 2048 x 1024, D3Q19 128^3; the targets are GPU
// targets, so the CPU rates are recorded for context).
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <string>
#include <thread>
#include <vector>

#include "../../aero/lbm/lbm_test_support.h"
#include "../../aero/lbm3d/lbm3d_test_support.h"
#include "bench_record.h"
#include "orrery/aero/newtonian.h"
#include "orrery/aero/panel_method.h"
#include "orrery/geom/primitives.h"

using namespace orrery;
using bench::Goal;
using bench::Measurement;

namespace {

constexpr double pi = std::numbers::pi;
constexpr double degree = pi / 180.0;

std::string threads_note() {
    return bench::cat({std::to_string(std::thread::hardware_concurrency()), " hardware threads"});
}

// Steps per second of `count` solver steps after `warmup` untimed ones.
template <typename Solver>
double steps_per_second(Solver& solver, std::size_t warmup, std::size_t count) {
    aero::lbm::test::require_ok(solver.step(warmup));
    const auto start = std::chrono::steady_clock::now();
    aero::lbm::test::require_ok(solver.step(count));
    const double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    return static_cast<double>(count) / seconds;
}

void record_rate(const std::string& id, const std::string& title, double rate, double cells,
                 const std::string& notes) {
    Measurement m;
    m.id = id;
    m.title = title;
    m.value = rate;
    m.unit = "steps/s";
    m.notes = notes;
    m.context = {{"mlups", bench::number(rate * cells / 1e6)}, {"threads", threads_note()}};
    bench::record(m);
}

} // namespace

TEST_CASE("3D panel method solve at about 2000 panels", "[bench][aero][panel]") {
    aero::AeroGeometry g;
    g.mesh = geom::sphere_profile(1.0).mesh({32, 0.098});
    g.reference = {pi, 2.0, {}};
    aero::FlowConditions c;
    c.speed = 50.0;
    c.angle_of_attack = 2.0 * degree;
    std::size_t panels = 0;
    bool finite = true;
    const auto samples = bench::time_ms(
        [&] {
            aero::PanelSolver3D solver;
            solver.setup(g, c);
            solver.step();
            panels = solver.panels().panels.size();
            finite = finite && std::isfinite(solver.result().cd);
        },
        1, 3);
    CHECK(panels >= 2000);
    CHECK(finite);
    Measurement m;
    m.id = "aero.panel3d.n2000";
    m.title = "Panel 3D, N = 2000 (setup and solve)";
    m.value = bench::median(samples) / 1000.0;
    m.unit = "s";
    m.goal = Goal::below;
    m.target = 3.0;
    m.notes = "sphere, dense LU, influence matrix assembly included";
    m.context = {{"panels", std::to_string(panels)}, {"threads", threads_note()}};
    bench::record(m);
}

TEST_CASE("Newtonian solver on 100k triangles", "[bench][aero][newtonian]") {
    aero::AeroGeometry g;
    g.mesh = geom::sphere_profile(1.0).mesh({256, 0.016});
    g.reference = {pi, 2.0, {}};
    aero::FlowConditions c;
    c.temperature = 220.0;
    c.density = 1e-4;
    c.speed = 10.0 * c.speed_of_sound();
    c.angle_of_attack = 5.0 * degree;
    aero::NewtonianSolver solver;
    solver.setup(g, c);
    const auto samples = bench::time_ms([&] { solver.step(); }, 2, 15);
    CHECK(g.mesh.triangles.size() >= 100000);
    CHECK(std::isfinite(solver.result().cd));
    Measurement m;
    m.id = "aero.newtonian.n100k";
    m.title = "Newtonian, 100k triangles";
    m.value = bench::median(samples);
    m.unit = "ms";
    m.goal = Goal::below;
    m.target = 100.0;
    m.notes = "sphere, Mach 10, modified Newtonian";
    m.context = {{"triangles", std::to_string(g.mesh.triangles.size())}};
    bench::record(m);
}

TEST_CASE("CPU D2Q9 throughput at 2048 x 1024", "[bench][aero][lbm][cpu]") {
    if (!bench::optimised_build()) {
        bench::record_skipped("aero.lbm2d.cpu.2048x1024", "LBM D2Q9 2048 x 1024 (CPU)",
                              "unoptimised build");
        return;
    }
    const auto setup =
        aero::lbm::test::cylinder_setup(2048, 1024, 64.0, 400.0, 512.5, 0.05, 1000.0);
    auto solver = aero::lbm::test::take(aero::lbm::make_cpu_solver(setup.config, setup.mask));
    const double rate = steps_per_second(*solver, 5, 40);
    CHECK(std::isfinite(aero::lbm::test::take(solver->force()).x));
    record_rate("aero.lbm2d.cpu.2048x1024", "LBM D2Q9 2048 x 1024 (CPU)", rate, 2048.0 * 1024.0,
                "cylinder D 64, Re 1000, BGK, double precision; the section 8 target is a "
                "GPU target");
}

TEST_CASE("CPU D3Q19 throughput at 128^3", "[bench][aero][lbm3d][cpu]") {
    if (!bench::optimised_build()) {
        bench::record_skipped("aero.lbm3d.cpu.128", "LBM D3Q19 128^3 (CPU)", "unoptimised build");
        return;
    }
    const auto setup = aero::lbm::test::sphere_channel(128, 128, 128, 24.0, 2.5, 0.05, 300.0);
    auto solver = aero::lbm::test::take(aero::lbm::make_cpu_solver(setup.config, setup.mask));
    const double rate = steps_per_second(*solver, 3, 20);
    CHECK(std::isfinite(aero::lbm::test::take(solver->mass())));
    record_rate("aero.lbm3d.cpu.128", "LBM D3Q19 128^3 (CPU)", rate, 128.0 * 128.0 * 128.0,
                "sphere channel D 24, Re 300, BGK, double precision; the section 8 target is a "
                "GPU target");
}
