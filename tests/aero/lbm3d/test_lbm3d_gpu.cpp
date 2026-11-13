// The GPU D3Q19 solver (OpenGL compute kernels) against the CPU reference.
// Tests tagged [gl] need an OpenGL 4.6 context (lbm_gl_support.h).
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <numbers>
#include <regex>
#include <string>
#include <vector>

#include "../lbm/lbm_gl_support.h"
#include "lbm3d_test_support.h"
#include "orrery/aero/lbm3d/d3q19.h"
#include "orrery/aero/lbm3d/lbm3d.h"
#include "orrery/aero/lbm3d/lbm_aero_solver3d.h"
#include "orrery/geom/primitives.h"
#include "orrery/render/embedded_lbm_shaders.h"

using namespace orrery;
using namespace orrery::aero::lbm;
using namespace orrery::aero::lbm::test;

namespace {

// Integers of the GLSL array initialiser `name[19] = int[19](...)`.
std::vector<int> glsl_int_array(std::string_view source, const std::string& name) {
    const std::regex pattern(name + R"(\[19\]\s*=\s*int\[19\]\(([^)]*)\))");
    std::match_results<std::string_view::const_iterator> match;
    std::vector<int> values;
    if (std::regex_search(source.begin(), source.end(), match, pattern)) {
        const std::string list = match[1].str();
        const std::regex number(R"(-?\d+)");
        for (auto it = std::sregex_iterator(list.begin(), list.end(), number);
             it != std::sregex_iterator(); ++it) {
            values.push_back(std::stoi(it->str()));
        }
    }
    return values;
}

struct Comparison {
    double velocity = 0.0;
    double populations = 0.0;
    double force_x = 0.0; // relative to the CPU value
};

Comparison compare(aero::compute::IComputeBackend& backend, const Lbm3DConfig& config,
                   const SolidMask3D& mask, std::size_t steps, const char* label) {
    auto cpu = take(make_cpu_solver(config, mask));
    auto gpu = take(make_gpu_solver(backend, config, mask));
    require_ok(cpu->step(steps));
    require_ok(gpu->step(steps));
    const MacroFields3D c = take(cpu->macroscopic());
    const MacroFields3D g = take(gpu->macroscopic());
    Comparison out;
    out.velocity = velocity_l2(g, c);
    out.populations = l2_relative(take(gpu->populations()), take(cpu->populations()));
    const Force3D fc = take(cpu->force());
    const Force3D fg = take(gpu->force());
    out.force_x = fc.force[0] != 0.0 ? std::abs(fg.force[0] - fc.force[0]) / std::abs(fc.force[0])
                                     : std::abs(fg.force[0]);
    std::cout << "measured: GPU vs CPU " << label << " after " << steps << " steps: velocity L2 "
              << out.velocity << ", populations L2 " << out.populations << ", force x rel "
              << out.force_x << "\n";
    return out;
}

} // namespace

TEST_CASE("The D3Q19 GLSL tables match the C++ lattice", "[aero][lbm3d][shaders]") {
    const std::string_view source = render::lbm_shaders::lbm_d3q19_common.source;
    const auto as_ints = [](const auto& table) {
        std::vector<int> values;
        values.reserve(table.size());
        for (const auto v : table) {
            values.push_back(static_cast<int>(v));
        }
        return values;
    };
    CHECK(glsl_int_array(source, "LBM3_CX") == as_ints(d3q19::cx));
    CHECK(glsl_int_array(source, "LBM3_CY") == as_ints(d3q19::cy));
    CHECK(glsl_int_array(source, "LBM3_CZ") == as_ints(d3q19::cz));
    CHECK(glsl_int_array(source, "LBM3_OPPOSITE") == as_ints(d3q19::opposite));
    CHECK(glsl_int_array(source, "LBM3_MIRROR_Y") == as_ints(d3q19::mirror_y));
    CHECK(glsl_int_array(source, "LBM3_MIRROR_Z") == as_ints(d3q19::mirror_z));
}

TEST_CASE("The OpenGL backend builds the D3Q19 kernels and reports its memory",
          "[aero][lbm3d][gl]") {
    GlCompute gl = gl_compute_or_skip();
    render::GlComputeBackend& backend = *gl.backend;
    for (const std::string_view kernel : {kernel3d_step, kernel3d_moments, kernel3d_force}) {
        CHECK(backend.has_kernel(kernel));
    }
    CHECK_FALSE(backend.has_kernel("lbm_d3q19_common"));
    {
        const auto compiled = backend.compile_all();
        INFO((compiled ? std::string() : compiled.error()));
        REQUIRE(compiled);
    }
    const aero::compute::DeviceMemory memory = backend.memory();
    REQUIRE(memory.max_binding_bytes);
    CHECK(*memory.max_binding_bytes >= std::size_t{1} << 27U); // GL 4.3 minimum: 2^27 bytes
    std::cout << "measured: device " << backend.info().device << ", largest binding "
              << static_cast<double>(*memory.max_binding_bytes) / 1048576.0 << " MiB, free "
              << (memory.available_bytes ? static_cast<double>(*memory.available_bytes) / 1048576.0
                                         : -1.0)
              << " MiB (-1: not reported)\n";
    CHECK(gl.context.debug_error_count() == 0);
}

TEST_CASE("GPU and CPU D3Q19 agree on the analytic flows", "[aero][lbm3d][gl]") {
    GlCompute gl = gl_compute_or_skip();
    const Comparison poiseuille = compare(*gl.backend, poiseuille3d_config(4, 32, 4, 0.8, 1e-6),
                                          fluid_mask_3d(4, 32, 4), 4000, "Poiseuille 4x32x4");
    CHECK(poiseuille.velocity < 1e-4);
    const Comparison duct = compare(*gl.backend, duct_config(4, 20, 20, 0.8, 1e-6),
                                    fluid_mask_3d(4, 20, 20), 4000, "duct 4x20x20");
    CHECK(duct.velocity < 1e-4);
    const Comparison couette = compare(*gl.backend, couette3d_config(4, 4, 16, 0.8, 0.02),
                                       fluid_mask_3d(4, 4, 16), 4000, "Couette 4x4x16");
    CHECK(couette.velocity < 1e-4);
    CHECK(gl.context.debug_error_count() == 0);
}

TEST_CASE("GPU and CPU D3Q19 agree on a laminar sphere channel", "[aero][lbm3d][gl]") {
    GlCompute gl = gl_compute_or_skip();
    // Re 20, D 8, 64 x 32 x 32, about 5 convective times.
    const SphereSetup setup = sphere_channel(64, 32, 32, 8.0, 2.5, 0.05, 20.0);
    const Comparison bgk = compare(*gl.backend, setup.config, setup.mask, 800, "sphere Re 20");
    CHECK(bgk.velocity < 1e-4);
    CHECK(bgk.populations < 1e-4);
    CHECK(bgk.force_x < 1e-4);
    SphereSetup les = setup;
    les.config.smagorinsky = Smagorinsky{true, 0.1};
    const Comparison closure = compare(*gl.backend, les.config, les.mask, 800, "sphere Re 20 LES");
    CHECK(closure.velocity < 1e-4);
    CHECK(closure.force_x < 1e-4);
    SphereSetup interpolated = setup;
    interpolated.config.solid_wall = SolidWall::interpolated;
    const Comparison bouzidi = compare(*gl.backend, interpolated.config, interpolated.mask, 800,
                                       "sphere Re 20, interpolated wall");
    CHECK(bouzidi.velocity < 1e-4);
    CHECK(bouzidi.force_x < 1e-4);
    SphereSetup extrapolated = setup;
    extrapolated.config.outlet = Outlet::extrapolation;
    extrapolated.config.y_low = wall(WallKind::periodic);
    extrapolated.config.y_high = wall(WallKind::periodic);
    const Comparison other = compare(*gl.backend, extrapolated.config, extrapolated.mask, 800,
                                     "sphere Re 20, extrapolation outlet, periodic y");
    CHECK(other.velocity < 1e-4);
    CHECK(gl.context.debug_error_count() == 0);
}

TEST_CASE("GPU and CPU D3Q19 agree on links with unusable wall distances", "[aero][lbm3d][gl]") {
    GlCompute gl = gl_compute_or_skip();
    // The sphere channel of the interpolated-wall comparison with the
    // distances spoilt on the front half of the body (infinite on the fluid
    // side) and on its rear half (positive inside): those links take the
    // halfway wall on both solvers, the others keep their fraction.
    SphereSetup setup = sphere_channel(48, 24, 24, 8.0, 2.0, 0.05, 20.0);
    setup.config.solid_wall = SolidWall::interpolated;
    const double centre = 2.0 * setup.diameter;
    for (std::size_t z = 0; z < 24; ++z) {
        for (std::size_t y = 0; y < 24; ++y) {
            for (std::size_t x = 0; x < 48; ++x) {
                const std::size_t c = setup.mask.index(x, y, z);
                const bool front = static_cast<double>(x) < centre;
                if (front && setup.mask.solid[c] == 0) {
                    setup.mask.distance[c] = std::numeric_limits<double>::infinity();
                } else if (!front && setup.mask.solid[c] != 0) {
                    setup.mask.distance[c] = 1.0;
                }
            }
        }
    }
    const Comparison spoilt =
        compare(*gl.backend, setup.config, setup.mask, 400, "sphere Re 20, unusable distances");
    CHECK(spoilt.velocity < 1e-4);
    CHECK(spoilt.populations < 1e-4);
    CHECK(spoilt.force_x < 1e-4);
    CHECK(gl.context.debug_error_count() == 0);
}

TEST_CASE("GPU D3Q19: a uniform stream between co-moving walls stays uniform at any density",
          "[aero][lbm3d][gl]") {
    GlCompute gl = gl_compute_or_skip();
    for (const double rho : {0.7, 2.0}) {
        Lbm3DConfig config = periodic_box(4, 8, 8, 0.7);
        const double u = 0.03;
        config.y_low = wall(WallKind::no_slip, u);
        config.y_high = wall(WallKind::no_slip, u);
        config.z_low = wall(WallKind::no_slip, u);
        config.z_high = wall(WallKind::no_slip, u);
        auto solver = take(make_gpu_solver(*gl.backend, config, fluid_mask_3d(4, 8, 8)));
        require_ok(solver->initialize(uniform_state3d(rho, u)));
        require_ok(solver->step(300));
        const MacroFields3D fields = take(solver->macroscopic());
        double worst = 0.0;
        for (std::size_t c = 0; c < fields.density.size(); ++c) {
            worst = std::max(worst, std::abs(fields.density[c] - rho) / rho);
            worst = std::max(worst, std::abs(fields.velocity_x[c] - u) / u);
        }
        std::cout << "measured: GPU co-moving walls at density " << rho << ": " << worst << "\n";
        CHECK(worst < 1e-4);
    }
    CHECK(gl.context.debug_error_count() == 0);
}

TEST_CASE("The 3D IAeroSolver adapter runs on the GPU backend", "[aero][lbm3d][gl]") {
    GlCompute gl = gl_compute_or_skip();
    aero::AeroGeometry geometry;
    geometry.mesh = geom::sphere_profile(0.01).mesh({32, 0.1});
    geometry.reference.length = 0.02;
    geometry.reference.area = 0.25 * std::numbers::pi * 0.02 * 0.02;
    aero::FlowConditions conditions;
    conditions.speed = 0.1;
    conditions.density = 1000.0;
    conditions.viscosity = 1000.0 * 0.1 * 0.02 / 50.0; // Re 50
    LbmSolver3DOptions options;
    options.resolution = LatticeResolution{10.0, 0.05};
    options.upstream = 2.0;
    options.downstream = 4.0;
    options.width = 4.0;
    options.duration = 5.0;
    options.average_window = 2.0;
    options.steps_per_call = 1000;
    LbmAeroSolver3D cpu(options);
    cpu.setup(geometry, conditions);
    while (!cpu.step()) {
    }
    options.backend = gl.backend.get();
    LbmAeroSolver3D gpu(options);
    gpu.setup(geometry, conditions);
    while (!gpu.step()) {
    }
    const double cd_cpu = cpu.result().cd;
    const double cd_gpu = gpu.result().cd;
    std::cout << "measured: 3D adapter Cd CPU " << cd_cpu << ", GPU " << cd_gpu << "\n";
    CHECK(std::abs(cd_gpu - cd_cpu) / cd_cpu < 1e-4);
    CHECK(gpu.result().iterations == cpu.result().iterations);
    CHECK(gl.context.debug_error_count() == 0);
}

TEST_CASE("GPU D3Q19 throughput at 128^3", "[.benchmark][aero][lbm3d][gl]") {
    GlCompute gl = gl_compute_or_skip();
    for (const bool les : {false, true}) {
        SphereSetup setup = sphere_channel(128, 128, 128, 24.0, 2.5, 0.05, 300.0);
        setup.config.smagorinsky = Smagorinsky{les, 0.1};
        auto solver = take(make_gpu_solver(*gl.backend, setup.config, setup.mask));
        require_ok(solver->step(50)); // warm-up: compile, first touch
        gl.backend->finish();
        const std::size_t steps = 1000;
        const auto start = std::chrono::steady_clock::now();
        require_ok(solver->step(steps));
        gl.backend->finish();
        const double seconds =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        const double rate = static_cast<double>(steps) / seconds;
        std::cout << "measured: GPU D3Q19 128^3 sphere channel" << (les ? " (Smagorinsky)" : "")
                  << ": " << rate << " steps/s, " << rate * 128.0 * 128.0 * 128.0 / 1e6
                  << " million lattice updates per second on " << gl.backend->info().device << "\n";
        CHECK(rate >= 60.0);
        const double mass = take(solver->mass());
        CHECK(std::isfinite(mass));
    }
}
