// The OpenGL compute backend and the GPU D2Q9 solver against the CPU
// reference. Tests tagged [gl] need an OpenGL 4.6 context (lbm_gl_support.h).
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <regex>
#include <set>
#include <string>
#include <vector>

#include "lbm_gl_support.h"
#include "lbm_test_support.h"
#include "orrery/aero/compute/float32_pack.h"
#include "orrery/aero/lbm/d2q9.h"
#include "orrery/aero/lbm/force_history.h"
#include "orrery/aero/lbm/lbm2d.h"
#include "orrery/render/embedded_lbm_shaders.h"

using namespace orrery;
using namespace orrery::aero::lbm;
using namespace orrery::aero::lbm::test;

namespace {

std::string read_normalized(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    std::error_code ec;
    const auto size = std::filesystem::file_size(path, ec);
    std::string text;
    if (file && !ec) {
        text.assign(static_cast<std::size_t>(size), '\0');
        file.read(text.data(), static_cast<std::streamsize>(text.size()));
        text.resize(static_cast<std::size_t>(file.gcount()));
    }
    std::string out;
    out.reserve(text.size());
    for (std::size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '\r' && i + 1 < text.size() && text[i + 1] == '\n') {
            continue;
        }
        out += text[i];
    }
    return out;
}

// Integers of the GLSL array initialiser `name[9] = int[9](...)`.
std::vector<int> glsl_int_array(std::string_view source, const std::string& name) {
    const std::regex pattern(name + R"(\[9\]\s*=\s*int\[9\]\(([^)]*)\))");
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
    double density = 0.0;
    double populations = 0.0;
    double force_x = 0.0; // relative to the CPU value
};

// Runs the same problem on both solvers for `steps` steps.
Comparison compare(aero::compute::IComputeBackend& backend, const Lbm2DConfig& config,
                   const SolidMask2D& mask, std::size_t steps, const InitialState* initial,
                   const char* label) {
    auto cpu = take(make_cpu_solver(config, mask));
    auto gpu = take(make_gpu_solver(backend, config, mask));
    if (initial != nullptr) {
        require_ok(cpu->initialize(*initial));
        require_ok(gpu->initialize(*initial));
    }
    require_ok(cpu->step(steps));
    require_ok(gpu->step(steps));
    const MacroFields2D c = take(cpu->macroscopic());
    const MacroFields2D g = take(gpu->macroscopic());
    Comparison out;
    out.velocity = velocity_l2(g, c);
    out.density = density_deviation_error(g, c);
    out.populations = l2_relative(take(gpu->populations()), take(cpu->populations()));
    const Force2D fc = take(cpu->force());
    const Force2D fg = take(gpu->force());
    out.force_x = fc.x != 0.0 ? std::abs(fg.x - fc.x) / std::abs(fc.x) : std::abs(fg.x);
    std::cout << "measured: GPU vs CPU " << label << " after " << steps << " steps: velocity L2 "
              << out.velocity << ", density deviation " << out.density << ", populations L2 "
              << out.populations << ", force x rel " << out.force_x << "\n";
    return out;
}

InitialState taylor_green(std::size_t n, double u0) {
    const double k = 2.0 * std::numbers::pi / static_cast<double>(n);
    return [k, u0](std::size_t xi, std::size_t yi) {
        const double x = static_cast<double>(xi);
        const double y = static_cast<double>(yi);
        const double rho = 1.0 - 0.75 * u0 * u0 * (std::cos(2.0 * k * x) + std::cos(2.0 * k * y));
        return d2q9::Moments{rho, -u0 * std::cos(k * x) * std::sin(k * y),
                             u0 * std::sin(k * x) * std::cos(k * y)};
    };
}

} // namespace

TEST_CASE("Every LBM shader file is embedded byte for byte", "[aero][lbm][shaders]") {
    const std::filesystem::path dir = ORRERY_RENDER_LBM_SHADER_DIR;
    std::set<std::string> files;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (entry.path().extension() == ".glsl") {
            files.insert(entry.path().filename().string());
        }
    }
    std::set<std::string> embedded;
    for (const render::lbm_shaders::EmbeddedShader* shader : render::lbm_shaders::all) {
        embedded.insert(std::string(shader->name));
        INFO(shader->name);
        CHECK(shader->source == read_normalized(dir / std::string(shader->name)));
    }
    CHECK(embedded == files);
}

TEST_CASE("The GLSL lattice tables match the C++ lattice", "[aero][lbm][shaders]") {
    const std::string_view source = render::lbm_shaders::lbm_d2q9_common.source;
    const auto as_ints = [](const auto& table) {
        return std::vector<int>(table.begin(), table.end());
    };
    CHECK(glsl_int_array(source, "LBM_CX") == as_ints(d2q9::cx));
    CHECK(glsl_int_array(source, "LBM_CY") == as_ints(d2q9::cy));
    std::vector<int> opposite;
    std::vector<int> mirror;
    for (std::size_t i = 0; i < d2q9::q; ++i) {
        opposite.push_back(static_cast<int>(d2q9::opposite[i]));
        mirror.push_back(static_cast<int>(d2q9::mirror_y[i]));
    }
    CHECK(glsl_int_array(source, "LBM_OPPOSITE") == opposite);
    CHECK(glsl_int_array(source, "LBM_MIRROR_Y") == mirror);
}

TEST_CASE("The OpenGL compute backend manages buffers and kernels", "[aero][lbm][gl]") {
    GlCompute gl = gl_compute_or_skip();
    render::GlComputeBackend& backend = *gl.backend;
    CHECK(backend.info().name == "opengl");
    CHECK(backend.info().executes_kernels);
    CHECK(backend.kernel_names() ==
          std::vector<std::string>{"lbm_d2q9_force", "lbm_d2q9_moments", "lbm_d2q9_step"});
    CHECK(backend.has_kernel(kernel_step));
    CHECK_FALSE(backend.has_kernel("lbm_d2q9_common"));
    {
        const auto compiled = backend.compile_all();
        INFO((compiled ? std::string() : compiled.error()));
        REQUIRE(compiled);
    }

    const aero::compute::BufferId id = take(backend.create_buffer(64));
    CHECK(backend.buffer_size(id) == 64);
    std::vector<std::byte> bytes(64, std::byte{0x5a});
    REQUIRE(backend.readback(id, 0, bytes));
    for (const std::byte b : bytes) {
        CHECK(b == std::byte{0});
    }
    const std::vector<double> values{1.0, -0.25, 3.5, 1e-3};
    REQUIRE(backend.upload(id, 16, aero::compute::pack_float32(values)));
    std::vector<std::byte> part(16);
    REQUIRE(backend.readback(id, 16, part));
    std::vector<double> back(4);
    REQUIRE(aero::compute::unpack_float32(part, back));
    CHECK(back[1] == -0.25);
    CHECK(back[3] == aero::compute::round_to_float32(1e-3));
    CHECK_FALSE(backend.upload(id, 60, part));
    CHECK_FALSE(backend.readback(aero::compute::BufferId{4242}, 0, part));
    CHECK_FALSE(backend.create_buffer(0));

    CHECK_FALSE(backend.dispatch({"no_such_kernel", {1, 1, 1}, {}, {}}));
    CHECK_FALSE(backend.dispatch({kernel_moments, {0, 1, 1}, {}, {}}));
    // u_nx is declared uint: an int32 value is a type error.
    const std::array<aero::compute::Uniform, 1> wrong{
        aero::compute::Uniform{"u_nx", std::int32_t{4}}};
    const auto mismatch = backend.dispatch({kernel_moments, {1, 1, 1}, wrong, {}});
    REQUIRE_FALSE(mismatch);
    CHECK(mismatch.error().find("u_nx") != std::string::npos);
    backend.release_buffer(id);
    CHECK(backend.buffer_size(id) == 0);
    CHECK(gl.context.debug_error_count() == 0);
}

TEST_CASE("GPU and CPU agree on the analytic flows", "[aero][lbm][gl]") {
    GlCompute gl = gl_compute_or_skip();
    const Comparison poiseuille = compare(*gl.backend, poiseuille_config(8, 64, 0.8, 1e-6),
                                          fluid_mask(8, 64), 4000, nullptr, "Poiseuille 8x64");
    CHECK(poiseuille.velocity < 1e-4);
    const Comparison couette = compare(*gl.backend, couette_config(8, 32, 0.9, 0.03),
                                       fluid_mask(8, 32), 4000, nullptr, "Couette 8x32");
    CHECK(couette.velocity < 1e-4);

    Lbm2DConfig box;
    box.nx = 64;
    box.ny = 64;
    box.tau = 0.6;
    box.x_boundary = XBoundary::periodic;
    box.bottom = Wall{WallKind::periodic, 0.0};
    box.top = Wall{WallKind::periodic, 0.0};
    const InitialState vortex = taylor_green(64, 0.03);
    const Comparison tg =
        compare(*gl.backend, box, fluid_mask(64, 64), 2000, &vortex, "Taylor-Green 64x64");
    CHECK(tg.velocity < 1e-4);
    CHECK(tg.density < 1e-3);
    CHECK(gl.context.debug_error_count() == 0);
}

TEST_CASE("GPU and CPU agree on the cylinder channel", "[aero][lbm][gl]") {
    GlCompute gl = gl_compute_or_skip();
    // Re 100, D = 10: Zou-He inlet, extrapolation outlet, slip sides, solid
    // bounce-back and momentum exchange, with and without the LES closure.
    const CylinderSetup setup = cylinder_setup(200, 100, 10.0, 50.0, 50.5, 0.05, 100.0);
    const Comparison laminar =
        compare(*gl.backend, setup.config, setup.mask, 4000, nullptr, "cylinder Re 100");
    CHECK(laminar.velocity < 1e-4);
    CHECK(laminar.force_x < 1e-4);

    CylinderSetup les = setup;
    les.config.smagorinsky = Smagorinsky{true, 0.17};
    const Comparison closure =
        compare(*gl.backend, les.config, les.mask, 4000, nullptr, "cylinder Re 100 LES");
    CHECK(closure.velocity < 1e-4);
    CHECK(closure.force_x < 1e-4);
    CHECK(gl.context.debug_error_count() == 0);
}

TEST_CASE("GPU and CPU agreement over a long cylinder run", "[.measure][aero][lbm][gl]") {
    GlCompute gl = gl_compute_or_skip();
    const CylinderSetup setup = cylinder_setup(400, 200, 20.0, 100.0, 100.5, 0.05, 100.0);
    auto cpu = take(make_cpu_solver(setup.config, setup.mask));
    auto gpu = take(make_gpu_solver(*gl.backend, setup.config, setup.mask));
    for (int block = 1; block <= 30; ++block) {
        require_ok(cpu->step(2000));
        require_ok(gpu->step(2000));
        const MacroFields2D c = take(cpu->macroscopic());
        const MacroFields2D g = take(gpu->macroscopic());
        const Force2D fc = take(cpu->force());
        const Force2D fg = take(gpu->force());
        const double cd = force_coefficients(fc.x, fc.y, 0.05, 20.0).drag;
        const double cl = force_coefficients(fc.x, fc.y, 0.05, 20.0).lift;
        const double cl_gpu = force_coefficients(fg.x, fg.y, 0.05, 20.0).lift;
        std::cout << "measured: long run step " << cpu->steps() << " velocity L2 "
                  << velocity_l2(g, c) << " Cd " << cd << " Cl cpu " << cl << " gpu " << cl_gpu
                  << "\n";
    }
}

TEST_CASE("GPU D2Q9 throughput at 2048 x 1024", "[.benchmark][aero][lbm][gl]") {
    GlCompute gl = gl_compute_or_skip();
    const CylinderSetup setup = cylinder_setup(2048, 1024, 64.0, 400.0, 512.5, 0.05, 1000.0);
    CylinderSetup les = setup;
    les.config.smagorinsky = Smagorinsky{true, 0.1};
    for (const CylinderSetup* run : std::array<const CylinderSetup*, 2>{&setup, &les}) {
        auto gpu = take(make_gpu_solver(*gl.backend, run->config, run->mask));
        require_ok(gpu->step(50)); // warm-up: compile, first dispatches
        gl.backend->finish();
        const std::size_t steps = 1000;
        const auto start = std::chrono::steady_clock::now();
        require_ok(gpu->step(steps));
        gl.backend->finish();
        const double seconds =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        const double rate = static_cast<double>(steps) / seconds;
        const double mlups = rate * 2048.0 * 1024.0 / 1e6;
        std::cout << "measured: 2048x1024 D2Q9 "
                  << (run->config.smagorinsky.enabled ? "BGK+Smagorinsky" : "BGK") << " " << rate
                  << " steps/s (" << mlups << " MLUPS) on " << gl.backend->info().device << "\n";
        CHECK(rate >= 200.0);
        CHECK(std::isfinite(take(gpu->force()).x));
    }
}
