// The GPU solver's protocol on the host-memory NullComputeBackend: buffer
// layout, the shifted binary32 initial state, ping-pong bindings, uniforms,
// barriers and the conversion of what the kernels write back. The kernels
// themselves run in tests/aero/lbm/test_lbm_gpu.cpp on OpenGL.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <set>
#include <string>
#include <vector>

#include "lbm_test_support.h"
#include "orrery/aero/compute/float32_pack.h"
#include "orrery/aero/compute/null_compute_backend.h"
#include "orrery/aero/lbm/lbm2d.h"

using namespace orrery::aero;
using namespace orrery::aero::lbm;
using namespace orrery::aero::lbm::test;

namespace {

const compute::NullComputeBackend::RecordedUniform*
find_uniform(const compute::NullComputeBackend::RecordedDispatch& d, const std::string& name) {
    for (const auto& u : d.uniforms) {
        if (u.name == name) {
            return &u;
        }
    }
    return nullptr;
}

compute::BufferId bound_at(const compute::NullComputeBackend::RecordedDispatch& d,
                           std::uint32_t binding) {
    for (const auto& b : d.buffers) {
        if (b.binding == binding) {
            return b.buffer;
        }
    }
    return {};
}

} // namespace

TEST_CASE("GPU solver allocates its buffers and uploads the shifted state", "[aero][lbm][gpu]") {
    compute::NullComputeBackend backend;
    const CylinderSetup setup = cylinder_setup(40, 20, 6.0, 12.0, 10.0, 0.05, 50.0);
    const std::size_t links = boundary_links(setup.config, setup.mask).size();
    REQUIRE(links > 0);
    auto solver = take(make_gpu_solver(backend, setup.config, setup.mask));
    // Two population buffers, solid flags, moments, outlet history, links and
    // link forces.
    CHECK(backend.live_buffers() == 7);
    CHECK(solver->steps() == 0);

    // The readback of the untouched state is the binary32 initial state.
    const std::vector<double> gpu = take(solver->populations());
    const std::vector<double> exact =
        initial_populations(setup.config, setup.mask, uniform_inlet_state(setup.config));
    REQUIRE(gpu.size() == exact.size());
    double worst = 0.0;
    for (std::size_t k = 0; k < gpu.size(); ++k) {
        worst = std::max(worst, std::abs(gpu[k] - exact[k]));
    }
    // Shifted storage: the error is that of the small deviation f - w.
    CHECK(worst < 1e-9);
    CHECK(worst > 0.0);
    CHECK(take(solver->mass()) ==
          Catch::Approx(static_cast<double>(std::size_t{800} - setup.mask.count())).epsilon(1e-7));
}

TEST_CASE("GPU solver steps ping-pong between its population buffers", "[aero][lbm][gpu]") {
    compute::NullComputeBackend backend;
    CylinderSetup setup = cylinder_setup(40, 20, 6.0, 12.0, 10.0, 0.05, 50.0);
    setup.config.smagorinsky = Smagorinsky{true, 0.2};
    auto solver = take(make_gpu_solver(backend, setup.config, setup.mask));
    backend.clear_log();
    const std::size_t barriers = backend.barrier_count();
    require_ok(solver->step(3));
    CHECK(solver->steps() == 3);
    const auto& log = backend.dispatches();
    REQUIRE(log.size() == 3);
    CHECK(backend.barrier_count() == barriers + 3);
    for (std::size_t n = 0; n < 3; ++n) {
        CHECK(log[n].kernel == std::string(kernel_step));
        CHECK(log[n].global.x == 40);
        CHECK(log[n].global.y == 20);
        CHECK(log[n].barriers_before == barriers + n);
    }
    // Source and destination alternate.
    CHECK(bound_at(log[0], 0) == bound_at(log[1], 2));
    CHECK(bound_at(log[0], 2) == bound_at(log[1], 0));
    CHECK(bound_at(log[2], 0) == bound_at(log[0], 0));
    CHECK(bound_at(log[0], 1) == bound_at(log[1], 1));

    const auto* tau = find_uniform(log[0], "u_tau");
    REQUIRE(tau != nullptr);
    CHECK(std::get<compute::Float32Uniform>(tau->value).value == setup.config.tau);
    const auto* c2 = find_uniform(log[0], "u_smagorinsky_c2");
    REQUIRE(c2 != nullptr);
    CHECK(std::get<compute::Float32Uniform>(c2->value).value == Catch::Approx(0.04));
    const auto* mode = find_uniform(log[0], "u_x_mode");
    REQUIRE(mode != nullptr);
    CHECK(std::get<std::uint32_t>(mode->value) == 1U);
    const auto* top = find_uniform(log[0], "u_top_kind");
    REQUIRE(top != nullptr);
    CHECK(std::get<std::uint32_t>(top->value) == 1U); // slip
}

TEST_CASE("GPU solver converts the kernel outputs on readback", "[aero][lbm][gpu]") {
    compute::NullComputeBackend backend;
    const CylinderSetup setup = cylinder_setup(16, 8, 3.0, 6.0, 4.0, 0.05, 20.0);
    const std::size_t cells = std::size_t{16} * 8;
    const std::vector<BoundaryLink> links = boundary_links(setup.config, setup.mask);

    // Stand-ins for the kernels that write recognisable values.
    backend.register_kernel(
        std::string(kernel_moments),
        [cells](const compute::DispatchCommand&, const compute::NullComputeBackend::Bindings& b) {
            std::vector<double> planes(3 * cells);
            for (std::size_t c = 0; c < cells; ++c) {
                planes[c] = 0.001;             // rho - 1
                planes[cells + c] = 0.04;      // ux
                planes[2 * cells + c] = -0.01; // uy
            }
            const std::vector<std::byte> bytes = compute::pack_float32(planes);
            std::memcpy(b.at(3).data(), bytes.data(), bytes.size());
            return compute::Status(std::monostate{});
        });
    backend.register_kernel(std::string(kernel_force),
                            [](const compute::DispatchCommand& command,
                               const compute::NullComputeBackend::Bindings& b) {
                                std::vector<double> parts(std::size_t{2} * command.global.x, 0.0);
                                const std::vector<std::byte> bytes = compute::pack_float32(parts);
                                std::memcpy(b.at(5).data(), bytes.data(), bytes.size());
                                return compute::Status(std::monostate{});
                            });
    auto solver = take(make_gpu_solver(backend, setup.config, setup.mask));
    const MacroFields2D fields = take(solver->macroscopic());
    CHECK(fields.density[5] == Catch::Approx(1.001).epsilon(1e-7));
    CHECK(fields.velocity_x[5] == Catch::Approx(0.04).epsilon(1e-7));
    CHECK(fields.velocity_y[5] == Catch::Approx(-0.01).epsilon(1e-7));
    CHECK(fields.solid == setup.mask.solid);

    // With zero shifted parts the force is the weight term 2 c_o w_o over the
    // links, which vanishes for a closed body.
    const Force2D f = take(solver->force());
    CHECK(std::abs(f.x) < 1e-15);
    CHECK(std::abs(f.y) < 1e-15);
    const auto& last = backend.dispatches().back();
    CHECK(last.kernel == std::string(kernel_force));
    CHECK(last.global.x == links.size());
}

TEST_CASE("GPU solver reports backend failures", "[aero][lbm][gpu]") {
    compute::NullComputeBackend strict(false);
    const CylinderSetup setup = cylinder_setup(16, 8, 3.0, 6.0, 4.0, 0.05, 20.0);
    auto solver = take(make_gpu_solver(strict, setup.config, setup.mask));
    const Status stepped = solver->step(1);
    REQUIRE_FALSE(stepped);
    CHECK(stepped.error().find("lbm_d2q9_step") != std::string::npos);
    CHECK_FALSE(solver->macroscopic());
    CHECK_FALSE(solver->force());

    Lbm2DConfig bad = setup.config;
    bad.tau = 0.4;
    CHECK_FALSE(make_gpu_solver(strict, bad, setup.mask));
}
