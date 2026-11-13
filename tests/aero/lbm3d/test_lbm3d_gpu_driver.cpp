// The GPU D3Q19 driver on the host-memory NullComputeBackend: buffer layout,
// dispatch protocol, the binary32 parameter checks and the memory checks,
// without a GPU.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <string>

#include "lbm3d_test_support.h"
#include "orrery/aero/compute/null_compute_backend.h"
#include "orrery/aero/lbm3d/lbm3d.h"

using namespace orrery::aero;
using namespace orrery::aero::lbm;
using namespace orrery::aero::lbm::test;
using Catch::Approx;

TEST_CASE("The GPU D3Q19 driver uploads, ping-pongs and reads back", "[aero][lbm3d]") {
    compute::NullComputeBackend backend;
    const SphereSetup setup = sphere_channel(24, 12, 12, 6.0, 2.0, 0.05, 20.0);
    auto solver = take(make_gpu_solver(backend, setup.config, setup.mask));
    // Two population buffers, node classes, wall fractions, moments, outlet
    // history, links and link forces.
    CHECK(backend.live_buffers() == 8);
    CHECK_FALSE(solver->links().empty());

    // The initial populations come back within binary32 rounding of the
    // shifted values.
    const std::vector<double> expected =
        initial_populations(setup.config, setup.mask, uniform_inlet_state(setup.config));
    const std::vector<double> back = take(solver->populations());
    REQUIRE(back.size() == expected.size());
    double worst = 0.0;
    for (std::size_t k = 0; k < back.size(); ++k) {
        worst = std::max(worst, std::abs(back[k] - expected[k]));
    }
    CHECK(worst < 1e-8);

    backend.clear_log();
    require_ok(solver->step(3));
    CHECK(solver->steps() == 3);
    const auto& log = backend.dispatches();
    REQUIRE(log.size() == 3);
    for (std::size_t n = 0; n < 3; ++n) {
        CHECK(log[n].kernel == kernel3d_step);
        CHECK(log[n].global.x == 24);
        CHECK(log[n].global.y == 12);
        CHECK(log[n].global.z == 12);
        CHECK(log[n].barriers_before == backend.barrier_count() - 3 + n);
    }
    // Ping-pong: the destination of one step is the source of the next.
    const auto binding = [](const compute::NullComputeBackend::RecordedDispatch& d,
                            std::uint32_t point) {
        for (const compute::BufferBinding& b : d.buffers) {
            if (b.binding == point) {
                return b.buffer;
            }
        }
        return compute::BufferId{};
    };
    CHECK(binding(log[0], 2) == binding(log[1], 0));
    CHECK(binding(log[1], 2) == binding(log[2], 0));
    CHECK(binding(log[0], 1).valid());
    CHECK(binding(log[0], 6).valid());
    CHECK(binding(log[0], 7).valid());

    // The force kernel is a no-op here: the zero-filled link buffer means
    // fs_o + fs_j = 0 on every link, so the force is the weight part,
    // sum 2 c_o w_o.
    backend.clear_log();
    const Force3D f = take(solver->force());
    REQUIRE(backend.dispatches().size() == 1);
    CHECK(backend.dispatches()[0].kernel == kernel3d_force);
    Force3D weights;
    for (const BoundaryLink3D& link : solver->links()) {
        add_link_force(weights, setup.config, link,
                       2.0 * d3q19::w[d3q19::opposite[link.direction]]);
    }
    CHECK(f.force[0] == Approx(weights.force[0]).margin(1e-12));
    CHECK(f.moment[1] == Approx(weights.moment[1]).margin(1e-9));

    const MacroFields3D fields = take(solver->macroscopic());
    CHECK(fields.density.size() == setup.mask.solid.size());
    CHECK(fields.velocity_z.size() == setup.mask.solid.size());
}

TEST_CASE("The GPU D3Q19 driver refuses what binary32 cannot carry", "[aero][lbm3d]") {
    compute::NullComputeBackend backend;
    Lbm3DConfig config = periodic_box(8, 8, 8, 0.50000001);
    const SolidMask3D mask = fluid_mask_3d(8, 8, 8);
    REQUIRE(make_cpu_solver(config, mask)); // positive viscosity in double
    auto rounded = make_gpu_solver(backend, config, mask);
    REQUIRE_FALSE(rounded);
    CHECK(rounded.error().find("tau") != std::string::npos);
    CHECK(rounded.error().find("binary32") != std::string::npos);
    CHECK(backend.live_buffers() == 0); // refused before allocating

    config.tau = 0.6;
    config.body_force_y = 1e100;
    auto huge = make_gpu_solver(backend, config, mask);
    REQUIRE_FALSE(huge);
    CHECK(huge.error().find("body force y") != std::string::npos);
    CHECK(backend.live_buffers() == 0);
}

TEST_CASE("The GPU D3Q19 driver checks the device memory", "[aero][lbm3d]") {
    compute::NullComputeBackend backend;
    const Lbm3DConfig config = periodic_box(16, 16, 16, 0.6);
    const SolidMask3D mask = fluid_mask_3d(16, 16, 16);
    const std::size_t population = *gpu_population_buffer_bytes(config);
    const std::size_t total = *gpu_memory_bytes(config);

    backend.set_memory({std::nullopt, population - 1});
    auto binding = make_gpu_solver(backend, config, mask);
    REQUIRE_FALSE(binding);
    CHECK(binding.error().find("largest buffer") != std::string::npos);
    CHECK(backend.live_buffers() == 0);

    backend.set_memory({total - 1, std::nullopt});
    auto full = make_gpu_solver(backend, config, mask);
    REQUIRE_FALSE(full);
    CHECK(full.error().find("available") != std::string::npos);
    CHECK(backend.live_buffers() == 0);

    backend.set_memory({total, population});
    CHECK(make_gpu_solver(backend, config, mask));
}
