// The GPU D3Q19 driver on the host-memory NullComputeBackend: buffer layout,
// dispatch protocol, the binary32 parameter checks and the memory checks,
// without a GPU.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

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
        worst = worse(worst, std::abs(back[k] - expected[k]));
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

TEST_CASE("The GPU D3Q19 force dispatch stays within 65535 work groups per dimension",
          "[aero][lbm3d]") {
    // A periodic 128 x 128 x 66 lattice, fluid where x and y are even: each
    // fluid node has 16 links (all but +-z), 4,325,376 in all, more than the
    // 65535 x 64 = 4,194,240 work items one dimension of 64-item groups
    // reaches at the minimum GL_MAX_COMPUTE_WORK_GROUP_COUNT.
    constexpr std::uint64_t kLocalSize = 64; // lbm_d3q19_force.comp.glsl
    constexpr std::uint64_t kMaxGroups = 65535;
    const Lbm3DConfig config = periodic_box(128, 128, 66, 0.6);
    SolidMask3D mask = fluid_mask_3d(128, 128, 66);
    for (std::size_t z = 0; z < 66; ++z) {
        for (std::size_t y = 0; y < 128; ++y) {
            for (std::size_t x = 0; x < 128; ++x) {
                mask.solid[mask.index(x, y, z)] = x % 2 == 0 && y % 2 == 0 ? 0 : 1;
            }
        }
    }
    // The force kernel emulated on the host with the device limit: every
    // work item of the rounded-up groups forms its link index as the GLSL
    // kernel does and counts its visit.
    compute::NullComputeBackend backend;
    std::vector<std::uint8_t> visits;
    backend.register_kernel(
        std::string(kernel3d_force),
        [&visits](const compute::DispatchCommand& command,
                  const compute::NullComputeBackend::Bindings&) -> compute::Status {
            const std::uint64_t groups_x = (command.global.x + kLocalSize - 1) / kLocalSize;
            if (groups_x > kMaxGroups || command.global.y > kMaxGroups ||
                command.global.z > kMaxGroups) {
                return orrery::core::failure(std::string("more than 65535 work groups"));
            }
            if (command.global.z != 1) {
                return orrery::core::failure(std::string("the emulation tiles x and y only"));
            }
            std::uint64_t count = 0;
            std::uint64_t row = 0;
            for (const compute::Uniform& uniform : command.uniforms) {
                if (uniform.name == "u_link_count") {
                    count = std::get<std::uint32_t>(uniform.value);
                } else if (uniform.name == "u_link_row") {
                    row = std::get<std::uint32_t>(uniform.value);
                }
            }
            visits.assign(count, 0);
            for (std::uint64_t y = 0; y < command.global.y; ++y) {
                for (std::uint64_t x = 0; x < groups_x * kLocalSize; ++x) {
                    const std::uint64_t k = x + row * y;
                    if (k < count && visits[k] < 255) {
                        ++visits[k];
                    }
                }
            }
            return std::monostate{};
        });
    auto solver = take(make_gpu_solver(backend, config, mask));
    const std::size_t links = solver->links().size();
    REQUIRE(links == std::size_t{128} * 128 * 66 / 4 * 16);
    REQUIRE(links > kMaxGroups * kLocalSize);
    const auto force = solver->force();
    INFO((force ? std::string() : force.error()));
    REQUIRE(force);
    REQUIRE(visits.size() == links);
    CHECK(std::all_of(visits.begin(), visits.end(), [](std::uint8_t v) { return v == 1; }));
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
    // No links; the one wall slot of the halfway wall.
    const std::size_t total = *gpu_memory_bytes(config, 0, 0);
    CHECK(total == *gpu_memory_bytes(config) + std::size_t{19} * 4);

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

TEST_CASE("The GPU D3Q19 memory check counts the boundary tables", "[aero][lbm3d]") {
    // A periodic 32^3 checkerboard: every fluid node has 6 links (the face
    // neighbours are solid). Besides the lattice buffers the solver
    // allocates per link two uint32 (cell, direction) and one float (force),
    // and for the interpolated wall 19 floats per fluid node with links
    // (19 in all for the halfway wall).
    for (const SolidWall wall_kind : {SolidWall::interpolated, SolidWall::halfway}) {
        Lbm3DConfig config = periodic_box(32, 32, 32, 0.6);
        config.solid_wall = wall_kind;
        SolidMask3D mask = fluid_mask_3d(32, 32, 32);
        mask.distance.assign(mask.solid.size(), 0.5);
        std::size_t fluid = 0;
        for (std::size_t z = 0; z < 32; ++z) {
            for (std::size_t y = 0; y < 32; ++y) {
                for (std::size_t x = 0; x < 32; ++x) {
                    const std::size_t c = mask.index(x, y, z);
                    const bool solid = (x + y + z) % 2 == 1;
                    mask.solid[c] = solid ? 1 : 0;
                    mask.distance[c] = solid ? -0.5 : 0.5;
                    fluid += solid ? 0 : 1;
                }
            }
        }
        const std::size_t links = boundary_links(config, mask).size();
        REQUIRE(links == 6 * fluid);
        const std::size_t slots = wall_kind == SolidWall::interpolated ? fluid : 1;
        const std::size_t base = *gpu_memory_bytes(config);
        const std::size_t total = base + slots * 19 * 4 + links * (2 * 4 + 4);
        INFO("base " << base << ", total " << total);
        CHECK(gpu_memory_bytes(config, links, wall_kind == SolidWall::interpolated ? fluid : 0) ==
              total);

        compute::NullComputeBackend backend;
        backend.set_memory({base + (total - base) / 2, std::nullopt});
        auto between = make_gpu_solver(backend, config, mask);
        REQUIRE_FALSE(between);
        CHECK(between.error().find("available") != std::string::npos);
        CHECK(backend.live_buffers() == 0);
        backend.set_memory({total - 1, std::nullopt});
        CHECK_FALSE(make_gpu_solver(backend, config, mask));
        CHECK(backend.live_buffers() == 0);
        backend.set_memory({total, std::nullopt});
        CHECK(make_gpu_solver(backend, config, mask));
    }
}
