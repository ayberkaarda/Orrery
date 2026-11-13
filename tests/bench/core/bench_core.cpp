// Core benchmarks (docs/performance.md): gravity evaluation time for
// N = 100 .. 100k (spec section 8: Barnes-Hut N = 10k < 8 ms, N = 100k
// < 120 ms on 8 cores) and the determinism contract (10k steps
// bit-identical, against the golden hashes of tests/core/test_determinism.cpp).
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "../../core/gravity_test_support.h"
#include "bench_record.h"
#include "orrery/core/barnes_hut.h"
#include "orrery/core/force_model.h"
#include "orrery/core/integrator.h"
#include "orrery/core/state_hash.h"

using namespace orrery::core;
using orrery::bench::cat;
using orrery::bench::Goal;
using orrery::bench::Measurement;

namespace {

// Golden hashes after 10000 steps; must match tests/core/test_determinism.cpp.
constexpr std::uint64_t golden_leapfrog = 0x08ca0a8c4993debeULL;
constexpr std::uint64_t golden_yoshida4 = 0xd0c645137b27a3cdULL;
constexpr std::uint64_t golden_rk4 = 0xab3b17d03159374dULL;

// Uniform double in [0, 1) from the top 53 bits of mt19937_64.
double uniform(std::mt19937_64& rng) {
    return static_cast<double>(rng() >> 11U) * 0x1p-53;
}

// Seeded Plummer sphere (scale radius 1 AU, truncated at 10 AU), masses
// uniform in [0.5, 1.5] / n Msun, at rest: the cluster of the Barnes-Hut
// tests (test_barnes_hut.cpp).
BodyStore plummer_cluster(std::size_t n, std::uint64_t seed) {
    std::mt19937_64 rng(seed);
    BodyStore store;
    for (std::size_t i = 0; i < n; ++i) {
        double r = 0.0;
        do {
            const double u = std::max(uniform(rng), 1e-12);
            r = 1.0 / std::sqrt(std::pow(u, -2.0 / 3.0) - 1.0);
        } while (r > 10.0);
        const double cos_t = 2.0 * uniform(rng) - 1.0;
        const double sin_t = std::sqrt(1.0 - cos_t * cos_t);
        const double phi = 2.0 * 3.14159265358979323846 * uniform(rng);
        const double mass = (0.5 + uniform(rng)) / static_cast<double>(n);
        test::add_or_throw(
            store, BodyInit{"b" + std::to_string(i),
                            Star{},
                            space::Mass(mass),
                            {space::Length(r * sin_t * std::cos(phi)),
                             space::Length(r * sin_t * std::sin(phi)), space::Length(r * cos_t)},
                            {}});
    }
    return store;
}

std::vector<Vec3<space::Length>> positions_of(const BodyStore& store) {
    std::vector<Vec3<space::Length>> p(store.size());
    for (std::size_t i = 0; i < store.size(); ++i) {
        p[i] = store.position(i);
    }
    return p;
}

BarnesHutGravity barnes_hut(int threads, std::size_t leaf) {
    BarnesHutOptions options;
    options.opening_angle = 0.5;
    options.softening = space::Length(1e-3);
    options.max_threads = threads;
    options.leaf_capacity = leaf;
    const auto model = BarnesHutGravity::create(options);
    if (!model) {
        throw std::runtime_error("invalid Barnes-Hut options");
    }
    return model.value();
}

std::string label(std::size_t n) {
    return n >= 1000 ? std::to_string(n / 1000) + "k" : std::to_string(n);
}

// Median evaluation time of `model` on the cluster of n bodies.
std::vector<double> force_samples(const IForceModel& model, std::size_t n) {
    const BodyStore store = plummer_cluster(n, 777);
    const std::vector<Vec3<space::Length>> p = positions_of(store);
    std::vector<Vec3<space::Acceleration>> a(n);
    const int repeats = n >= 100000 ? 7 : 21;
    return orrery::bench::time_ms([&] { model.accelerations(store, p, a); }, 2, repeats);
}

void record_force(const std::string& id, const std::string& title,
                  const std::vector<double>& samples, Goal goal, double target,
                  const std::string& notes) {
    Measurement m;
    m.id = id;
    m.title = title;
    m.value = orrery::bench::median(samples);
    m.unit = "ms";
    m.goal = goal;
    m.target = target;
    m.notes = notes;
    m.context = {
        {"repeats", std::to_string(samples.size())},
        {"min_ms", orrery::bench::number(*std::min_element(samples.begin(), samples.end()))},
        {"hardware_threads", std::to_string(std::thread::hardware_concurrency())}};
    orrery::bench::record(m);
}

std::string hex(std::uint64_t value) {
    std::array<char, 24> buffer{};
    std::snprintf(buffer.data(), buffer.size(), "0x%016llx",
                  static_cast<unsigned long long>(value));
    return {buffer.data()};
}

} // namespace

TEST_CASE("Barnes-Hut force evaluation against the section 8 targets", "[bench][core][force]") {
    const std::string notes = "theta 0.5, softening 1e-3 AU, leaf 8, Plummer cluster, all threads";
    for (const auto& [n, target] :
         {std::pair{std::size_t{10000}, 8.0}, std::pair{std::size_t{100000}, 120.0}}) {
        const auto samples = force_samples(barnes_hut(0, 8), n);
        record_force(cat({"core.force.barnes_hut.n", label(n)}),
                     cat({"Force N = ", label(n), " (Barnes-Hut)"}), samples, Goal::below, target,
                     notes);
    }
}

TEST_CASE("Gravity evaluation over N = 100 .. 100k", "[bench][core][force]") {
    for (const std::size_t n :
         {std::size_t{100}, std::size_t{1000}, std::size_t{10000}, std::size_t{100000}}) {
        record_force(cat({"core.force.barnes_hut.single_thread.n", label(n)}),
                     cat({"Force N = ", label(n), " (Barnes-Hut, one thread)"}),
                     force_samples(barnes_hut(1, 8), n), Goal::info, 0.0,
                     "theta 0.5, leaf 8, one thread");
        if (n <= 10000) {
            const auto direct = NewtonianGravity::create(space::Length(1e-3));
            REQUIRE(direct);
            record_force(cat({"core.force.direct.n", label(n)}),
                         cat({"Force N = ", label(n), " (direct summation)"}),
                         force_samples(direct.value(), n), Goal::info, 0.0,
                         "pairwise, softening 1e-3 AU, one thread");
        }
    }
    for (const std::size_t leaf : {std::size_t{16}, std::size_t{32}}) {
        const std::string size = std::to_string(leaf);
        record_force(cat({"core.force.barnes_hut.leaf", size, ".n10k"}),
                     cat({"Force N = 10k (Barnes-Hut, leaf ", size, ")"}),
                     force_samples(barnes_hut(0, leaf), 10000), Goal::info, 0.0,
                     "theta 0.5, all threads");
    }
}

TEST_CASE("10000 golden steps are bit-identical and match the golden hashes",
          "[bench][core][determinism]") {
    struct Case {
        IntegratorKind kind;
        const char* name;
        std::uint64_t golden;
    };
    for (const Case& c : {Case{IntegratorKind::leapfrog_kdk, "leapfrog_kdk", golden_leapfrog},
                          Case{IntegratorKind::yoshida4, "yoshida4", golden_yoshida4},
                          Case{IntegratorKind::rk4, "rk4", golden_rk4}}) {
        std::vector<std::uint64_t> hashes;
        hashes.reserve(2);
        const auto samples = orrery::bench::time_ms(
            [&] { hashes.push_back(state_hash(test::golden_run(c.kind))); }, 0, 2);
        REQUIRE(hashes.size() == 2);
        const bool identical = hashes[0] == hashes[1] && hashes[0] == c.golden;
        CHECK(identical);
        Measurement m;
        m.id = cat({"core.determinism.", c.name});
        m.title = cat({"Determinism, 10k steps (", c.name, ")"});
        m.value = identical ? 1.0 : 0.0;
        m.unit = "bit-identical";
        m.goal = Goal::at_least;
        m.target = 1.0;
        m.notes = "two runs against the golden hash";
        m.context = {{"hash", hex(hashes[0])},
                     {"golden", hex(c.golden)},
                     {"run_ms", orrery::bench::number(orrery::bench::median(samples))}};
        orrery::bench::record(m);
    }
}
