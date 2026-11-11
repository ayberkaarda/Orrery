#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <random>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gravity_test_support.h"
#include "orrery/core/barnes_hut.h"
#include "orrery/core/conservation.h"
#include "orrery/core/force_model.h"
#include "orrery/core/octree.h"
#include "orrery/core/state_hash.h"

using namespace orrery::core;
using namespace orrery::core::test;

namespace {

// Uniform double in [0, 1) from the top 53 bits of mt19937_64, whose output
// sequence is fixed by the standard (std::uniform_real_distribution is not).
double uniform(std::mt19937_64& rng) {
    return static_cast<double>(rng() >> 11U) * 0x1p-53;
}

// Seeded Plummer sphere (scale radius 1 AU, truncated at 10 AU) with masses
// uniform in [0.5, 1.5] / n Msun and, every `probe_every` bodies, a probe.
// Directions are uniform on the sphere; velocities are zero (only forces are
// compared).
BodyStore plummer_cluster(std::size_t n, std::uint64_t seed, std::size_t probe_every = 0) {
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
        const bool probe = probe_every != 0 && i % probe_every == probe_every - 1;
        add_or_throw(store,
                     BodyInit{"b" + std::to_string(i),
                              probe ? BodyKind{Probe{}} : BodyKind{Star{}},
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

std::vector<Vec3<space::Acceleration>> evaluate(const IForceModel& model, const BodyStore& store) {
    const std::vector<Vec3<space::Length>> p = positions_of(store);
    std::vector<Vec3<space::Acceleration>> a(store.size());
    model.accelerations(store, p, a);
    return a;
}

BarnesHutGravity make_bh(double theta, double eps, int threads = 0, std::size_t leaf = 8) {
    BarnesHutOptions options;
    options.opening_angle = theta;
    options.softening = space::Length(eps);
    options.max_threads = threads;
    options.leaf_capacity = leaf;
    const auto model = BarnesHutGravity::create(options);
    if (!model) {
        throw std::runtime_error("invalid Barnes-Hut options");
    }
    return model.value();
}

NewtonianGravity make_direct(double eps) {
    const auto model = NewtonianGravity::create(space::Length(eps));
    if (!model) {
        throw std::runtime_error("invalid softening");
    }
    return model.value();
}

struct ErrorStats {
    double median = 0.0;
    double max = 0.0;
    double p99 = 0.0;
};

// Relative error |a - a_ref| / |a_ref| per body.
ErrorStats relative_errors(std::span<const Vec3<space::Acceleration>> a,
                           std::span<const Vec3<space::Acceleration>> ref) {
    std::vector<double> e(a.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        e[i] = norm(a[i] - ref[i]).value() / norm(ref[i]).value();
    }
    ErrorStats s;
    if (e.empty()) {
        return s;
    }
    std::sort(e.begin(), e.end());
    s.median = e[e.size() / 2];
    s.p99 = e[(e.size() * 99) / 100];
    s.max = e.back();
    return s;
}

bool bit_identical(std::span<const Vec3<space::Acceleration>> a,
                   std::span<const Vec3<space::Acceleration>> b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        for (const auto& [u, v] :
             {std::pair{a[i].x.value(), b[i].x.value()}, std::pair{a[i].y.value(), b[i].y.value()},
              std::pair{a[i].z.value(), b[i].z.value()}}) {
            if (std::bit_cast<std::uint64_t>(u) != std::bit_cast<std::uint64_t>(v)) {
                return false;
            }
        }
    }
    return true;
}

std::uint64_t hash_accelerations(std::span<const Vec3<space::Acceleration>> a) {
    std::uint64_t h = fnv1a_offset_basis;
    for (const Vec3<space::Acceleration>& v : a) {
        h = fnv1a_append(h, std::bit_cast<std::uint64_t>(v.x.value()));
        h = fnv1a_append(h, std::bit_cast<std::uint64_t>(v.y.value()));
        h = fnv1a_append(h, std::bit_cast<std::uint64_t>(v.z.value()));
    }
    return h;
}

std::string hex(std::uint64_t v) {
    std::array<char, 32> buffer{};
    std::snprintf(buffer.data(), buffer.size(), "0x%016llx", static_cast<unsigned long long>(v));
    return buffer.data();
}

} // namespace

TEST_CASE("Barnes-Hut option validation", "[core][barnes_hut]") {
    BarnesHutOptions o;
    CHECK(BarnesHutGravity::create(o));
    o.opening_angle = 1.5;
    CHECK(BarnesHutGravity::create(o).error() == BarnesHutError::invalid_opening_angle);
    o.opening_angle = -0.1;
    CHECK(BarnesHutGravity::create(o).error() == BarnesHutError::invalid_opening_angle);
    o.opening_angle = std::nan("");
    CHECK(BarnesHutGravity::create(o).error() == BarnesHutError::invalid_opening_angle);
    o = BarnesHutOptions{};
    o.softening = space::Length(-1.0);
    CHECK(BarnesHutGravity::create(o).error() == BarnesHutError::invalid_softening);
    o = BarnesHutOptions{};
    o.leaf_capacity = 0;
    CHECK(BarnesHutGravity::create(o).error() == BarnesHutError::invalid_leaf_capacity);
    o = BarnesHutOptions{};
    o.max_threads = -1;
    CHECK(BarnesHutGravity::create(o).error() == BarnesHutError::invalid_thread_count);
    CHECK(make_bh(0.5, 0.0).name() == "barnes_hut");
}

TEST_CASE("Morton keys interleave x, y, z bits", "[core][barnes_hut]") {
    CHECK(Octree::morton_key(1, 0, 0) == 1U);
    CHECK(Octree::morton_key(0, 1, 0) == 2U);
    CHECK(Octree::morton_key(0, 0, 1) == 4U);
    CHECK(Octree::morton_key(2, 0, 0) == 8U);
    CHECK(Octree::morton_key(0x1fffff, 0x1fffff, 0x1fffff) == (std::uint64_t{1} << 63U) - 1U);
}

TEST_CASE("octree moments and sorted order", "[core][barnes_hut]") {
    const BodyStore store = plummer_cluster(500, 7);
    const std::vector<Vec3<space::Length>> p = positions_of(store);
    std::vector<double> gm(store.size());
    std::vector<std::uint32_t> sources(store.size());
    double total = 0.0;
    for (std::size_t i = 0; i < store.size(); ++i) {
        gm[i] = store.mass(i).value();
        sources[i] = static_cast<std::uint32_t>(i);
        total += gm[i];
    }
    Octree tree;
    REQUIRE(tree.build(p, gm, sources, 4));
    const std::span<const Octree::Node> nodes = tree.nodes();
    REQUIRE(!nodes.empty());
    CHECK(std::is_sorted(tree.keys().begin(), tree.keys().end()));
    CHECK(nodes[0].count == store.size());
    CHECK(nodes[0].next == nodes.size());
    CHECK(std::abs(nodes[0].gm - total) <= 1e-14 * total);
    // Every leaf holds at most 4 sources; leaves tile the sorted range.
    std::uint32_t covered = 0;
    for (const Octree::Node& node : nodes) {
        if (node.leaf) {
            CHECK(node.first == covered);
            CHECK(node.count <= 4U);
            covered += node.count;
        }
    }
    CHECK(covered == store.size());
    // Non-finite input is rejected.
    std::vector<Vec3<space::Length>> bad = p;
    bad[3].y = space::Length(std::nan(""));
    CHECK_FALSE(tree.build(bad, gm, sources, 4));
    CHECK(tree.empty());
}

TEST_CASE("Barnes-Hut accuracy against direct summation, N = 2000", "[core][barnes_hut]") {
    // Acceptance (Section 8): median relative acceleration error < 1e-3 at
    // theta = 0.5, N = 2000, seeded cluster.
    const BodyStore store = plummer_cluster(2000, 20260101);
    const double eps = 1.0e-3;
    const std::vector<Vec3<space::Acceleration>> ref = evaluate(make_direct(eps), store);

    const std::vector<Vec3<space::Acceleration>> bh = evaluate(make_bh(0.5, eps), store);
    const ErrorStats s = relative_errors(bh, ref);
    report("barnes_hut N=2000 theta=0.5 eps=1e-3 Plummer: median rel error " + sci(s.median) +
           ", p99 " + sci(s.p99) + ", max " + sci(s.max));
    CHECK(s.median < 1.0e-3);
    CHECK(s.max < 2.0e-2);

    for (const double theta : {0.3, 0.7, 1.0}) {
        const ErrorStats t = relative_errors(evaluate(make_bh(theta, eps), store), ref);
        report("barnes_hut N=2000 theta=" + std::to_string(theta).substr(0, 3) + ": median " +
               sci(t.median) + ", max " + sci(t.max));
        if (theta < 0.5) {
            CHECK(t.median < s.median);
        }
    }

    // theta = 0 opens every node: direct summation in tree order.
    const ErrorStats exact = relative_errors(evaluate(make_bh(0.0, eps), store), ref);
    report("barnes_hut N=2000 theta=0: max rel error vs direct " + sci(exact.max));
    CHECK(exact.max < 1.0e-12);

    // Potential energy from the same tree walk.
    const std::vector<Vec3<space::Length>> p = positions_of(store);
    const double u_ref = make_direct(eps).potential_energy(store, p).value();
    const double u_bh = make_bh(0.5, eps).potential_energy(store, p).value();
    const double u_exact = make_bh(0.0, eps).potential_energy(store, p).value();
    report("barnes_hut N=2000 theta=0.5: potential energy rel error " +
           sci(std::abs(u_bh - u_ref) / std::abs(u_ref)));
    CHECK(std::abs(u_bh - u_ref) < 1.0e-3 * std::abs(u_ref));
    CHECK(std::abs(u_exact - u_ref) < 1.0e-12 * std::abs(u_ref));
}

TEST_CASE("Barnes-Hut softening matches NewtonianGravity", "[core][barnes_hut]") {
    // Coincident pair with eps > 0 and a distant third body; theta = 0.5 is
    // exact for two-body leaves.
    BodyStore store;
    add_or_throw(store, BodyInit{"A", Star{}, space::Mass(1.0), {}, {}});
    add_or_throw(store, BodyInit{"B", Star{}, space::Mass(0.5), {}, {}});
    add_or_throw(store, BodyInit{"C",
                                 Planet{},
                                 space::Mass(1e-3),
                                 {space::Length(0.01), space::Length(0.0), space::Length(0.0)},
                                 {}});
    const std::vector<Vec3<space::Acceleration>> ref = evaluate(make_direct(0.05), store);
    const std::vector<Vec3<space::Acceleration>> bh = evaluate(make_bh(0.5, 0.05), store);
    for (std::size_t i = 0; i < store.size(); ++i) {
        CHECK(norm(bh[i] - ref[i]).value() <= 1e-14 * norm(ref[i]).value() + 1e-300);
    }
}

TEST_CASE("Barnes-Hut probes feel the field but exert none", "[core][barnes_hut]") {
    const BodyStore with_probes = plummer_cluster(1200, 99, 10);
    BodyStore without_probes;
    std::vector<std::size_t> kept;
    for (std::size_t i = 0; i < with_probes.size(); ++i) {
        if (with_probes.exerts_gravity(i)) {
            add_or_throw(without_probes,
                         BodyInit{with_probes.name(i), with_probes.kind(i), with_probes.mass(i),
                                  with_probes.position(i), with_probes.velocity(i)});
            kept.push_back(i);
        }
    }
    const BarnesHutGravity bh = make_bh(0.5, 1e-3);
    const std::vector<Vec3<space::Acceleration>> a = evaluate(bh, with_probes);
    const std::vector<Vec3<space::Acceleration>> b = evaluate(bh, without_probes);
    std::vector<Vec3<space::Acceleration>> a_sources;
    a_sources.reserve(kept.size());
    for (const std::size_t i : kept) {
        a_sources.push_back(a[i]);
    }
    CHECK(bit_identical(a_sources, b));
    // The probes' accelerations agree with direct summation.
    const std::vector<Vec3<space::Acceleration>> ref = evaluate(make_direct(1e-3), with_probes);
    const ErrorStats s = relative_errors(a, ref);
    CHECK(s.median < 1.0e-3);
}

TEST_CASE("Barnes-Hut is bit-identical for any thread count", "[core][barnes_hut][determinism]") {
    const BodyStore store = plummer_cluster(20000, 424242, 50);
    const unsigned hw = std::max(1U, std::thread::hardware_concurrency());
    const std::vector<Vec3<space::Acceleration>> one = evaluate(make_bh(0.5, 1e-3, 1), store);
    const std::uint64_t h1 = hash_accelerations(one);
    for (const int threads : {2, 3, 4, 8, static_cast<int>(hw), 0}) {
        const std::vector<Vec3<space::Acceleration>> many =
            evaluate(make_bh(0.5, 1e-3, threads), store);
        INFO("threads " << threads);
        CHECK(bit_identical(one, many));
    }
    report("barnes_hut N=20000 acceleration hash (1 thread and " + std::to_string(hw) +
           " threads): " + hex(h1));

    // Short leapfrog run: same final state for 1 thread and all threads.
    const auto run = [&](int threads) {
        SimState state(space::Time(0.0), plummer_cluster(3000, 5));
        const BarnesHutGravity bh = make_bh(0.5, 1e-2, threads);
        LeapfrogKdk leapfrog;
        advance(leapfrog, state, space::Time(0.5), 20, bh);
        return state_hash(state);
    };
    const std::uint64_t s1 = run(1);
    const std::uint64_t sn = run(0);
    report("barnes_hut N=3000 leapfrog 20 steps state hash, 1 thread " + hex(s1) +
           ", default threads " + hex(sn));
    CHECK(s1 == sn);
}

TEST_CASE("Barnes-Hut golden hash after 10000 steps",
          "[core][barnes_hut][determinism][acceptance]") {
    // Golden scenario of the direct-force goldens (literal hexadecimal initial
    // state of Sun, 8 planets and a probe), Barnes-Hut with theta = 0.5,
    // softening 1e-4 AU and leaf capacity 1 (so most pairs go through the
    // quadrupole multipoles), 10000 Leapfrog KDK steps of 1 day. One thread
    // and every thread count must reproduce the same pinned value.
    constexpr std::uint64_t golden_barnes_hut = 0x904687bcd8a69556ULL;
    const auto run = [](int threads) {
        SimState state = golden_initial_state();
        const BarnesHutGravity bh = make_bh(0.5, 1.0e-4, threads, 1);
        LeapfrogKdk leapfrog;
        advance(leapfrog, state, space::Time(1.0), 10000, bh);
        return state_hash(state);
    };
    const std::uint64_t single = run(1);
    report("barnes_hut golden hash (1 thread) " + hex(single));
    CHECK(single == golden_barnes_hut);
    const unsigned hw = std::max(1U, std::thread::hardware_concurrency());
    for (const int threads : {2, 4, static_cast<int>(hw), 0}) {
        INFO("threads " << threads);
        CHECK(run(threads) == golden_barnes_hut);
    }
}

TEST_CASE("Barnes-Hut momentum drift (observation)", "[core][barnes_hut]") {
    // The tree force is not pairwise symmetric, so momentum is conserved only
    // to the accuracy of the force. Measured and bounded loosely.
    SimState state(space::Time(0.0), plummer_cluster(2000, 31337));
    const double eps = 1e-2;
    const BarnesHutGravity bh = make_bh(0.5, eps);
    const ConservationDiagnostics diag(state, make_direct(eps));
    LeapfrogKdk leapfrog;
    advance(leapfrog, state, space::Time(0.5), 50, bh);
    // The cluster starts at rest (P0 = 0, sum m |v| = 0), so the momentum
    // error is measured against sum m |v| of the final state.
    const ConservedQuantities q = ConservationDiagnostics::measure(state, make_direct(eps));
    const double p_rel = norm(q.linear_momentum).value() / q.momentum_scale.value();
    const ConservationDrift d = diag.drift(state, make_direct(eps));
    report("barnes_hut N=2000 theta=0.5 leapfrog 50 x 0.5 d: |P| / sum m|v| " + sci(p_rel) +
           ", energy drift (direct U) " + sci(d.relative_energy));
    CHECK(p_rel < 1.0e-3);
}

TEST_CASE("Barnes-Hut force timing", "[.benchmark][barnes_hut]") {
    // Section 8 target: N = 10k < 8 ms, N = 100k < 120 ms on 8 cores.
    const unsigned hw = std::thread::hardware_concurrency();
    for (const std::size_t n : {std::size_t{10000}, std::size_t{100000}}) {
        const BodyStore store = plummer_cluster(n, 777);
        const std::vector<Vec3<space::Length>> p = positions_of(store);
        std::vector<Vec3<space::Acceleration>> a(n);
        for (const auto& [threads, leaf] :
             {std::pair{1, std::size_t{16}}, std::pair{0, std::size_t{8}},
              std::pair{0, std::size_t{16}}, std::pair{0, std::size_t{32}}}) {
            const BarnesHutGravity bh = make_bh(0.5, 1e-3, threads, leaf);
            bh.accelerations(store, p, a); // warm-up
            std::vector<double> ms;
            for (int rep = 0; rep < 7; ++rep) {
                const auto t0 = std::chrono::steady_clock::now();
                bh.accelerations(store, p, a);
                const auto t1 = std::chrono::steady_clock::now();
                ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
            }
            std::sort(ms.begin(), ms.end());
            report("barnes_hut timing N=" + std::to_string(n) +
                   " theta=0.5 leaf=" + std::to_string(leaf) + " threads=" +
                   (threads == 0 ? std::to_string(hw) + " (all)" : std::to_string(threads)) +
                   ": median " + std::to_string(ms[ms.size() / 2]) + " ms, min " +
                   std::to_string(ms.front()) + " ms");
        }
    }
    SUCCEED("observation only");
}
