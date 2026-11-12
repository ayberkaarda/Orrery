#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>

#include "orrery/core/sim_snapshot.h"
#include "orrery/render/trails.h"

using namespace orrery;
using namespace orrery::render;
using Catch::Approx;

namespace {

constexpr double kAu = 149597870700.0;

core::BodyInit body(const char* name, double mass, double x_au, double y_au) {
    return {name,
            core::Planet{},
            core::space::Mass(mass),
            {core::space::Length(x_au), core::space::Length(y_au), core::space::Length(0.0)},
            {}};
}

} // namespace

TEST_CASE("A trail ring keeps the newest points in order", "[render][trails]") {
    TrailRing ring(4);
    CHECK(ring.empty());
    for (int i = 0; i < 6; ++i) {
        ring.push(glm::dvec3(i, 0.0, 0.0));
    }
    REQUIRE(ring.size() == 4);
    CHECK(ring.at(0).x == 2.0);
    CHECK(ring.at(3).x == 5.0);
    CHECK(ring.newest().x == 5.0);
    CHECK_THROWS_AS(ring.at(4), std::out_of_range);

    // Shrinking keeps the newest points; growing keeps all of them.
    ring.set_capacity(2);
    REQUIRE(ring.size() == 2);
    CHECK(ring.at(0).x == 4.0);
    CHECK(ring.at(1).x == 5.0);
    ring.set_capacity(8);
    CHECK(ring.size() == 2);
    ring.push(glm::dvec3(6.0, 0.0, 0.0));
    CHECK(ring.newest().x == 6.0);
    ring.clear();
    CHECK(ring.empty());

    TrailRing none(0);
    none.push(glm::dvec3(1.0));
    CHECK(none.empty());
}

TEST_CASE("Trail history follows bodies by id across snapshots", "[render][trails]") {
    core::BodyStore bodies;
    REQUIRE(bodies.add(body("Sun", 1.0, 0.0, 0.0)));
    REQUIRE(bodies.add(body("A", 1e-6, 1.0, 0.0)));
    REQUIRE(bodies.add(body("B", 1e-6, 2.0, 0.0)));
    core::SimState state(core::space::Time(0.0), std::move(bodies));

    TrailHistory history(TrailSettings{3, 0.0});
    for (int k = 0; k < 5; ++k) {
        REQUIRE(state.bodies().set_position(
            1, {core::space::Length(1.0), core::space::Length(0.1 * k), core::space::Length(0.0)}));
        history.record(*core::SimSnapshot::capture(state));
    }
    REQUIRE(history.trails().size() == 3);
    const TrailRing* a = history.find(state.bodies().id(1));
    REQUIRE(a != nullptr);
    REQUIRE(a->size() == 3);
    CHECK(a->newest().y == Approx(0.4 * kAu));
    CHECK(a->at(0).y == Approx(0.2 * kAu));

    // B is removed (merged): its trail goes; a new body starts an empty trail.
    const core::BodyId b = state.bodies().id(2);
    REQUIRE(state.bodies().remove(b));
    REQUIRE(state.bodies().add(body("C", 1e-6, 3.0, 0.0)));
    history.record(*core::SimSnapshot::capture(state));
    CHECK(history.find(b) == nullptr);
    const TrailRing* c = history.find(state.bodies().id(2));
    REQUIRE(c != nullptr);
    CHECK(c->size() == 1);
    CHECK(history.trails().size() == 3);

    // A new length applies to existing trails.
    history.set_settings(TrailSettings{2, 0.0});
    CHECK(history.find(state.bodies().id(1))->size() == 2);
    history.clear();
    CHECK(history.trails().empty());
}

TEST_CASE("Angular spacing gives every orbit the same angular coverage", "[render][trails]") {
    // Two circular orbits around a heavy body, sampled 200 times per inner
    // period: with a minimum step of 0.05 rad both trails keep points 0.05 rad
    // (or one sample) apart, so 40 points span about 2 rad of each orbit.
    core::BodyStore bodies;
    REQUIRE(bodies.add(body("Sun", 1.0, 0.0, 0.0)));
    REQUIRE(bodies.add(body("Inner", 1e-6, 1.0, 0.0)));
    REQUIRE(bodies.add(body("Outer", 1e-6, 4.0, 0.0)));
    core::SimState state(core::space::Time(0.0), std::move(bodies));
    TrailHistory history(TrailSettings{40, 0.05});
    const double two_pi = 6.283185307179586;
    for (int k = 0; k <= 4000; ++k) {
        const double inner = two_pi * k / 200.0;
        const double outer = inner / 8.0; // Kepler: period ratio 8 at radius ratio 4
        REQUIRE(state.bodies().set_position(1, {core::space::Length(std::cos(inner)),
                                                core::space::Length(std::sin(inner)),
                                                core::space::Length(0.0)}));
        REQUIRE(state.bodies().set_position(2, {core::space::Length(4.0 * std::cos(outer)),
                                                core::space::Length(4.0 * std::sin(outer)),
                                                core::space::Length(0.0)}));
        history.record(*core::SimSnapshot::capture(state));
    }
    for (std::size_t index : {std::size_t{1}, std::size_t{2}}) {
        const TrailRing* ring = history.find(state.bodies().id(index));
        REQUIRE(ring != nullptr);
        REQUIRE(ring->size() == 40);
        const double start = std::atan2(ring->at(0).y, ring->at(0).x);
        const double end = std::atan2(ring->newest().y, ring->newest().x);
        double span = end - start;
        while (span < 0.0) {
            span += two_pi;
        }
        INFO("body " << index);
        CHECK(span > 1.9);
        CHECK(span < 2.6);
    }
}

TEST_CASE("Trail segments are camera-relative, float-safe and fade with age", "[render][trails]") {
    // Points 1 km apart, 1e4 AU from the origin, seen from 10 km away.
    const glm::dvec3 base{1.0e4 * kAu, 0.0, 0.0};
    TrailRing ring(4);
    for (int i = 0; i < 4; ++i) {
        ring.push(base + glm::dvec3(0.0, 1000.0 * i, 0.0));
    }
    Camera camera;
    OrbitMode orbit;
    orbit.focus = base;
    orbit.distance = 1.0e4;
    orbit.pitch = 0.0;
    camera.set_mode(orbit);
    camera.update(0.0, CameraInput{}, BodyFrameLookup{});

    const glm::dvec3 head = base + glm::dvec3(0.0, 3500.0, 0.0);
    std::vector<TrailSegment> segments;
    append_trail_segments(ring, head, camera, segments);
    REQUIRE(segments.size() == 4);
    // Consecutive segments share their endpoints.
    for (std::size_t i = 0; i + 1 < segments.size(); ++i) {
        CHECK(segments[i].p1 == segments[i + 1].p0);
        CHECK(segments[i].fade1 == segments[i + 1].fade0);
    }
    // Fades: oldest point age 4 of capacity 4 -> 0; head -> 1.
    CHECK(segments.front().fade0 == 0.0F);
    CHECK(segments.back().fade1 == 1.0F);
    CHECK(segments[1].fade0 == Approx(0.25));
    // Every endpoint matches the double-precision camera-relative position.
    for (int i = 0; i < 4; ++i) {
        const glm::dvec3 expected = camera.to_render_double(ring.at(static_cast<std::size_t>(i)));
        const glm::dvec3 got(segments[static_cast<std::size_t>(i)].p0);
        CHECK(glm::length(got - expected) < 1.0e-3); // [m], float rounding at 10 km
    }
    const glm::dvec3 head_render(segments.back().p1);
    CHECK(glm::length(head_render - camera.to_render_double(head)) < 1.0e-3);
    // Converting world positions to float first would have lost the 1 km spacing.
    CHECK(static_cast<float>(base.x) == static_cast<float>(base.x + 1000.0));
}

TEST_CASE("Trail segments skip duplicate world endpoints", "[render][trails]") {
    const glm::dvec3 p{2.0e5, 1.0e5, -1.0e6};
    const glm::dvec3 q = p + glm::dvec3(3.0e4, 0.0, 0.0);
    Camera camera;
    camera.set_mode(FreeMode{});
    camera.update(0.0, CameraInput{}, BodyFrameLookup{});
    std::vector<TrailSegment> segments;

    SECTION("empty ring") {
        const TrailRing ring(512);
        append_trail_segments(ring, p, camera, segments);
        CHECK(segments.empty());
    }
    SECTION("one stationary sample") {
        TrailRing ring(512);
        ring.push(p);
        append_trail_segments(ring, p, camera, segments);
        CHECK(segments.empty());
    }
    SECTION("hundreds of stationary duplicates") {
        TrailRing ring(512);
        for (int i = 0; i < 512; ++i) {
            ring.push(p);
        }
        append_trail_segments(ring, p, camera, segments);
        CHECK(segments.empty());
    }
    SECTION("duplicates before a move give one segment") {
        TrailRing ring(512);
        for (int i = 0; i < 300; ++i) {
            ring.push(p);
        }
        append_trail_segments(ring, q, camera, segments);
        REQUIRE(segments.size() == 1);
        CHECK(glm::length(glm::dvec3(segments[0].p0) - p) < 1.0e-6 * glm::length(p));
        CHECK(glm::length(glm::dvec3(segments[0].p1) - q) < 1.0e-6 * glm::length(q));
        // A run of equal points keeps the fade of its first point, so the
        // fade stays continuous along the polyline.
        CHECK(segments[0].fade0 == Approx(1.0 - 300.0 / 512.0));
        CHECK(segments[0].fade1 == 1.0F);
    }
    SECTION("repeated samples inside a moving trail are merged") {
        TrailRing ring(8);
        for (const glm::dvec3& point : {p, p, p, q, q}) {
            ring.push(point);
        }
        const glm::dvec3 r = q + glm::dvec3(0.0, 3.0e4, 0.0);
        append_trail_segments(ring, r, camera, segments);
        REQUIRE(segments.size() == 2);
        CHECK(segments[0].p1 == segments[1].p0);
        CHECK(segments[0].fade1 == segments[1].fade0);
        CHECK(segments[1].fade0 == Approx(1.0 - 2.0 / 8.0));
    }
    SECTION("a moving one-sample trail gives one segment") {
        TrailRing ring(512);
        ring.push(q);
        append_trail_segments(ring, p, camera, segments);
        REQUIRE(segments.size() == 1);
        CHECK(std::isfinite(segments[0].p0.x));
        CHECK(std::isfinite(segments[0].fade0));
    }
}
