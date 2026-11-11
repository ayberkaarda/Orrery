#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <variant>

#include "orrery/core/body.h"
#include "orrery/core/body_store.h"
#include "orrery/scene/scenario.h"
#include "scene_test_support.h"

namespace core = orrery::core;
namespace scene = orrery::scene;
using scene::test::relative_error;
using scene::test::same_bits;
using scene::test::scenario_path;
using scene::test::worst_error;

namespace {

constexpr double tolerance = 1e-12;

// Calls compare(expected, actual) for every number of the two stores; also
// checks that names, kinds and order agree.
template <typename F>
void for_each_number(const core::BodyStore& expected, const core::BodyStore& actual, F&& compare) {
    REQUIRE(expected.size() == actual.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        CHECK(expected.name(i) == actual.name(i));
        CHECK(expected.kind(i) == actual.kind(i));
        compare(expected.mass(i).value(), actual.mass(i).value());
        const auto p0 = expected.position(i);
        const auto p1 = actual.position(i);
        const auto v0 = expected.velocity(i);
        const auto v1 = actual.velocity(i);
        compare(p0.x.value(), p1.x.value());
        compare(p0.y.value(), p1.y.value());
        compare(p0.z.value(), p1.z.value());
        compare(v0.x.value(), v1.x.value());
        compare(v0.y.value(), v1.y.value());
        compare(v0.z.value(), v1.z.value());
    }
}

// Largest relative error over every number of the two stores; positive
// infinity if any pair involves a non-finite number.
double max_state_error(const core::BodyStore& expected, const core::BodyStore& actual) {
    double worst = 0.0;
    for_each_number(expected, actual,
                    [&worst](double a, double b) { worst = worst_error(worst, a, b); });
    return worst;
}

// Number of pairs whose bit patterns differ (distinguishes signed zeros).
std::size_t bit_mismatches(const core::BodyStore& expected, const core::BodyStore& actual) {
    std::size_t count = 0;
    for_each_number(expected, actual, [&count](double a, double b) {
        if (!same_bits(a, b)) {
            ++count;
        }
    });
    return count;
}

scene::Scene round_trip(const scene::Scene& original) {
    const auto saved = scene::save_scene(original);
    REQUIRE(saved.has_value());
    auto loaded = scene::load_scene(saved.value());
    REQUIRE(loaded.has_value());
    return std::move(loaded).value();
}

// Uniform in log-magnitude over many decades, random sign.
double wide_random(std::mt19937_64& rng) {
    std::uniform_real_distribution<double> exponent(-30.0, 30.0);
    std::uniform_real_distribution<double> mantissa(1.0, 10.0);
    std::bernoulli_distribution negative(0.5);
    const double v = mantissa(rng) * std::pow(10.0, exponent(rng));
    return negative(rng) ? -v : v;
}

} // namespace

TEST_CASE("bundled scenarios round-trip through save and load", "[scene][roundtrip]") {
    const std::string file = GENERATE("sun_earth.json", "sun_earth_moon.json");
    CAPTURE(file);
    const auto loaded = scene::load_scene_file(scenario_path(file));
    REQUIRE(loaded.has_value());
    const scene::Scene once = round_trip(loaded.value());
    CHECK(once.name == loaded.value().name);
    CHECK(once.metadata == loaded.value().metadata);
    CHECK(max_state_error(loaded.value().state.bodies(), once.state.bodies()) < tolerance);
    const scene::Scene twice = round_trip(once);
    CHECK(max_state_error(once.state.bodies(), twice.state.bodies()) < tolerance);
}

TEST_CASE("Keplerian scenarios round-trip at the element level", "[scene][roundtrip]") {
    const auto text = scene::read_text_file(scenario_path("sun_earth_moon.json"));
    REQUIRE(text.has_value());
    const auto parsed = scene::parse_scenario(text.value());
    REQUIRE(parsed.has_value());
    // Degrees are kept on save; the conversion costs at most a few ulp.
    const auto reparsed = scene::parse_scenario(scene::save_scenario(parsed.value()));
    REQUIRE(reparsed.has_value());
    REQUIRE(reparsed.value().bodies.size() == parsed.value().bodies.size());
    for (std::size_t i = 0; i < parsed.value().bodies.size(); ++i) {
        const auto* a = std::get_if<scene::KeplerianSpec>(&parsed.value().bodies[i].orbit);
        const auto* b = std::get_if<scene::KeplerianSpec>(&reparsed.value().bodies[i].orbit);
        REQUIRE((a == nullptr) == (b == nullptr));
        if (a == nullptr) {
            continue;
        }
        CHECK(a->parent == b->parent);
        CHECK(a->elements.semi_major_axis == b->elements.semi_major_axis);
        CHECK(a->elements.eccentricity == b->elements.eccentricity);
        CHECK(relative_error(a->elements.inclination.radians(), b->elements.inclination.radians()) <
              tolerance);
        CHECK(relative_error(a->elements.longitude_of_ascending_node.radians(),
                             b->elements.longitude_of_ascending_node.radians()) < tolerance);
        CHECK(relative_error(a->elements.argument_of_periapsis.radians(),
                             b->elements.argument_of_periapsis.radians()) < tolerance);
        CHECK(relative_error(a->elements.mean_anomaly.radians(),
                             b->elements.mean_anomaly.radians()) < tolerance);
    }
    const auto first = scene::build_scene(parsed.value());
    const auto second = scene::build_scene(reparsed.value());
    REQUIRE(first.has_value());
    REQUIRE(second.has_value());
    CHECK(max_state_error(first.value().state.bodies(), second.value().state.bodies()) < tolerance);
}

TEST_CASE("random states round-trip exactly", "[scene][roundtrip][property]") {
    const auto seed = GENERATE(1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u);
    std::mt19937_64 rng(seed);
    scene::Scene original;
    original.name = "random";
    original.metadata = {"generated", "t = 0", "property test"};
    original.state.set_time(core::space::Time(wide_random(rng)));
    original.aero_tables = {"table-a", "table-b"};
    for (int i = 0; i < 18; ++i) {
        const auto pick = static_cast<std::size_t>(i % 6);
        core::BodyKind kind = core::Planet{};
        switch (pick) {
            case 0:
                kind = core::Star{};
                break;
            case 1:
                kind = core::Planet{};
                break;
            case 2:
                kind = core::Moon{};
                break;
            case 3:
                kind = core::Asteroid{};
                break;
            case 4:
                kind = core::BlackHole{std::uniform_real_distribution<double>(0.0, 1.0)(rng)};
                break;
            default:
                // Tables in first-use order, as the loader numbers them.
                kind = core::Probe{core::AeroRef{i == 5 ? 1U : (i == 11 ? 2U : 0U)}};
                break;
        }
        const double mass = std::abs(wide_random(rng));
        const auto added = original.state.bodies().add(core::BodyInit{
            "body-" + std::to_string(i),
            kind,
            core::space::Mass(mass),
            {core::space::Length(wide_random(rng)), core::space::Length(wide_random(rng)),
             core::space::Length(wide_random(rng))},
            {core::space::Velocity(wide_random(rng)), core::space::Velocity(wide_random(rng)),
             core::space::Velocity(wide_random(rng))}});
        REQUIRE(added.has_value());
    }
    const scene::Scene copy = round_trip(original);
    // Shortest round-trip formatting reproduces every double bit for bit.
    CHECK(same_bits(copy.state.time().value(), original.state.time().value()));
    CHECK(bit_mismatches(original.state.bodies(), copy.state.bodies()) == 0);
    CHECK(max_state_error(original.state.bodies(), copy.state.bodies()) == 0.0);
}

TEST_CASE("signed zeros round-trip bit for bit", "[scene][roundtrip]") {
    scene::Scene original;
    original.metadata = {"test", "t = 0", ""};
    original.state.set_time(core::space::Time(-0.0));
    REQUIRE(original.state.bodies()
                .add(core::BodyInit{"Sun",
                                    core::Star{},
                                    core::space::Mass(1.0),
                                    {core::space::Length(-0.0), core::space::Length(0.0),
                                     core::space::Length(-0.0)},
                                    {core::space::Velocity(0.0), core::space::Velocity(-0.0),
                                     core::space::Velocity(-0.0)}})
                .has_value());
    const scene::Scene copy = round_trip(original);
    CHECK(same_bits(copy.state.time().value(), -0.0));
    CHECK(bit_mismatches(original.state.bodies(), copy.state.bodies()) == 0);
}

TEST_CASE("the round-trip oracle never hides a non-finite mismatch", "[scene][roundtrip]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    CHECK(relative_error(1.0, nan) == inf);
    CHECK(relative_error(1.0, inf) == inf);
    CHECK(relative_error(nan, 1.0) == inf);
    CHECK(relative_error(-inf, 1.0) == inf);
    CHECK(relative_error(inf, inf) == inf);
    CHECK(relative_error(1.0, 1.0) == 0.0);
    CHECK(relative_error(0.0, -0.0) == 0.0);
    CHECK(relative_error(2.0, 1.0) == 0.5);

    // A non-finite pair among agreeing pairs dominates the aggregate, so
    // neither "< tolerance" nor "== 0" can pass.
    double worst = worst_error(0.0, 1.0, 1.0);
    worst = worst_error(worst, 1.0, nan);
    worst = worst_error(worst, 3.0, 3.0);
    CHECK(worst == inf);
    CHECK_FALSE(worst < tolerance);
    CHECK(worst_error(worst_error(0.0, nan, 1.0), 2.0, 2.0) == inf);
    CHECK(worst_error(worst_error(0.0, 1.0, inf), 2.0, 2.0) == inf);

    // Bit patterns tell signed zeros apart where == does not.
    CHECK(0.0 == -0.0);
    CHECK_FALSE(same_bits(0.0, -0.0));
    CHECK(same_bits(-0.0, -0.0));
    CHECK(same_bits(1.5, 1.5));
    CHECK_FALSE(same_bits(1.0, std::nextafter(1.0, 2.0)));
}

TEST_CASE("a probe whose aero_ref has no table cannot be saved", "[scene][roundtrip]") {
    scene::Scene s;
    s.metadata = {"test", "t = 0", ""};
    REQUIRE(s.state.bodies()
                .add(core::BodyInit{
                    "probe", core::Probe{core::AeroRef{3}}, core::space::Mass(1e-27), {}, {}})
                .has_value());
    const auto saved = scene::save_scene(s);
    REQUIRE_FALSE(saved.has_value());
    CHECK(saved.error().path == "bodies[0].aero_ref");
}
