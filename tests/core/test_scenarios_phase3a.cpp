// Phase 3a embedded scenarios: J2000 solar system (JPL Horizons), the
// figure-eight three-body choreography, and Sgr A* + S2. Loaded from
// assets/scenarios through orrery_scene and integrated with orrery_core.

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include "gravity_test_support.h"
#include "orrery/core/conservation.h"
#include "orrery/core/ias15.h"
#include "orrery/core/integrator.h"
#include "orrery/scene/scenario.h"

using namespace orrery::core;
using namespace orrery::core::test;
namespace scene = orrery::scene;

namespace {

scene::Scene load(const char* file) {
    const auto loaded =
        orrery::scene::load_scene_file(std::filesystem::path(ORRERY_SCENARIO_DIR) / file);
    if (!loaded) {
        throw std::runtime_error(std::string("cannot load scenario ") + file);
    }
    return loaded.value();
}

std::size_t index_of(const BodyStore& store, const std::string& name) {
    for (std::size_t i = 0; i < store.size(); ++i) {
        if (store.name(i) == name) {
            return i;
        }
    }
    throw std::runtime_error("no body named " + name);
}

constexpr double gauss_k = 0.01720209895;
// Figure-eight period in G = m = 1 units (Chenciner and Montgomery 2000,
// value computed by C. Simo) and in days after the scenario's scaling.
constexpr double figure_eight_period_scaled = 6.32591398;
const double figure_eight_period = figure_eight_period_scaled / gauss_k;

double config_distance2(const SimState& a, const SimState& b) {
    double d2 = 0.0;
    for (std::size_t i = 0; i < a.bodies().size(); ++i) {
        const Vec3<space::Length> d = a.bodies().position(i) - b.bodies().position(i);
        d2 += d.x.value() * d.x.value() + d.y.value() * d.y.value() + d.z.value() * d.z.value();
    }
    return d2;
}

struct FigureEightRun {
    double max_energy = 0.0;
    double max_momentum = 0.0;
    double max_angular = 0.0;
    std::vector<double> return_times; // time of the k-th return, k = 1..K
    long long force_evaluations = 0;
};

// Integrates `periods` periods. Around each k T (literature T) the state is
// sampled at k T + j delta, j = -2..2, delta = T / 2000; the return time is the
// vertex of the parabola through the three samples of |X(t) - X(0)|^2 around
// its minimum. `advance_to(state, dt)` advances by dt.
template <typename Advance>
FigureEightRun run_figure_eight(int periods, const Advance& advance_by) {
    const scene::Scene sc = load("figure_eight.json");
    SimState state = sc.state;
    const SimState initial = state;
    const NewtonianGravity gravity;
    const ConservationDiagnostics diag(initial, gravity);
    const double period = figure_eight_period;
    const double delta = period / 2000.0;
    FigureEightRun run;
    RunningMax de;
    RunningMax dp;
    RunningMax dl;
    const auto sample = [&] {
        const ConservationDrift d = diag.drift(state, gravity);
        de.add(d.relative_energy);
        dp.add(d.linear_momentum);
        dl.add(d.angular_momentum);
    };
    double t = 0.0;
    for (int k = 1; k <= periods; ++k) {
        const double window_start = k * period - 2.0 * delta;
        run.force_evaluations += advance_by(state, window_start - t);
        t = window_start;
        sample();
        std::array<double, 5> d2{};
        d2[0] = config_distance2(state, initial);
        for (std::size_t j = 1; j < 5; ++j) {
            run.force_evaluations += advance_by(state, delta);
            t += delta;
            d2[j] = config_distance2(state, initial);
        }
        sample();
        const auto m = static_cast<std::size_t>(
            std::clamp<std::ptrdiff_t>(std::min_element(d2.begin(), d2.end()) - d2.begin(), 1, 3));
        const double denom = d2[m - 1] - 2.0 * d2[m] + d2[m + 1];
        const double t_m = window_start + static_cast<double>(m) * delta;
        run.return_times.push_back(t_m + 0.5 * delta * (d2[m - 1] - d2[m + 1]) / denom);
    }
    run.max_energy = de.all_finite() ? de.value : std::nan("");
    run.max_momentum = dp.all_finite() ? dp.value : std::nan("");
    run.max_angular = dl.all_finite() ? dl.value : std::nan("");
    return run;
}

} // namespace

TEST_CASE("phase 3a scenarios load with provenance", "[scene][phase3a]") {
    const scene::Scene ss = load("solar_system_j2000.json");
    CHECK(ss.state.bodies().size() == 10);
    CHECK(ss.metadata.source.find("https://ssd.jpl.nasa.gov/api/horizons.api") !=
          std::string::npos);
    CHECK(ss.metadata.source.find("retrieved 2026-11-11") != std::string::npos);
    CHECK(ss.metadata.source.find("REF_PLANE=ECLIPTIC") != std::string::npos);
    CHECK(ss.metadata.epoch.find("TDB") != std::string::npos);

    const scene::Scene f8 = load("figure_eight.json");
    CHECK(f8.state.bodies().size() == 3);
    const ConservedQuantities q = ConservationDiagnostics::measure(f8.state, NewtonianGravity{});
    // v3 = -2 v1 exactly in binary, so the total momentum is exactly zero.
    CHECK(q.linear_momentum == Vec3<space::Momentum>{});

    const scene::Scene s2 = load("sgr_a_s2.json");
    CHECK(s2.state.bodies().size() == 2);
    CHECK(tag_of(s2.state.bodies().kind(0)) == BodyKindTag::black_hole);
    CHECK(s2.state.bodies().mass(0).value() == 4.261e6);
}

TEST_CASE("J2000 solar system: IAS15 one year against JPL Horizons", "[scene][phase3a]") {
    const scene::Scene sc = load("solar_system_j2000.json");
    SimState state = sc.state;
    const NewtonianGravity gravity;
    const ConservationDiagnostics diag(state, gravity);
    IAS15 ias;
    // One Julian year in 1-day output intervals.
    RunningMax de;
    for (int day = 0; day < 365; ++day) {
        step_or_throw(ias, state, space::Time(1.0), gravity);
        de.add(diag.drift(state, gravity).relative_energy);
    }
    step_or_throw(ias, state, space::Time(0.25), gravity);
    const ConservationDrift d = diag.drift(state, gravity);

    // JPL Horizons API, retrieved 2026-11-11, same query as the scenario
    // (CENTER=500@0, REF_PLANE=ECLIPTIC, REF_SYSTEM=ICRF, VEC_CORR=NONE,
    // OUT_UNITS=AU-D) at TLIST=2451910.25 (2000-Dec-31 18:00 TDB, J2000 +
    // 365.25 d). Positions in AU, DE441.
    struct Reference {
        const char* name;
        std::array<double, 3> position;
    };
    const std::array<Reference, 5> reference = {{
        {"Sun", {-4.636934570434959E-03, -4.922487071773944E-03, 1.552423563633955E-04}},
        {"Mercury", {1.589771551565417E-01, -4.175526676624136E-01, -4.856965360957761E-02}},
        {"Earth", {-1.817077874770017E-01, 9.623230821239243E-01, 1.554721937867174E-04}},
        {"Moon", {-1.791155500176532E-01, 9.616609852001022E-01, -5.989660265828535E-05}},
        {"Jupiter system", {1.793910107332750E+00, 4.709245150325921E+00, -5.968147594278114E-02}},
    }};
    for (const Reference& r : reference) {
        const Vec3<space::Length> p = state.bodies().position(index_of(state.bodies(), r.name));
        const Vec3<space::Length> h{space::Length(r.position[0]), space::Length(r.position[1]),
                                    space::Length(r.position[2])};
        const double err = norm(p - h).value();
        report(std::string("J2000 + 365.25 d, IAS15 vs Horizons: ") + r.name + " " + sci(err) +
               " AU (" + sci(err * 149597870.7) + " km)");
        CHECK(err < 1e-5);
    }
    report("J2000 IAS15 1 yr: max |dE/E| " + sci(de.value) + ", momentum drift " +
           sci(d.linear_momentum) + ", angular momentum drift " + sci(d.angular_momentum) +
           ", steps " + std::to_string(ias.accepted_steps()));
    CHECK(de.below(1e-13));
    CHECK(d.linear_momentum < 1e-13);
    CHECK(d.angular_momentum < 1e-13);
}

TEST_CASE("figure-eight with IAS15: energy and period over 100 periods", "[scene][phase3a]") {
    // Acceptance: figure-eight conserved to < 1e-8 with IAS15.
    IAS15 ias;
    const NewtonianGravity gravity;
    const FigureEightRun run = run_figure_eight(100, [&](SimState& s, double dt) {
        const long long before = ias.force_evaluations();
        step_or_throw(ias, s, space::Time(dt), gravity);
        return ias.force_evaluations() - before;
    });
    const double t1 = run.return_times.front();
    const double t100 = run.return_times.back() / 100.0;
    const double lit_rel = std::abs(t1 - figure_eight_period) / figure_eight_period;
    const double drift_rel = std::abs(t100 - t1) / t1;
    report("figure-8 IAS15 100 periods: max |dE/E| " + sci(run.max_energy) +
           ", max momentum drift " + sci(run.max_momentum) + ", angular momentum drift " +
           sci(run.max_angular) + "; first period " + std::to_string(t1) + " d (" +
           std::to_string(t1 * gauss_k) + " scaled), mean over 100 " + std::to_string(t100) +
           " d; |T1 - T_lit| / T_lit " + sci(lit_rel) + ", |T100 - T1| / T1 " + sci(drift_rel) +
           "; force evaluations " + std::to_string(run.force_evaluations));
    CHECK(run.max_energy < 1e-8);
    CHECK(run.max_momentum < 1e-12);
    CHECK(drift_rel < 1e-8);
    // The published initial conditions carry 8-9 significant digits.
    CHECK(lit_rel < 1e-7);
}

TEST_CASE("figure-eight with Yoshida4 for comparison", "[scene][phase3a]") {
    const NewtonianGravity gravity;
    Yoshida4 yoshida;
    const double delta = figure_eight_period / 2000.0;
    const FigureEightRun run = run_figure_eight(100, [&](SimState& s, double dt) {
        const auto steps = static_cast<long long>(std::llround(dt / delta));
        const double h = dt / static_cast<double>(steps);
        advance(yoshida, s, space::Time(h), steps, gravity);
        return 4 * steps;
    });
    const double t1 = run.return_times.front();
    const double t100 = run.return_times.back() / 100.0;
    report("figure-8 Yoshida4 dt=T/2000 100 periods: max |dE/E| " + sci(run.max_energy) +
           ", momentum drift " + sci(run.max_momentum) + "; first period " + std::to_string(t1) +
           " d, |T100 - T1| / T1 " + sci(std::abs(t100 - t1) / t1) + "; force evaluations " +
           std::to_string(run.force_evaluations));
    CHECK(run.max_energy < 1e-8);
    CHECK(std::abs(t100 - t1) / t1 < 1e-6);
}

TEST_CASE("Sgr A* + S2: period from the integrated orbit", "[scene][phase3a]") {
    // Acceptance: S2 period 16.0 +- 0.3 yr.
    const scene::Scene sc = load("sgr_a_s2.json");
    SimState state = sc.state;
    const NewtonianGravity gravity;
    const double julian_year = 365.25;
    const double mu = constants::gravitational_constant_space.value() * 4.261e6;
    const double a = 1031.3158086;
    const double kepler = 2.0 * std::numbers::pi * std::sqrt(a * a * a / mu);

    // r . v of S2 relative to Sgr A*: changes sign from - to + at pericentre.
    const auto radial = [](const SimState& s) {
        const Vec3<space::Length> r = s.bodies().position(1) - s.bodies().position(0);
        const Vec3<space::Velocity> v = s.bodies().velocity(1) - s.bodies().velocity(0);
        return r.x.value() * v.x.value() + r.y.value() * v.y.value() + r.z.value() * v.z.value();
    };
    IAS15 ias;
    const double sample = kepler / 400.0;
    std::vector<double> pericentres;
    double t = 0.0;
    double previous = radial(state);
    while (pericentres.size() < 3) {
        const SimState before = state;
        step_or_throw(ias, state, space::Time(sample), gravity);
        const double now = radial(state);
        if (previous < 0.0 && now >= 0.0) {
            // Bisection on the crossing time from the saved sample.
            double lo = 0.0;
            double hi = sample;
            for (int it = 0; it < 60; ++it) {
                const double mid = 0.5 * (lo + hi);
                SimState probe = before;
                IAS15 fresh;
                step_or_throw(fresh, probe, space::Time(mid), gravity);
                (radial(probe) < 0.0 ? lo : hi) = mid;
            }
            pericentres.push_back(t + 0.5 * (lo + hi));
        }
        previous = now;
        t += sample;
    }
    const double p1 = (pericentres[1] - pericentres[0]) / julian_year;
    const double p2 = (pericentres[2] - pericentres[1]) / julian_year;
    report("S2 integrated period " + std::to_string(p1) + " yr and " + std::to_string(p2) +
           " yr (Julian), Kepler 2 pi sqrt(a^3 / GM) " + std::to_string(kepler / julian_year) +
           " yr, GRAVITY 2020 fit 16.0455 yr; relative difference to Kepler " +
           sci(std::abs(p1 * julian_year - kepler) / kepler));
    CHECK(std::abs(p1 - 16.0) <= 0.3);
    CHECK(std::abs(p2 - 16.0) <= 0.3);
    CHECK(std::abs(p1 * julian_year - kepler) < 1e-8 * kepler);
}
