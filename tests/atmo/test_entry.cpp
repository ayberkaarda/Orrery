#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <memory>
#include <numbers>
#include <utility>
#include <variant>

#include "orrery/atmo/entry.h"

using orrery::atmo::EntryConfig;
using orrery::atmo::EntryExperiment;
using orrery::atmo::EntryOutcome;
using orrery::core::Angle;
namespace si = orrery::core::si;

namespace {

EntryExperiment make(const EntryConfig& config) {
    auto run = EntryExperiment::create(config);
    REQUIRE(run);
    return std::move(run).value();
}

void run_to_end(EntryExperiment& run) {
    auto outcome = run.run();
    REQUIRE(outcome);
}

constexpr double kDegree = std::numbers::pi / 180.0;

} // namespace

TEST_CASE("Apollo-like ballistic entry: peak deceleration 6 to 8 g", "[atmo][entry]") {
    EntryExperiment run = make(orrery::atmo::apollo_like_entry());
    CHECK(run.config().integrator == orrery::atmo::EntryIntegrator::ias15);
    run_to_end(run);
    const auto& s = run.summary();
    INFO("peak g " << s.peak_g_load << " at " << s.peak_g_time_s << " s, heat flux "
                   << s.peak_heat_flux_w_m2 << " W/m^2");
    CHECK(s.outcome == EntryOutcome::impact);
    CHECK(s.peak_g_load >= 6.0);
    CHECK(s.peak_g_load <= 8.0);
    // Sutton-Graves on the 4.694 m heat shield: order 1 MW/m^2.
    CHECK(s.peak_heat_flux_w_m2 > 0.5e6);
    CHECK(s.peak_heat_flux_w_m2 < 3.0e6);
    // Events: entry at the start (the run starts on the interface), impact last.
    const auto events = run.events();
    REQUIRE(events.size() == 2);
    REQUIRE(std::holds_alternative<orrery::atmo::AtmosphericEntryEvent>(events[0]));
    CHECK(std::get<orrery::atmo::AtmosphericEntryEvent>(events[0]).time_s == 0.0);
    REQUIRE(std::holds_alternative<orrery::atmo::ImpactEvent>(events[1]));
    const auto& impact = std::get<orrery::atmo::ImpactEvent>(events[1]);
    CHECK(impact.time_s == s.duration_s);
    CHECK(run.series().altitude_m.back() == 0.0);
    CHECK(orrery::atmo::event_name(events[1]) == "impact");
    // Subsonic terminal descent under drag alone.
    CHECK(impact.speed_m_s < 300.0);
    // Eastward along the equator, the ground track stays on it.
    CHECK(std::abs(impact.latitude_deg) < 1e-9);
    CHECK(impact.longitude_deg > 5.0);
    CHECK(impact.longitude_deg < 60.0);
}

TEST_CASE("Ballistic entry at the lifting Apollo angle exceeds the 6 to 8 g band",
          "[atmo][entry]") {
    // Apollo's nominal -6.5 degrees relied on lift (ADR-012 excludes it):
    // ballistic, the same entry is far more severe.
    EntryConfig c = orrery::atmo::apollo_like_entry();
    c.initial.flight_path_angle = Angle::from_degrees(-6.5);
    EntryExperiment run = make(c);
    run_to_end(run);
    CHECK(run.summary().peak_g_load > 10.0);
    CHECK(run.outcome() == EntryOutcome::impact);
}

TEST_CASE("Entry energy balance: mechanical energy loss equals drag work", "[atmo][entry]") {
    for (const auto frame :
         {orrery::atmo::AtmosphereFrame::inertial, orrery::atmo::AtmosphereFrame::co_rotating}) {
        EntryConfig c = orrery::atmo::apollo_like_entry();
        c.frame = frame;
        EntryExperiment run = make(c);
        run_to_end(run);
        const orrery::atmo::EntryEnergyBalance b = run.energy_balance();
        INFO("frame " << orrery::atmo::to_string(frame) << ": dE " << b.energy_change_j
                      << " J, drag work " << b.drag_work_j << " J, quadrature "
                      << b.drag_work_quadrature_j << " J");
        // Most of the orbital energy is dissipated.
        CHECK(b.energy_change_j < -3.0e11);
        // Acceptance: within 1 percent of the independently integrated drag work.
        CHECK(b.relative_error < 0.01);
        // The split step's own bookkeeping closes to the gravity integrator's
        // energy error.
        CHECK(std::abs(b.residual_j) < 1e-6 * std::abs(b.drag_work_j));
        CHECK(b.energy_j == run.series().energy_j.back());
    }
}

TEST_CASE("Shallow entry skips out of the atmosphere", "[atmo][entry]") {
    EntryExperiment run = make(orrery::atmo::skip_out_entry());
    run_to_end(run);
    CHECK(run.outcome() == EntryOutcome::skip_out);
    const auto events = run.events();
    REQUIRE(events.size() == 2);
    REQUIRE(std::holds_alternative<orrery::atmo::SkipOutEvent>(events[1]));
    const auto& skip = std::get<orrery::atmo::SkipOutEvent>(events[1]);
    CHECK(orrery::atmo::event_name(events[1]) == "skip_out");
    CHECK(skip.time_s > 0.0);
    CHECK(skip.flight_path_angle_deg > 0.0);
    CHECK(skip.speed_m_s < 11000.0);
    // Below escape speed: the capsule will fall back after an apoapsis
    // above the atmosphere.
    CHECK(skip.bound);
    CHECK(skip.apoapsis_altitude_m > 121920.0);
    CHECK(run.summary().peak_g_load > 0.5);
    CHECK(run.series().altitude_m.back() >= 121920.0);
    CHECK(run.energy_balance().relative_error < 0.01);
}

// Allen and Eggers (NACA Report 1381, 1958): ballistic entry through an
// isothermal exponential atmosphere without gravity, straight path:
//   n_max = V_E^2 sin(gamma) / (2 e H g0),   V(n_max) = V_E exp(-1/2),
//   rho(n_max) = beta sin(gamma) / H,        beta = m / (Cd A).
TEST_CASE("Steep entry through an exponential atmosphere matches Allen-Eggers", "[atmo][entry]") {
    orrery::atmo::ExponentialAtmosphereParameters params;
    params.top = si::Length(150000.0);
    auto atmosphere = orrery::atmo::ExponentialAtmosphere::create(params);
    REQUIRE(atmosphere);
    EntryConfig c;
    c.planet.atmosphere =
        std::make_shared<const orrery::atmo::ExponentialAtmosphere>(atmosphere.value());
    c.planet.gm = si::GravitationalParameter(1.0); // no gravity, as in the theory
    c.vehicle.mass = si::Mass(300.0);
    auto table = orrery::atmo::AeroCoefficientTable::constant(1.0, si::Area(1.0));
    REQUIRE(table);
    c.vehicle.aero = table.value();
    c.initial.altitude = si::Length(120000.0);
    c.initial.speed = si::Velocity(10000.0);
    c.initial.flight_path_angle = Angle::from_degrees(-60.0);
    c.time_step = si::Time(0.01);
    EntryExperiment run = make(c);
    run_to_end(run);
    const double sin_gamma = std::sin(60.0 * kDegree);
    const double h = 7200.0;
    const double n_max = 1.0e8 * sin_gamma / (2.0 * std::exp(1.0) * h * 9.80665);
    const auto& s = run.summary();
    INFO("peak g " << s.peak_g_load << " (Allen-Eggers " << n_max << ")");
    CHECK(std::abs(s.peak_g_load / n_max - 1.0) < 0.015);
    const auto& series = run.series();
    std::size_t peak = 0;
    for (std::size_t i = 0; i < series.size(); ++i) {
        if (series.g_load[i] > series.g_load[peak]) {
            peak = i;
        }
    }
    CHECK(std::abs(series.speed_m_s[peak] / (1.0e4 * std::exp(-0.5)) - 1.0) < 0.015);
    const double rho_peak = 300.0 * sin_gamma / h;
    CHECK(std::abs(series.density_kg_m3[peak] / rho_peak - 1.0) < 0.05);
}

// Quadratic drag dv/dt = -k v|v| in a uniform atmosphere without gravity:
// v(t) = v0 / (1 + k v0 t). With rho 1.2, Cd 1, A 1 m^2 and m 1 kg, k is
// 0.6 per metre, so at 100 m/s the drag timescale 1 / (k v) is 1/60 s, far
// shorter than the 0.25 s half-step kick.
TEST_CASE("Stiff drag follows the analytic quadratic-drag solution", "[atmo][entry]") {
    orrery::atmo::ExponentialAtmosphereParameters params;
    params.surface_density = si::Density(1.2);
    params.scale_height = si::Length(1.0e12); // uniform over the run
    params.top = si::Length(1.0e6);
    auto atmosphere = orrery::atmo::ExponentialAtmosphere::create(params);
    REQUIRE(atmosphere);
    EntryConfig c;
    c.planet.atmosphere =
        std::make_shared<const orrery::atmo::ExponentialAtmosphere>(atmosphere.value());
    c.planet.gm = si::GravitationalParameter(1.0);
    c.vehicle.mass = si::Mass(1.0);
    auto table = orrery::atmo::AeroCoefficientTable::constant(1.0, si::Area(1.0));
    REQUIRE(table);
    c.vehicle.aero = table.value();
    c.initial.altitude = si::Length(1000.0);
    c.initial.speed = si::Velocity(100.0);
    c.initial.flight_path_angle = Angle::from_degrees(0.0);
    c.max_duration = si::Time(5.0);
    EntryExperiment run = make(c);
    auto outcome = run.run();
    REQUIRE(outcome);
    CHECK(outcome.value() == EntryOutcome::time_limit);
    const auto& series = run.series();
    REQUIRE(series.size() == 11);
    const double k = 0.6;
    for (std::size_t i = 0; i < series.size(); ++i) {
        const double t = series.time_s[i];
        const double exact = 100.0 / (1.0 + k * 100.0 * t);
        INFO("t " << t << " s: speed " << series.speed_m_s[i] << " m/s, exact " << exact);
        REQUIRE(std::isfinite(series.speed_m_s[i]));
        CHECK(std::abs(series.speed_m_s[i] / exact - 1.0) < 1e-4);
        if (i > 0) {
            CHECK(series.speed_m_s[i] < series.speed_m_s[i - 1]);
        }
    }
    // The kicks' bookkeeping matches the kinetic energy actually removed.
    const orrery::atmo::EntryEnergyBalance b = run.energy_balance();
    CHECK(b.energy_change_j < 0.0);
    CHECK(std::abs(b.drag_work_j / b.energy_change_j - 1.0) < 1e-9);
}

TEST_CASE("A low ballistic coefficient deep in the atmosphere stays dissipative", "[atmo][entry]") {
    // A 10 g, 1 m^2 sheet at 1 km on the US76 atmosphere: beta = 0.01 kg/m^2.
    EntryConfig c;
    c.vehicle.mass = si::Mass(0.01);
    auto table = orrery::atmo::AeroCoefficientTable::constant(1.0, si::Area(1.0));
    REQUIRE(table);
    c.vehicle.aero = table.value();
    c.vehicle.nose_radius = si::Length(0.5);
    c.initial.altitude = si::Length(1000.0);
    c.initial.speed = si::Velocity(1000.0);
    c.initial.flight_path_angle = Angle::from_degrees(-10.0);
    c.max_duration = si::Time(20.0);
    EntryExperiment run = make(c);
    auto outcome = run.run();
    REQUIRE(outcome);
    CHECK(outcome.value() != EntryOutcome::failed);
    const auto& series = run.series();
    REQUIRE(series.size() > 2);
    for (std::size_t i = 1; i < series.size(); ++i) {
        INFO("row " << i << ": speed " << series.speed_m_s[i] << " m/s");
        REQUIRE(std::isfinite(series.speed_m_s[i]));
        REQUIRE(std::isfinite(series.energy_j[i]));
        // Drag only removes energy (gravity's own error is far smaller).
        CHECK(series.energy_j[i] <= series.energy_j[i - 1] + 1e-6 * std::abs(series.energy_j[0]));
    }
    // Falls at about its terminal speed sqrt(2 m g / (rho Cd A)) ~ 0.42 m/s.
    CHECK(series.speed_m_s.back() < 1.0);
}

TEST_CASE("Ground track on the rotating planet", "[atmo][entry]") {
    // Heading north from the equator: the path stays in the x-z plane, so
    // the longitude is the planet's rotation only.
    EntryConfig c = orrery::atmo::apollo_like_entry();
    c.initial.heading = Angle::from_degrees(0.0);
    EntryExperiment run = make(c);
    REQUIRE(run.advance(si::Time(200.0)));
    const auto& series = run.series();
    const double omega_deg = c.planet.rotation_rate.value() / kDegree;
    for (std::size_t i = 0; i < series.size(); i += 50) {
        CHECK(std::abs(series.longitude_deg[i] + omega_deg * series.time_s[i]) < 1e-9);
    }
    CHECK(series.latitude_deg.back() > 5.0);
    const orrery::atmo::GroundPoint g = orrery::atmo::ground_point(
        c.planet, {si::Length(0.0), si::Length(7.0e6), si::Length(0.0)}, si::Time(0.0));
    CHECK(std::abs(g.longitude_deg - 90.0) < 1e-12);
    CHECK(g.latitude_deg == 0.0);
}

TEST_CASE("Co-rotating atmosphere reduces the airspeed of an eastward entry", "[atmo][entry]") {
    EntryConfig c = orrery::atmo::apollo_like_entry();
    c.frame = orrery::atmo::AtmosphereFrame::co_rotating;
    EntryExperiment run = make(c);
    // v_rel = v - omega z x r at the start (equator, heading east).
    const double omega = c.planet.rotation_rate.value();
    const double r = c.planet.radius.value() + 121920.0;
    const double gamma = -5.7 * kDegree;
    const double east = 11000.0 * std::cos(gamma) - omega * r;
    const double up = 11000.0 * std::sin(gamma);
    CHECK(std::abs(run.series().airspeed_m_s.front() - std::hypot(east, up)) < 1e-6);
    CHECK(std::abs(run.series().speed_m_s.front() - 11000.0) < 1e-6);
}

TEST_CASE("Entry runs are deterministic and independent of the advance chunks", "[atmo][entry]") {
    EntryExperiment whole = make(orrery::atmo::apollo_like_entry());
    run_to_end(whole);
    EntryExperiment again = make(orrery::atmo::apollo_like_entry());
    run_to_end(again);
    CHECK(whole.series() == again.series());
    EntryExperiment chunked = make(orrery::atmo::apollo_like_entry());
    while (!chunked.finished()) {
        REQUIRE(chunked.advance(si::Time(37.3)));
    }
    CHECK(chunked.series() == whole.series());
    CHECK(chunked.events().size() == whole.events().size());
    // After the end nothing changes.
    const std::size_t rows = chunked.series().size();
    auto after = chunked.advance(si::Time(100.0));
    REQUIRE(after);
    CHECK(after.value() == EntryOutcome::impact);
    CHECK(chunked.series().size() == rows);
}

TEST_CASE("A step-bounded advance takes at most the given number of steps", "[atmo][entry]") {
    EntryExperiment run = make(orrery::atmo::apollo_like_entry());
    auto outcome = run.advance(si::Time(100.0), 3);
    REQUIRE(outcome);
    CHECK(outcome.value() == EntryOutcome::running);
    CHECK(run.series().size() == 4); // the initial row and three steps
    CHECK(run.time().value() == 1.5);
    REQUIRE(run.advance(si::Time(100.0), 0));
    CHECK(run.series().size() == 4);
}

TEST_CASE("Gravity integrators agree on the split step", "[atmo][entry]") {
    EntryExperiment ias15 = make(orrery::atmo::apollo_like_entry());
    run_to_end(ias15);
    EntryConfig c = orrery::atmo::apollo_like_entry();
    c.integrator = orrery::atmo::EntryIntegrator::yoshida4;
    EntryExperiment yoshida = make(c);
    run_to_end(yoshida);
    CHECK(std::abs(yoshida.summary().peak_g_load / ias15.summary().peak_g_load - 1.0) < 1e-4);
    CHECK(orrery::atmo::to_string(c.integrator) == "yoshida4");
}

TEST_CASE("Time limit and coasting above the atmosphere", "[atmo][entry]") {
    EntryConfig c = orrery::atmo::apollo_like_entry();
    c.max_duration = si::Time(50.0);
    EntryExperiment limited = make(c);
    run_to_end(limited);
    CHECK(limited.outcome() == EntryOutcome::time_limit);
    CHECK(limited.series().time_s.back() == 50.0);
    CHECK(limited.time().value() == 50.0);

    // Start 1500 km up, above the 1000 km top of US76: long coast steps,
    // then the entry interface is crossed and reported.
    EntryConfig high = orrery::atmo::apollo_like_entry();
    high.initial.altitude = si::Length(1.5e6);
    high.initial.flight_path_angle = Angle::from_degrees(-30.0);
    EntryExperiment coast = make(high);
    run_to_end(coast);
    const auto& t = coast.series().time_s;
    REQUIRE(t.size() > 2);
    CHECK(t[1] - t[0] > 1.0);
    const auto events = coast.events();
    REQUIRE_FALSE(events.empty());
    REQUIRE(std::holds_alternative<orrery::atmo::AtmosphericEntryEvent>(events[0]));
    CHECK(std::get<orrery::atmo::AtmosphericEntryEvent>(events[0]).time_s > 0.0);
    CHECK(coast.outcome() == EntryOutcome::impact);
}

TEST_CASE("Mars entry with the Mars atmosphere", "[atmo][entry]") {
    // Pathfinder-like: 585 kg, 2.65 m aeroshell, Cd 1.7, 7.48 km/s at -14 degrees.
    EntryConfig c;
    c.planet = orrery::atmo::Planet::mars();
    c.vehicle.mass = si::Mass(585.0);
    auto table = orrery::atmo::AeroCoefficientTable::constant(
        1.7, si::Area(std::numbers::pi * 1.325 * 1.325));
    REQUIRE(table);
    c.vehicle.aero = table.value();
    c.vehicle.nose_radius = si::Length(0.6638);
    c.initial.altitude = si::Length(125000.0);
    c.initial.speed = si::Velocity(7480.0);
    c.initial.flight_path_angle = Angle::from_degrees(-14.0);
    EntryExperiment run = make(c);
    run_to_end(run);
    INFO("peak g " << run.summary().peak_g_load);
    CHECK(run.outcome() == EntryOutcome::impact);
    CHECK(run.summary().peak_g_load > 5.0);
    CHECK(run.summary().peak_g_load < 40.0);
    CHECK(run.energy_balance().relative_error < 0.01);
}

TEST_CASE("Entry configuration is validated", "[atmo][entry]") {
    const auto rejected = [](const EntryConfig& c) {
        auto run = EntryExperiment::create(c);
        return !run && run.error().code == orrery::atmo::AtmoErrorCode::invalid_parameter;
    };
    EntryConfig c = orrery::atmo::apollo_like_entry();
    c.vehicle.mass = si::Mass(0.0);
    CHECK(rejected(c));
    c = orrery::atmo::apollo_like_entry();
    c.time_step = si::Time(0.0);
    CHECK(rejected(c));
    c = orrery::atmo::apollo_like_entry();
    c.max_coast_step = si::Time(0.1);
    CHECK(rejected(c));
    c = orrery::atmo::apollo_like_entry();
    c.initial.altitude = si::Length(-1.0);
    CHECK(rejected(c));
    c = orrery::atmo::apollo_like_entry();
    c.planet.atmosphere.reset();
    CHECK(rejected(c));
    c = orrery::atmo::apollo_like_entry();
    c.ias15_tolerance = 0.0;
    CHECK(rejected(c));
    EntryExperiment run = make(orrery::atmo::apollo_like_entry());
    CHECK_FALSE(run.advance(si::Time(-1.0)));
    CHECK(orrery::atmo::to_string(EntryOutcome::skip_out) == "skip_out");
}
