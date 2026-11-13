#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "orrery/atmo/aero_table.h"
#include "orrery/atmo/drag.h"
#include "orrery/atmo/error.h"
#include "orrery/atmo/planet.h"
#include "orrery/core/angle.h"
#include "orrery/core/force_model.h"
#include "orrery/core/ias15.h"
#include "orrery/core/integrator.h"
#include "orrery/core/quantity.h"
#include "orrery/core/result.h"
#include "orrery/core/sim_state.h"
#include "orrery/core/vec3.h"

// Atmospheric entry of a probe (docs/atmosphere.md). The coupling chain of
// ADR-014 is wired explicitly inside EntryExperiment, with no globals:
//
//   core SimState (position, velocity) -> IAtmosphere (density, Mach)
//   AeroCoefficientTable (Cd(Mach), area) -> AtmosphericDrag (DragForce)
//   AtmosphericDrag (velocity kick) + NewtonianGravity -> core integrator
namespace orrery::atmo {

struct EntryVehicle {
    std::string name = "capsule";
    core::si::Mass mass{5560.0};
    // Cd(Mach) and the reference area it is normalised by.
    AeroCoefficientTable aero;
    // Stagnation-point radius for the Sutton-Graves heat flux.
    core::si::Length nose_radius{4.694};
};

// Initial state at time 0, planet-centred inertial values. The body-fixed
// frame coincides with the inertial frame at time 0, so latitude and
// longitude locate the start on the surface below.
struct EntryInitialState {
    core::si::Length altitude{121920.0};
    // Inertial speed.
    core::si::Velocity speed{11000.0};
    // Inertial flight-path angle above the local horizontal (negative while
    // descending).
    core::Angle flight_path_angle = core::Angle::from_degrees(-6.5);
    core::Angle latitude{};
    core::Angle longitude{};
    // Azimuth of the horizontal velocity, clockwise from north (90 = east).
    core::Angle heading = core::Angle::from_degrees(90.0);
};

// Integrator of the gravity part of the split step (IAS15 recommended).
enum class EntryIntegrator : std::uint8_t { ias15, yoshida4, leapfrog_kdk };

[[nodiscard]] std::string_view to_string(EntryIntegrator integrator) noexcept;

struct EntryConfig {
    Planet planet = Planet::earth();
    EntryVehicle vehicle;
    EntryInitialState initial;
    AtmosphereFrame frame = AtmosphereFrame::inertial;
    EntryIntegrator integrator = EntryIntegrator::ias15;
    double ias15_tolerance = core::IAS15::default_tolerance;
    // Split step while inside the atmosphere; also the spacing of the series.
    core::si::Time time_step{0.5};
    // Longest step while above the atmosphere's top.
    core::si::Time max_coast_step{30.0};
    // The run ends with outcome time_limit at this simulated time.
    core::si::Time max_duration{4000.0};
    // End the run when the vehicle leaves the atmosphere after entering it.
    bool stop_on_skip_out = true;
};

// The trajectory crossed the entry interface altitude downwards (also
// reported at time 0 when the run starts at or below it).
struct AtmosphericEntryEvent {
    double time_s = 0.0;
    double altitude_m = 0.0;
    double speed_m_s = 0.0;
    double flight_path_angle_deg = 0.0;

    friend bool operator==(const AtmosphericEntryEvent&, const AtmosphericEntryEvent&) = default;
};

// After an entry, the trajectory crossed the entry interface upwards.
struct SkipOutEvent {
    double time_s = 0.0;
    double altitude_m = 0.0;
    double speed_m_s = 0.0;
    double flight_path_angle_deg = 0.0;
    // v^2 / 2 - GM / r at the crossing.
    double specific_energy_j_kg = 0.0;
    // Negative specific energy: the vehicle will fall back.
    bool bound = false;
    // Apoapsis altitude of the osculating orbit (bound only, else 0).
    double apoapsis_altitude_m = 0.0;

    friend bool operator==(const SkipOutEvent&, const SkipOutEvent&) = default;
};

// The trajectory reached altitude 0 (spherical surface). Ends the run.
struct ImpactEvent {
    double time_s = 0.0;
    double speed_m_s = 0.0;
    double airspeed_m_s = 0.0;
    double flight_path_angle_deg = 0.0;
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;

    friend bool operator==(const ImpactEvent&, const ImpactEvent&) = default;
};

using EntryEvent = std::variant<AtmosphericEntryEvent, SkipOutEvent, ImpactEvent>;

// "atmospheric_entry", "skip_out" or "impact".
[[nodiscard]] std::string_view event_name(const EntryEvent& event) noexcept;

// Time series of a run, one row per accepted step (structure of arrays, all
// columns the same length, ready for plotting). SI units as named.
struct EntrySeries {
    std::vector<double> time_s;
    std::vector<double> altitude_m;
    // Inertial speed and flight-path angle (planet-centred).
    std::vector<double> speed_m_s;
    std::vector<double> flight_path_angle_deg;
    // Speed relative to the air (AtmosphereFrame).
    std::vector<double> airspeed_m_s;
    std::vector<double> mach;
    std::vector<double> cd;
    std::vector<double> density_kg_m3;
    std::vector<double> dynamic_pressure_pa;
    // Sensed (non-gravitational) acceleration |a_drag| / g0, g0 = 9.80665.
    std::vector<double> g_load;
    // Sutton-Graves stagnation-point convective heat flux, W/m^2.
    std::vector<double> heat_flux_w_m2;
    // Ground track on the rotating planet (docs/atmosphere.md, "Frames").
    std::vector<double> latitude_deg;
    std::vector<double> longitude_deg;
    // Cumulative work of the drag force (negative), J.
    std::vector<double> drag_work_j;
    // Mechanical energy m (v^2 / 2 - GM / r), J.
    std::vector<double> energy_j;

    [[nodiscard]] std::size_t size() const noexcept { return time_s.size(); }

    friend bool operator==(const EntrySeries&, const EntrySeries&) = default;
};

enum class EntryOutcome : std::uint8_t { running, impact, skip_out, time_limit, failed };

[[nodiscard]] std::string_view to_string(EntryOutcome outcome) noexcept;

struct EntrySummary {
    EntryOutcome outcome = EntryOutcome::running;
    double duration_s = 0.0;
    double peak_g_load = 0.0;
    double peak_g_time_s = 0.0;
    double peak_heat_flux_w_m2 = 0.0;
    double peak_heat_flux_time_s = 0.0;
    double peak_dynamic_pressure_pa = 0.0;
    double peak_dynamic_pressure_time_s = 0.0;
    // Integrated stagnation-point heat load, J/m^2.
    double heat_load_j_m2 = 0.0;
    double min_altitude_m = 0.0;
    double final_speed_m_s = 0.0;
};

// Work-energy balance of the probe (docs/atmosphere.md, "Energy balance").
// core::ConservationDiagnostics leaves test particles out of its totals, so
// the entry tracks the probe's own mechanical energy relative to the planet.
struct EntryEnergyBalance {
    double initial_energy_j = 0.0;
    double energy_j = 0.0;
    double energy_change_j = 0.0;
    // Sum of the kinetic energy removed by the drag kicks (exact
    // bookkeeping of the split step).
    double drag_work_j = 0.0;
    // Trapezoidal integral of F . v over the series: an estimate of the drag
    // work independent of the integrator's bookkeeping.
    double drag_work_quadrature_j = 0.0;
    // energy_change - drag_work: the gravity integrator's energy error.
    double residual_j = 0.0;
    // |energy_change - drag_work_quadrature| / |drag_work_quadrature| (0
    // while no drag acted).
    double relative_error = 0.0;
};

// Latitude and longitude of the point below `position` on the planet rotating
// at planet.rotation_rate about +z since time 0, degrees; longitude in
// [-180, 180).
struct GroundPoint {
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
};

[[nodiscard]] GroundPoint ground_point(const Planet& planet,
                                       const core::Vec3<core::si::Length>& position,
                                       core::si::Time time) noexcept;

// Entry simulation: a two-body SimState (planet at the origin, body 0; probe,
// body 1) in space units, integrated with Strang splitting
//   drag kick (dt/2) -> gravity step (dt, core integrator) -> drag kick (dt/2)
// so that the position-only core integrators stay untouched while drag acts
// on the velocity (docs/atmosphere.md, "Integration"). Events are detected on
// every step from the altitude before and after it (linear interpolation of
// the crossing time). Movable, not copyable, not thread-safe.
class EntryExperiment {
public:
    // Validates the configuration (finite positive mass, steps, durations,
    // nose radius; an atmosphere; initial altitude above 0) and builds the
    // chain.
    [[nodiscard]] static core::Result<EntryExperiment, AtmoError> create(EntryConfig config);

    EntryExperiment(EntryExperiment&&) noexcept;
    EntryExperiment& operator=(EntryExperiment&&) noexcept;
    EntryExperiment(const EntryExperiment&) = delete;
    EntryExperiment& operator=(const EntryExperiment&) = delete;
    ~EntryExperiment();

    // Advances by up to `duration` of simulated time, stopping early at an
    // end condition; returns the outcome afterwards (running if none was
    // reached). After a failure the outcome is failed and the state is the
    // last good one. Calls after the end change nothing.
    [[nodiscard]] core::Result<EntryOutcome, AtmoError> advance(core::si::Time duration);
    // Advances until an end condition (at the latest max_duration).
    [[nodiscard]] core::Result<EntryOutcome, AtmoError> run();

    [[nodiscard]] bool finished() const noexcept;
    [[nodiscard]] EntryOutcome outcome() const noexcept;
    [[nodiscard]] core::si::Time time() const noexcept;

    [[nodiscard]] const EntryConfig& config() const noexcept;
    [[nodiscard]] const core::SimState& state() const noexcept;
    [[nodiscard]] const AtmosphericDrag& drag() const noexcept;
    [[nodiscard]] const EntrySeries& series() const noexcept;
    [[nodiscard]] std::span<const EntryEvent> events() const noexcept;
    [[nodiscard]] const EntrySummary& summary() const noexcept;
    [[nodiscard]] EntryEnergyBalance energy_balance() const noexcept;

private:
    struct Impl;
    explicit EntryExperiment(std::unique_ptr<Impl> impl) noexcept;

    std::unique_ptr<Impl> impl_;
};

// Apollo-like lunar return (docs/atmosphere.md, "Scenarios"): command-module
// mass 5560 kg, 3.912 m diameter, Cd 1.3, nose radius 4.694 m, 11.0 km/s at
// the 121.92 km entry interface, flight-path angle -5.7 degrees, ballistic
// (no lift, ADR-012). Apollo flew about -6.5 degrees with lift modulation
// (L/D about 0.3) to hold 6 to 7 g; without lift that angle gives about 15 g,
// so the ballistic analogue sits on the shallow side of the Apollo corridor.
[[nodiscard]] EntryConfig apollo_like_entry();
// The same vehicle and speed at -5.0 degrees: one drag pass of about 1 g,
// then the vehicle leaves the atmosphere again (skip-out).
[[nodiscard]] EntryConfig skip_out_entry();

} // namespace orrery::atmo
