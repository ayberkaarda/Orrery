#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// Inputs, time series and summary of the Entry workspace
// (docs/entry-workspace.md): plain data shared by the setup panel, the
// solver thread, the profile plots, the view and the experiment records.
// Graphics-free; units are SI, angles in degrees as the user enters them.
namespace orrery::sim {

enum class EntryPlanet : std::uint8_t { earth, mars };
// Standard: US Standard Atmosphere 1976 for Earth, the Mars table for Mars.
enum class EntryAtmosphere : std::uint8_t { standard, exponential };
// Where the Cd(Mach) table came from: typed in, or sampled from an aero run.
enum class CdSource : std::uint8_t { table, aero_solver };
// How the initial state is given: altitude, speed and flight-path angle, or
// the vacuum periapsis altitude of the approach orbit (the flight-path angle
// at the start altitude is derived from it).
enum class EntryStart : std::uint8_t { state, periapsis };

enum class EntryOutcome : std::uint8_t {
    none,     // not finished
    landed,   // reached the ground
    skip_out, // left the atmosphere again on an open or high orbit
    orbiting, // the time limit came first (still flying)
};

[[nodiscard]] std::string_view to_string(EntryPlanet planet) noexcept;
[[nodiscard]] std::string_view to_string(EntryAtmosphere atmosphere) noexcept;
[[nodiscard]] std::string_view to_string(EntryOutcome outcome) noexcept;
// "Landed", "Skip-out", "Orbiting", "" for none.
[[nodiscard]] std::string_view outcome_label(EntryOutcome outcome) noexcept;

struct MachCd {
    double mach = 0.0;
    double cd = 0.0;

    friend bool operator==(const MachCd&, const MachCd&) = default;
};

// Integrator of the gravity part of the physics library's split step.
enum class EntryIntegrator : std::uint8_t { ias15, yoshida4, leapfrog_kdk };

// Integration settings of a run (docs/atmosphere.md, "Integration"); the
// defaults are the library's.
struct EntryIntegration {
    EntryIntegrator integrator = EntryIntegrator::ias15;
    double tolerance = 1.0e-9; // IAS15
    double time_step_s = 0.5;  // inside the atmosphere; also the series spacing
    double max_coast_step_s = 30.0;
    bool stop_on_skip_out = true;
    // Entry interface altitude; nullopt uses the planet's.
    std::optional<double> interface_altitude_m;

    friend bool operator==(const EntryIntegration&, const EntryIntegration&) = default;
};

struct EntrySetup {
    std::string name = "Apollo-like entry";
    EntryPlanet planet = EntryPlanet::earth;
    EntryAtmosphere atmosphere = EntryAtmosphere::standard;
    // Exponential model: rho = rho0 exp(-h / H).
    double exponential_density_kg_m3 = 1.225;
    double exponential_scale_height_m = 7200.0;
    double exponential_temperature_k = 288.15; // pressure and speed of sound
    // Probe (constant mass, no lift, heat shield forward: ADR-012).
    double mass_kg = 5500.0;
    double reference_area_m2 = 12.02;
    double nose_radius_m = 4.69; // stagnation-point heat flux (Sutton-Graves)
    CdSource cd_source = CdSource::table;
    std::vector<MachCd> cd_table{{0.5, 0.80}, {1.2, 1.05}, {2.0, 1.25}, {5.0, 1.30}, {25.0, 1.30}};
    // Initial state.
    EntryStart start = EntryStart::state;
    double altitude_m = 121'920.0; // 400 000 ft entry interface
    double speed_m_s = 7'800.0;    // inertial, planet-centred
    double flight_path_angle_deg = -2.0;
    double periapsis_altitude_m = 40'000.0;
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double heading_deg = 90.0; // from north towards east
    // Integration stops here when the probe neither lands nor leaves.
    double max_time_s = 3'600.0;
    // The air turns with the planet (airspeed = v - omega x r) instead of
    // resting in the inertial frame.
    bool co_rotating_atmosphere = false;
    EntryIntegration integration;

    friend bool operator==(const EntrySetup&, const EntrySetup&) = default;
};

// One recorded instant of an entry.
struct EntrySample {
    double time_s = 0.0;
    double altitude_m = 0.0;
    double speed_m_s = 0.0;
    double flight_path_angle_deg = 0.0;
    double g_load = 0.0;              // drag deceleration / g0
    double dynamic_pressure_pa = 0.0; // 0.5 rho v^2
    double heat_flux_w_m2 = 0.0;      // stagnation point, Sutton-Graves
    double mach = 0.0;
    double latitude_deg = 0.0;
    double longitude_deg = 0.0; // in (-180, 180]
    double downrange_m = 0.0;   // great-circle distance from the start
};

// The series of an entry, one vector per quantity (what ImPlot draws).
// Every vector has size() elements.
struct EntrySeries {
    std::vector<double> time_s;
    std::vector<double> altitude_m;
    std::vector<double> speed_m_s;
    std::vector<double> flight_path_angle_deg;
    std::vector<double> g_load;
    std::vector<double> dynamic_pressure_pa;
    std::vector<double> heat_flux_w_m2;
    std::vector<double> mach;
    std::vector<double> latitude_deg;
    std::vector<double> longitude_deg;
    std::vector<double> downrange_m;

    [[nodiscard]] std::size_t size() const noexcept { return time_s.size(); }
    [[nodiscard]] bool empty() const noexcept { return time_s.empty(); }
    // True when every vector has the same length.
    [[nodiscard]] bool consistent() const noexcept;
    void push(const EntrySample& sample);
    [[nodiscard]] EntrySample at(std::size_t index) const;
    void clear() noexcept;
    void reserve(std::size_t count);
};

struct EntrySummary {
    EntryOutcome outcome = EntryOutcome::none;
    double duration_s = 0.0;
    double peak_g = 0.0;
    double peak_g_time_s = 0.0;
    double peak_heat_flux_w_m2 = 0.0;
    double peak_heat_flux_time_s = 0.0;
    double peak_dynamic_pressure_pa = 0.0;
    double heat_load_j_m2 = 0.0; // time integral of the heat flux
    double downrange_m = 0.0;
    double final_speed_m_s = 0.0;
    double final_altitude_m = 0.0;

    friend bool operator==(const EntrySummary&, const EntrySummary&) = default;
};

// Peaks, integral and final values of `series` (trapezoidal heat load).
// Non-finite samples are skipped.
[[nodiscard]] EntrySummary summarize(const EntrySeries& series, EntryOutcome outcome);

// Value of `values` at time `t` by linear interpolation in `time_s`
// (clamped to the ends; nullopt when empty or the sizes differ).
[[nodiscard]] std::optional<double> value_at(const std::vector<double>& time_s,
                                             const std::vector<double>& values, double t);
// Index of the sample nearest to `t` (nullopt when empty).
[[nodiscard]] std::optional<std::size_t> nearest_index(const std::vector<double>& time_s, double t);

// The values of atmo::Planet::earth() and mars() the panels and the view need
// every frame (a test keeps them equal).
struct PlanetConstants {
    std::string_view name;
    double radius_m = 0.0;
    double mu_m3_s2 = 0.0;         // gravitational parameter
    double atmosphere_top_m = 0.0; // where the entry begins and skip-out is declared
};
[[nodiscard]] PlanetConstants planet_constants(EntryPlanet planet) noexcept;

// Flight-path angle [deg, negative = descending] at `altitude_m` and
// `speed_m_s` on the Kepler orbit whose periapsis is at
// `periapsis_altitude_m`; nullopt when no such orbit exists (periapsis above
// the start, or the speed too low to rise to the start altitude from it).
[[nodiscard]] std::optional<double>
flight_path_angle_for_periapsis(EntryPlanet planet, double altitude_m, double speed_m_s,
                                double periapsis_altitude_m) noexcept;
// Vacuum periapsis altitude of the orbit through the initial state (nullopt
// for radial or non-finite input).
[[nodiscard]] std::optional<double> periapsis_altitude(EntryPlanet planet, double altitude_m,
                                                       double speed_m_s,
                                                       double flight_path_angle_deg) noexcept;

// The flight-path angle the run starts with (the periapsis form resolved).
[[nodiscard]] std::optional<double> effective_flight_path_angle(const EntrySetup& setup) noexcept;

// Values no run can use, one line each naming the field ("Mass must be
// positive"); empty when the setup can run.
[[nodiscard]] std::vector<std::string> setup_errors(const EntrySetup& setup);

// Ballistic coefficient m / (Cd A) with the largest Cd of the table [kg/m^2];
// 0 when it cannot be formed.
[[nodiscard]] double ballistic_coefficient(const EntrySetup& setup) noexcept;

// Starting points of the setup panel; the first is the workspace default.
struct EntryPreset {
    const char* label = "";
    EntrySetup setup;
};
[[nodiscard]] std::vector<EntryPreset> entry_presets();

} // namespace orrery::sim
