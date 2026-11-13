#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "orrery/atmo/drag.h"
#include "orrery/atmo/entry.h"
#include "orrery/core/result.h"
#include "orrery/scene/experiment.h"

// Typed view of an experiment record of the "entry" domain
// (docs/atmosphere.md, "Experiment record"): the planet and atmosphere, the
// vehicle and its Cd(Mach) table, the initial state, the integration settings
// and, once a run finished, its result. Stored in the generic Experiment
// schema (version 1) with these keys; anything else is rejected by
// entry_from_experiment.
//
//   inputs      name (text), planet ("earth" | "mars"), atmosphere ("us76" |
//               "mars" | "exponential"); exponential: surface_density_kg_m3,
//               scale_height_m, temperature_k; vehicle (text), mass_kg,
//               reference_area_m2, nose_radius_m
//   conditions  altitude_m, speed_m_s, flight_path_angle_deg, latitude_deg,
//               longitude_deg, heading_deg, interface_altitude_m, frame
//               ("inertial" | "co_rotating")
//   solver      name: "ias15", "yoshida4" or "leapfrog_kdk"; version "1";
//               settings: tolerance, time_step_s, max_coast_step_s,
//               max_duration_s, stop_on_skip_out (0 or 1)
//   result      outcome ("running" | "impact" | "skip_out" | "time_limit" |
//               "failed"), duration_s, peak_g_load, peak_g_time_s,
//               peak_heat_flux_w_m2, peak_heat_flux_time_s,
//               peak_dynamic_pressure_pa, peak_dynamic_pressure_time_s,
//               heat_load_j_m2, min_altitude_m, final_speed_m_s, drag_work_j,
//               energy_change_j, energy_relative_error, and when the event
//               occurred: entry_time_s, skip_out_time_s, impact_time_s,
//               impact_latitude_deg, impact_longitude_deg
//   series      cd_table_mach, cd_table_cd (the Cd(Mach) table, an input:
//               parameters hold scalars only), and with a result the
//               atmo::EntrySeries columns under their member names (time_s,
//               altitude_m, ..., energy_j)
namespace orrery::scene {

enum class EntryAtmosphereModel : std::uint8_t { us76, mars, exponential };

[[nodiscard]] std::string_view to_string(EntryAtmosphereModel model) noexcept;
[[nodiscard]] std::optional<EntryAtmosphereModel>
entry_atmosphere_from_string(std::string_view name) noexcept;

struct EntryEnvironment {
    // "earth" or "mars" (atmo::Planet presets).
    std::string planet = "earth";
    EntryAtmosphereModel atmosphere = EntryAtmosphereModel::us76;
    // Exponential model only.
    double surface_density_kg_m3 = 1.225;
    double scale_height_m = 7200.0;
    double temperature_k = 288.15;

    friend bool operator==(const EntryEnvironment&, const EntryEnvironment&) = default;
};

struct EntryVehicleRecord {
    std::string name = "capsule";
    double mass_kg = 5560.0;
    double reference_area_m2 = 12.0193;
    double nose_radius_m = 4.694;
    // Cd(Mach) table, same length, Mach strictly increasing.
    std::vector<double> cd_table_mach{0.0};
    std::vector<double> cd_table_cd{1.3};

    friend bool operator==(const EntryVehicleRecord&, const EntryVehicleRecord&) = default;
};

// Initial state (atmo::EntryInitialState in degrees and metres).
struct EntryConditions {
    double altitude_m = 121920.0;
    double speed_m_s = 11000.0;
    double flight_path_angle_deg = -6.5;
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double heading_deg = 90.0;
    // Entry interface altitude; nullopt uses the planet's.
    std::optional<double> interface_altitude_m;
    atmo::AtmosphereFrame frame = atmo::AtmosphereFrame::inertial;

    friend bool operator==(const EntryConditions&, const EntryConditions&) = default;
};

struct EntrySolverSettings {
    atmo::EntryIntegrator integrator = atmo::EntryIntegrator::ias15;
    double tolerance = 1.0e-9;
    double time_step_s = 0.5;
    double max_coast_step_s = 30.0;
    double max_duration_s = 4000.0;
    bool stop_on_skip_out = true;

    friend bool operator==(const EntrySolverSettings&, const EntrySolverSettings&) = default;
};

struct EntryResultRecord {
    atmo::EntryOutcome outcome = atmo::EntryOutcome::running;
    double duration_s = 0.0;
    double peak_g_load = 0.0;
    double peak_g_time_s = 0.0;
    double peak_heat_flux_w_m2 = 0.0;
    double peak_heat_flux_time_s = 0.0;
    double peak_dynamic_pressure_pa = 0.0;
    double peak_dynamic_pressure_time_s = 0.0;
    double heat_load_j_m2 = 0.0;
    double min_altitude_m = 0.0;
    double final_speed_m_s = 0.0;
    double drag_work_j = 0.0;
    double energy_change_j = 0.0;
    double energy_relative_error = 0.0;
    std::optional<double> entry_time_s;
    std::optional<double> skip_out_time_s;
    std::optional<double> impact_time_s;
    std::optional<double> impact_latitude_deg;
    std::optional<double> impact_longitude_deg;
    atmo::EntrySeries series;

    friend bool operator==(const EntryResultRecord&, const EntryResultRecord&) = default;
};

struct EntryExperiment {
    std::string name;
    EntryEnvironment environment;
    EntryVehicleRecord vehicle;
    EntryConditions conditions;
    EntrySolverSettings solver;
    std::optional<EntryResultRecord> result;

    friend bool operator==(const EntryExperiment&, const EntryExperiment&) = default;
};

inline constexpr std::string_view kEntrySolverVersion = "1";

// The generic record of an entry experiment (domain entry, schema version 1).
[[nodiscard]] Experiment to_experiment(const EntryExperiment& experiment);

// Reads an entry experiment back; the error names the offending key
// ("inputs.mass_kg: must be positive and finite").
[[nodiscard]] core::Result<EntryExperiment, std::string>
entry_from_experiment(const Experiment& experiment);

[[nodiscard]] core::Result<EntryExperiment, std::string>
parse_entry_experiment(std::string_view json);
[[nodiscard]] std::string save_entry_experiment(const EntryExperiment& experiment);

// The simulation configuration of a record (planet preset, atmosphere model,
// Cd table, initial state, settings). Fails with a message naming the key.
[[nodiscard]] core::Result<atmo::EntryConfig, std::string>
to_entry_config(const EntryExperiment& experiment);

// The result record of a run (summary, energy balance, events, series).
[[nodiscard]] EntryResultRecord entry_result(const atmo::EntryExperiment& run);

} // namespace orrery::scene
