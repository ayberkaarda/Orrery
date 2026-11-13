#include "orrery/sim/entry_experiments.h"

#include <algorithm>
#include <fstream>
#include <string_view>
#include <system_error>
#include <utility>

#include "orrery/scene/scenario.h"

namespace orrery::sim {

namespace {

// Vehicle names that carry the Cd source through a record.
constexpr std::string_view kTabulatedVehicle = "probe";
constexpr std::string_view kSampledVehicle = "probe (Cd from the aero solver)";

scene::EntryAtmosphereModel atmosphere_model(const EntrySetup& s) noexcept {
    if (s.atmosphere == EntryAtmosphere::exponential) {
        return scene::EntryAtmosphereModel::exponential;
    }
    return s.planet == EntryPlanet::mars ? scene::EntryAtmosphereModel::mars
                                         : scene::EntryAtmosphereModel::us76;
}

// The summary of a recorded result: from its series, or from the record's
// own values when it carries no series.
EntrySummary recorded_summary(const scene::EntryResultRecord& r, const EntrySeries& series) {
    const EntryOutcome outcome = outcome_from(r.outcome);
    if (!series.empty()) {
        return summarize(series, outcome);
    }
    EntrySummary s;
    s.outcome = outcome;
    s.duration_s = r.duration_s;
    s.peak_g = r.peak_g_load;
    s.peak_g_time_s = r.peak_g_time_s;
    s.peak_heat_flux_w_m2 = r.peak_heat_flux_w_m2;
    s.peak_heat_flux_time_s = r.peak_heat_flux_time_s;
    s.peak_dynamic_pressure_pa = r.peak_dynamic_pressure_pa;
    s.heat_load_j_m2 = r.heat_load_j_m2;
    s.final_speed_m_s = r.final_speed_m_s;
    return s;
}

} // namespace

scene::EntryExperiment entry_record(const EntrySetup& s, const EntryTrace* result) {
    scene::EntryExperiment r;
    r.name = s.name;
    r.environment.planet = std::string(to_string(s.planet));
    r.environment.atmosphere = atmosphere_model(s);
    r.environment.surface_density_kg_m3 = s.exponential_density_kg_m3;
    r.environment.scale_height_m = s.exponential_scale_height_m;
    r.environment.temperature_k = s.exponential_temperature_k;
    r.vehicle.name =
        std::string(s.cd_source == CdSource::aero_solver ? kSampledVehicle : kTabulatedVehicle);
    r.vehicle.mass_kg = s.mass_kg;
    r.vehicle.reference_area_m2 = s.reference_area_m2;
    r.vehicle.nose_radius_m = s.nose_radius_m;
    r.vehicle.cd_table_mach.clear();
    r.vehicle.cd_table_cd.clear();
    for (const MachCd& p : s.cd_table) {
        r.vehicle.cd_table_mach.push_back(p.mach);
        r.vehicle.cd_table_cd.push_back(p.cd);
    }
    r.conditions.altitude_m = s.altitude_m;
    r.conditions.speed_m_s = s.speed_m_s;
    r.conditions.flight_path_angle_deg =
        effective_flight_path_angle(s).value_or(s.flight_path_angle_deg);
    r.conditions.latitude_deg = s.latitude_deg;
    r.conditions.longitude_deg = s.longitude_deg;
    r.conditions.heading_deg = s.heading_deg;
    r.conditions.interface_altitude_m = s.integration.interface_altitude_m;
    r.conditions.frame = s.co_rotating_atmosphere ? atmo::AtmosphereFrame::co_rotating
                                                  : atmo::AtmosphereFrame::inertial;
    r.solver.integrator = to_atmo(s.integration.integrator);
    r.solver.tolerance = s.integration.tolerance;
    r.solver.time_step_s = s.integration.time_step_s;
    r.solver.max_coast_step_s = s.integration.max_coast_step_s;
    r.solver.stop_on_skip_out = s.integration.stop_on_skip_out;
    r.solver.max_duration_s = s.max_time_s;
    if (result != nullptr && result->record) {
        r.result = *result->record;
    }
    return r;
}

core::Result<StoredEntry, std::string> entry_from_record(const scene::EntryExperiment& r) {
    const scene::EntryEnvironment& env = r.environment;
    if (env.planet != "earth" && env.planet != "mars") {
        return core::failure("inputs.planet: unknown planet '" + env.planet + "'");
    }
    StoredEntry stored;
    EntrySetup& s = stored.setup;
    s.name = r.name;
    s.planet = env.planet == "mars" ? EntryPlanet::mars : EntryPlanet::earth;
    s.atmosphere = env.atmosphere == scene::EntryAtmosphereModel::exponential
                       ? EntryAtmosphere::exponential
                       : EntryAtmosphere::standard;
    if (atmosphere_model(s) != env.atmosphere) {
        return core::failure(
            "inputs.atmosphere: the Entry workspace flies " + env.planet +
            " in its own atmosphere (" + std::string(scene::to_string(atmosphere_model(s))) +
            ") or an exponential one, not '" + std::string(scene::to_string(env.atmosphere)) + "'");
    }
    s.exponential_density_kg_m3 = r.environment.surface_density_kg_m3;
    s.exponential_scale_height_m = r.environment.scale_height_m;
    s.exponential_temperature_k = r.environment.temperature_k;
    s.mass_kg = r.vehicle.mass_kg;
    s.reference_area_m2 = r.vehicle.reference_area_m2;
    s.nose_radius_m = r.vehicle.nose_radius_m;
    s.cd_table.clear();
    const std::size_t points =
        std::min(r.vehicle.cd_table_mach.size(), r.vehicle.cd_table_cd.size());
    for (std::size_t i = 0; i < points; ++i) {
        s.cd_table.push_back({r.vehicle.cd_table_mach[i], r.vehicle.cd_table_cd[i]});
    }
    s.cd_source = r.vehicle.name == kSampledVehicle ? CdSource::aero_solver : CdSource::table;
    s.start = EntryStart::state;
    s.altitude_m = r.conditions.altitude_m;
    s.speed_m_s = r.conditions.speed_m_s;
    s.flight_path_angle_deg = r.conditions.flight_path_angle_deg;
    s.latitude_deg = r.conditions.latitude_deg;
    s.longitude_deg = r.conditions.longitude_deg;
    s.heading_deg = r.conditions.heading_deg;
    s.max_time_s = r.solver.max_duration_s;
    s.co_rotating_atmosphere = r.conditions.frame == atmo::AtmosphereFrame::co_rotating;
    s.integration.interface_altitude_m = r.conditions.interface_altitude_m;
    s.integration.integrator = from_atmo(r.solver.integrator);
    s.integration.tolerance = r.solver.tolerance;
    s.integration.time_step_s = r.solver.time_step_s;
    s.integration.max_coast_step_s = r.solver.max_coast_step_s;
    s.integration.stop_on_skip_out = r.solver.stop_on_skip_out;
    // The periapsis form, should the user switch to it, starts from this orbit.
    s.periapsis_altitude_m =
        periapsis_altitude(s.planet, s.altitude_m, s.speed_m_s, s.flight_path_angle_deg)
            .value_or(s.periapsis_altitude_m);
    if (r.result) {
        auto trace = std::make_shared<EntryTrace>();
        trace->series = series_from(r.result->series, planet_constants(s.planet).radius_m);
        trace->summary = recorded_summary(*r.result, trace->series);
        trace->record = std::make_shared<const scene::EntryResultRecord>(*r.result);
        stored.result = std::move(trace);
    }
    return stored;
}

scene::Experiment entry_to_experiment(const EntrySetup& setup, const EntryTrace* result) {
    return scene::to_experiment(entry_record(setup, result));
}

core::Result<StoredEntry, std::string> entry_from_experiment(const scene::Experiment& experiment) {
    auto typed = scene::entry_from_experiment(experiment);
    if (!typed) {
        return core::failure(std::string(typed.error()));
    }
    auto stored = entry_from_record(typed.value());
    if (!stored) {
        return core::failure(std::string(stored.error()));
    }
    return std::move(stored).value();
}

core::Result<StoredEntry, std::string>
load_entry_experiment_file(const std::filesystem::path& path) {
    const auto text = scene::read_text_file(path);
    if (!text) {
        return core::failure(path.filename().string() + ": " + scene::to_string(text.error()));
    }
    const auto generic = scene::parse_experiment(text.value());
    if (!generic) {
        return core::failure(path.filename().string() + ": " + scene::to_string(generic.error()));
    }
    auto typed = sim::entry_from_experiment(generic.value());
    if (!typed) {
        return core::failure(path.filename().string() + ": " + typed.error());
    }
    StoredEntry stored = std::move(typed).value();
    stored.path = path;
    return stored;
}

core::Result<std::filesystem::path, std::string>
save_entry_experiment_file(const std::filesystem::path& path, const EntrySetup& setup,
                           const EntryTrace* result) {
    std::error_code error;
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path(), error);
        if (error) {
            return core::failure("cannot create " + path.parent_path().string() + ": " +
                                 error.message());
        }
    }
    std::filesystem::path temporary = path;
    temporary += ".tmp";
    {
        std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
        if (!out) {
            return core::failure("cannot write " + temporary.string());
        }
        const std::string text = scene::save_entry_experiment(entry_record(setup, result));
        out.write(text.data(), static_cast<std::streamsize>(text.size()));
        out.close();
        if (!out) {
            std::filesystem::remove(temporary, error);
            return core::failure("cannot write " + temporary.string());
        }
    }
    std::filesystem::rename(temporary, path, error);
    if (error) {
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        return core::failure("cannot replace " + path.string() + ": " + error.message());
    }
    return path;
}

EntryListing list_entry_experiments(const std::filesystem::path& directory) {
    EntryListing listing;
    std::error_code error;
    std::filesystem::directory_iterator it(directory, error);
    if (error) {
        return listing;
    }
    std::vector<std::filesystem::path> files;
    for (const std::filesystem::directory_iterator end; it != end; it.increment(error)) {
        if (error) {
            break;
        }
        std::error_code type_error;
        if (it->path().extension() == ".json" &&
            std::filesystem::is_regular_file(it->path(), type_error)) {
            files.push_back(it->path());
        }
    }
    std::sort(files.begin(), files.end());
    for (const std::filesystem::path& path : files) {
        const auto text = scene::read_text_file(path);
        if (!text) {
            listing.problems.push_back(path.filename().string() + ": " +
                                       scene::to_string(text.error()));
            continue;
        }
        const auto generic = scene::parse_experiment(text.value());
        if (!generic) {
            listing.problems.push_back(path.filename().string() + ": " +
                                       scene::to_string(generic.error()));
            continue;
        }
        if (generic.value().domain != scene::Domain::entry) {
            continue;
        }
        auto typed = sim::entry_from_experiment(generic.value());
        if (!typed) {
            listing.problems.push_back(path.filename().string() + ": " + typed.error());
            continue;
        }
        StoredEntry stored = std::move(typed).value();
        stored.path = path;
        listing.entries.push_back(std::move(stored));
    }
    return listing;
}

} // namespace orrery::sim
