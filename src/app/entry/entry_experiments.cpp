#include "orrery/sim/entry_experiments.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <optional>
#include <system_error>
#include <utility>

#include "orrery/scene/scenario.h"

namespace orrery::sim {

namespace {

// Record keys (docs/entry-workspace.md, "Experiments").
constexpr const char* kSolverName = "entry_point_mass";

void put(scene::Parameters& p, const char* key, double value) {
    p.emplace(key, value);
}
void put(scene::Parameters& p, const char* key, std::string value) {
    p.emplace(key, std::move(value));
}

// Reads typed values from one section and remembers the first problem.
class Reader {
public:
    Reader(const scene::Parameters& p, std::string section, std::string& error)
        : p_(p), section_(std::move(section)), error_(error) {}

    double number(const char* key, double fallback) {
        const auto it = p_.find(key);
        if (it == p_.end()) {
            return fallback;
        }
        const double* value = std::get_if<double>(&it->second);
        if (value == nullptr || !std::isfinite(*value)) {
            fail(key, "must be a finite number");
            return fallback;
        }
        return *value;
    }

    std::string text(const char* key, std::string fallback) {
        const auto it = p_.find(key);
        if (it == p_.end()) {
            return fallback;
        }
        const std::string* value = std::get_if<std::string>(&it->second);
        if (value == nullptr) {
            fail(key, "must be text");
            return fallback;
        }
        return *value;
    }

    void fail(const char* key, const std::string& why) {
        if (error_.empty()) {
            error_ = section_ + "." + key + ": " + why;
        }
    }

private:
    const scene::Parameters& p_;
    std::string section_;
    std::string& error_;
};

scene::Series series_of(const EntrySeries& s) {
    return {{"time_s", s.time_s},
            {"altitude_m", s.altitude_m},
            {"speed_m_s", s.speed_m_s},
            {"flight_path_angle_deg", s.flight_path_angle_deg},
            {"g_load", s.g_load},
            {"dynamic_pressure_pa", s.dynamic_pressure_pa},
            {"heat_flux_w_m2", s.heat_flux_w_m2},
            {"mach", s.mach},
            {"latitude_deg", s.latitude_deg},
            {"longitude_deg", s.longitude_deg},
            {"downrange_m", s.downrange_m}};
}

std::optional<EntryOutcome> outcome_from(const std::string& text) {
    for (const EntryOutcome o : {EntryOutcome::none, EntryOutcome::landed, EntryOutcome::skip_out,
                                 EntryOutcome::orbiting}) {
        if (to_string(o) == text) {
            return o;
        }
    }
    return std::nullopt;
}

std::string read_setup(const scene::Experiment& e, EntrySetup& s) {
    std::string error;
    Reader in(e.inputs, "inputs", error);
    s.name = in.text("name", s.name);
    const std::string planet = in.text("planet", "earth");
    if (planet != "earth" && planet != "mars") {
        in.fail("planet", "must be \"earth\" or \"mars\"");
    }
    s.planet = planet == "mars" ? EntryPlanet::mars : EntryPlanet::earth;
    const std::string atmosphere = in.text("atmosphere", "standard");
    if (atmosphere != "standard" && atmosphere != "exponential") {
        in.fail("atmosphere", "must be \"standard\" or \"exponential\"");
    }
    s.atmosphere =
        atmosphere == "exponential" ? EntryAtmosphere::exponential : EntryAtmosphere::standard;
    s.exponential_density_kg_m3 = in.number("density_kg_m3", s.exponential_density_kg_m3);
    s.exponential_scale_height_m = in.number("scale_height_m", s.exponential_scale_height_m);
    s.mass_kg = in.number("mass_kg", s.mass_kg);
    s.reference_area_m2 = in.number("reference_area_m2", s.reference_area_m2);
    s.nose_radius_m = in.number("nose_radius_m", s.nose_radius_m);
    s.cd_source = in.text("cd_source", "table") == "aero" ? CdSource::aero_solver : CdSource::table;

    Reader c(e.conditions, "conditions", error);
    s.start = c.text("start", "state") == "periapsis" ? EntryStart::periapsis : EntryStart::state;
    s.altitude_m = c.number("altitude_m", s.altitude_m);
    s.speed_m_s = c.number("speed_m_s", s.speed_m_s);
    s.flight_path_angle_deg = c.number("flight_path_angle_deg", s.flight_path_angle_deg);
    s.periapsis_altitude_m = c.number("periapsis_altitude_m", s.periapsis_altitude_m);
    s.latitude_deg = c.number("latitude_deg", s.latitude_deg);
    s.longitude_deg = c.number("longitude_deg", s.longitude_deg);
    s.heading_deg = c.number("heading_deg", s.heading_deg);
    s.max_time_s = c.number("max_time_s", s.max_time_s);

    const auto mach = e.series.find("cd_mach");
    const auto cd = e.series.find("cd");
    if ((mach == e.series.end()) != (cd == e.series.end()) ||
        (mach != e.series.end() && mach->second.size() != cd->second.size())) {
        if (error.empty()) {
            error = "series.cd_mach and series.cd must have the same length";
        }
    } else if (mach != e.series.end()) {
        s.cd_table.clear();
        for (std::size_t i = 0; i < mach->second.size(); ++i) {
            s.cd_table.push_back({mach->second[i], cd->second[i]});
        }
    }
    return error;
}

std::string read_result(const scene::Experiment& e, EntryTrace& t) {
    std::string error;
    Reader r(e.result, "result", error);
    const std::string outcome = r.text("outcome", "none");
    const std::optional<EntryOutcome> parsed = outcome_from(outcome);
    if (!parsed) {
        r.fail("outcome", "must be none, landed, skip_out or orbiting");
    }
    EntrySeries& s = t.series;
    const std::vector<std::pair<const char*, std::vector<double>*>> columns{
        {"time_s", &s.time_s},
        {"altitude_m", &s.altitude_m},
        {"speed_m_s", &s.speed_m_s},
        {"flight_path_angle_deg", &s.flight_path_angle_deg},
        {"g_load", &s.g_load},
        {"dynamic_pressure_pa", &s.dynamic_pressure_pa},
        {"heat_flux_w_m2", &s.heat_flux_w_m2},
        {"mach", &s.mach},
        {"latitude_deg", &s.latitude_deg},
        {"longitude_deg", &s.longitude_deg},
        {"downrange_m", &s.downrange_m}};
    for (const auto& [key, column] : columns) {
        const auto it = e.series.find(key);
        if (it != e.series.end()) {
            *column = it->second;
        }
    }
    if (!s.consistent() && error.empty()) {
        error = "series: every entry series must have the same length";
    }
    if (error.empty() && !std::is_sorted(s.time_s.begin(), s.time_s.end())) {
        error = "series.time_s must not decrease";
    }
    t.summary = summarize(s, parsed.value_or(EntryOutcome::none));
    return error;
}

} // namespace

scene::Experiment entry_to_experiment(const EntrySetup& s, const EntryTrace* result) {
    scene::Experiment e;
    e.domain = scene::Domain::entry;
    put(e.inputs, "name", s.name);
    put(e.inputs, "planet", std::string(to_string(s.planet)));
    put(e.inputs, "atmosphere", std::string(to_string(s.atmosphere)));
    if (s.atmosphere == EntryAtmosphere::exponential) {
        put(e.inputs, "density_kg_m3", s.exponential_density_kg_m3);
        put(e.inputs, "scale_height_m", s.exponential_scale_height_m);
    }
    put(e.inputs, "mass_kg", s.mass_kg);
    put(e.inputs, "reference_area_m2", s.reference_area_m2);
    put(e.inputs, "nose_radius_m", s.nose_radius_m);
    put(e.inputs, "cd_source",
        std::string(s.cd_source == CdSource::aero_solver ? "aero" : "table"));
    put(e.conditions, "start",
        std::string(s.start == EntryStart::periapsis ? "periapsis" : "state"));
    put(e.conditions, "altitude_m", s.altitude_m);
    put(e.conditions, "speed_m_s", s.speed_m_s);
    put(e.conditions, "flight_path_angle_deg", s.flight_path_angle_deg);
    put(e.conditions, "periapsis_altitude_m", s.periapsis_altitude_m);
    put(e.conditions, "latitude_deg", s.latitude_deg);
    put(e.conditions, "longitude_deg", s.longitude_deg);
    put(e.conditions, "heading_deg", s.heading_deg);
    put(e.conditions, "max_time_s", s.max_time_s);
    e.solver.name = kSolverName;
    e.solver.version = "1";
    std::vector<double> mach;
    std::vector<double> cd;
    for (const MachCd& p : s.cd_table) {
        mach.push_back(p.mach);
        cd.push_back(p.cd);
    }
    e.series.emplace("cd_mach", std::move(mach));
    e.series.emplace("cd", std::move(cd));
    if (result != nullptr) {
        const EntrySummary& r = result->summary;
        put(e.result, "outcome", std::string(to_string(r.outcome)));
        put(e.result, "peak_g", r.peak_g);
        put(e.result, "peak_heat_flux_w_m2", r.peak_heat_flux_w_m2);
        put(e.result, "peak_dynamic_pressure_pa", r.peak_dynamic_pressure_pa);
        put(e.result, "heat_load_j_m2", r.heat_load_j_m2);
        put(e.result, "duration_s", r.duration_s);
        put(e.result, "downrange_m", r.downrange_m);
        for (auto& [key, values] : series_of(result->series)) {
            e.series.emplace(key, values);
        }
    }
    return e;
}

core::Result<StoredEntry, std::string> entry_from_experiment(const scene::Experiment& e) {
    if (e.domain != scene::Domain::entry) {
        return core::failure(std::string("domain: not an entry experiment"));
    }
    StoredEntry stored;
    if (std::string error = read_setup(e, stored.setup); !error.empty()) {
        return core::failure(std::move(error));
    }
    if (!e.result.empty()) {
        auto trace = std::make_shared<EntryTrace>();
        if (std::string error = read_result(e, *trace); !error.empty()) {
            return core::failure(std::move(error));
        }
        stored.result = std::move(trace);
    }
    return stored;
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
    auto typed = entry_from_experiment(generic.value());
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
        const std::string text = scene::save_experiment(entry_to_experiment(setup, result));
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
        auto typed = entry_from_experiment(generic.value());
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
