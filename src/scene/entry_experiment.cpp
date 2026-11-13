#include "orrery/scene/entry_experiment.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "orrery/atmo/aero_table.h"
#include "orrery/atmo/atmosphere.h"
#include "orrery/atmo/planet.h"

namespace orrery::scene {

namespace {

using Error = std::string;

struct ModelName {
    EntryAtmosphereModel model;
    std::string_view name;
};

constexpr std::array<ModelName, 3> kModels{{
    {EntryAtmosphereModel::us76, "us76"},
    {EntryAtmosphereModel::mars, "mars"},
    {EntryAtmosphereModel::exponential, "exponential"},
}};

constexpr std::array<atmo::EntryIntegrator, 3> kIntegrators{atmo::EntryIntegrator::ias15,
                                                            atmo::EntryIntegrator::yoshida4,
                                                            atmo::EntryIntegrator::leapfrog_kdk};

constexpr std::array<atmo::EntryOutcome, 5> kOutcomes{
    atmo::EntryOutcome::running, atmo::EntryOutcome::impact, atmo::EntryOutcome::skip_out,
    atmo::EntryOutcome::time_limit, atmo::EntryOutcome::failed};

constexpr std::array<atmo::AtmosphereFrame, 2> kFrames{atmo::AtmosphereFrame::inertial,
                                                       atmo::AtmosphereFrame::co_rotating};

// Value checks of a numeric key.
enum class Rule : std::uint8_t { finite, positive, non_negative, angle90, flag };

bool satisfies(double v, Rule rule) noexcept {
    switch (rule) {
        case Rule::finite:
            return std::isfinite(v);
        case Rule::positive:
            return v > 0.0 && std::isfinite(v);
        case Rule::non_negative:
            return v >= 0.0 && std::isfinite(v);
        case Rule::angle90:
            return v >= -90.0 && v <= 90.0;
        case Rule::flag:
            return v == 0.0 || v == 1.0;
    }
    return false;
}

std::string_view describe(Rule rule) noexcept {
    switch (rule) {
        case Rule::finite:
            return "must be finite";
        case Rule::positive:
            return "must be positive and finite";
        case Rule::non_negative:
            return "must be non-negative and finite";
        case Rule::angle90:
            return "must be within [-90, 90] degrees";
        case Rule::flag:
            return "must be 0 or 1";
    }
    return "is invalid";
}

template <typename Enum, std::size_t N, typename Name>
std::optional<Enum> from_name(const std::array<Enum, N>& values, std::string_view name,
                              Name to_name) {
    for (const Enum value : values) {
        if (to_name(value) == name) {
            return value;
        }
    }
    return std::nullopt;
}

template <typename Enum, std::size_t N, typename Name>
std::string names(const std::array<Enum, N>& values, Name to_name) {
    std::string list;
    for (const Enum value : values) {
        list.append(list.empty() ? "" : ", ").append(to_name(value));
    }
    return list;
}

// Typed access to one parameter map; every error names "<where>.<key>".
class Reader {
public:
    Reader(const Parameters& parameters, std::string where)
        : parameters_(&parameters), where_(std::move(where)) {}

    [[nodiscard]] std::string path(std::string_view key) const {
        return where_ + "." + std::string(key);
    }

    [[nodiscard]] bool has(std::string_view key) const {
        return parameters_->find(key) != parameters_->end();
    }

    [[nodiscard]] std::optional<Error>
    unknown_key(std::span<const std::string_view> allowed) const {
        for (const auto& entry : *parameters_) {
            if (std::find(allowed.begin(), allowed.end(), entry.first) == allowed.end()) {
                return path(entry.first) + ": unknown key";
            }
        }
        return std::nullopt;
    }

    // The number at `key` (fallback when absent) if it satisfies `rule`.
    [[nodiscard]] core::Result<double, Error> number(std::string_view key, double fallback,
                                                     Rule rule) const {
        const auto it = parameters_->find(key);
        if (it == parameters_->end()) {
            return fallback;
        }
        const auto* value = std::get_if<double>(&it->second);
        if (value == nullptr) {
            return core::failure(path(key) + ": expected a number");
        }
        if (!satisfies(*value, rule)) {
            return core::failure(path(key) + ": " + std::string(describe(rule)));
        }
        return *value;
    }

    [[nodiscard]] core::Result<std::string, Error> text(std::string_view key,
                                                        std::string fallback) const {
        const auto it = parameters_->find(key);
        if (it == parameters_->end()) {
            return fallback;
        }
        if (const auto* value = std::get_if<std::string>(&it->second)) {
            return *value;
        }
        return core::failure(path(key) + ": expected a string");
    }

private:
    const Parameters* parameters_;
    std::string where_;
};

template <typename Record> struct NumberField {
    std::string_view key;
    double Record::* member = nullptr;
    Rule rule = Rule::finite;
};

// Reads every field of `fields` into `record`; the first error wins.
template <typename Record, std::size_t N>
std::optional<Error>
read_numbers(const Reader& in, const std::array<NumberField<Record>, N>& fields, Record& record) {
    for (const NumberField<Record>& field : fields) {
        auto value = in.number(field.key, record.*field.member, field.rule);
        if (!value) {
            return value.error();
        }
        record.*field.member = value.value();
    }
    return std::nullopt;
}

template <typename Record, std::size_t N>
void write_numbers(Parameters& out, const std::array<NumberField<Record>, N>& fields,
                   const Record& record) {
    for (const NumberField<Record>& field : fields) {
        out.emplace(std::string(field.key), record.*field.member);
    }
}

constexpr std::array<NumberField<EntryVehicleRecord>, 3> kVehicleNumbers{{
    {"mass_kg", &EntryVehicleRecord::mass_kg, Rule::positive},
    {"reference_area_m2", &EntryVehicleRecord::reference_area_m2, Rule::positive},
    {"nose_radius_m", &EntryVehicleRecord::nose_radius_m, Rule::positive},
}};

constexpr std::array<NumberField<EntryEnvironment>, 3> kExponentialNumbers{{
    {"surface_density_kg_m3", &EntryEnvironment::surface_density_kg_m3, Rule::positive},
    {"scale_height_m", &EntryEnvironment::scale_height_m, Rule::positive},
    {"temperature_k", &EntryEnvironment::temperature_k, Rule::positive},
}};

constexpr std::array<NumberField<EntryConditions>, 6> kConditionNumbers{{
    {"altitude_m", &EntryConditions::altitude_m, Rule::positive},
    {"speed_m_s", &EntryConditions::speed_m_s, Rule::non_negative},
    {"flight_path_angle_deg", &EntryConditions::flight_path_angle_deg, Rule::angle90},
    {"latitude_deg", &EntryConditions::latitude_deg, Rule::angle90},
    {"longitude_deg", &EntryConditions::longitude_deg, Rule::finite},
    {"heading_deg", &EntryConditions::heading_deg, Rule::finite},
}};

constexpr std::array<NumberField<EntrySolverSettings>, 4> kSolverNumbers{{
    {"tolerance", &EntrySolverSettings::tolerance, Rule::positive},
    {"time_step_s", &EntrySolverSettings::time_step_s, Rule::positive},
    {"max_coast_step_s", &EntrySolverSettings::max_coast_step_s, Rule::positive},
    {"max_duration_s", &EntrySolverSettings::max_duration_s, Rule::positive},
}};

constexpr std::array<NumberField<EntryResultRecord>, 13> kResultNumbers{{
    {"duration_s", &EntryResultRecord::duration_s, Rule::finite},
    {"peak_g_load", &EntryResultRecord::peak_g_load, Rule::finite},
    {"peak_g_time_s", &EntryResultRecord::peak_g_time_s, Rule::finite},
    {"peak_heat_flux_w_m2", &EntryResultRecord::peak_heat_flux_w_m2, Rule::finite},
    {"peak_heat_flux_time_s", &EntryResultRecord::peak_heat_flux_time_s, Rule::finite},
    {"peak_dynamic_pressure_pa", &EntryResultRecord::peak_dynamic_pressure_pa, Rule::finite},
    {"peak_dynamic_pressure_time_s", &EntryResultRecord::peak_dynamic_pressure_time_s,
     Rule::finite},
    {"heat_load_j_m2", &EntryResultRecord::heat_load_j_m2, Rule::finite},
    {"min_altitude_m", &EntryResultRecord::min_altitude_m, Rule::finite},
    {"final_speed_m_s", &EntryResultRecord::final_speed_m_s, Rule::finite},
    {"drag_work_j", &EntryResultRecord::drag_work_j, Rule::finite},
    {"energy_change_j", &EntryResultRecord::energy_change_j, Rule::finite},
    {"energy_relative_error", &EntryResultRecord::energy_relative_error, Rule::finite},
}};

struct OptionalField {
    std::string_view key;
    std::optional<double> EntryResultRecord::* member = nullptr;
};

constexpr std::array<OptionalField, 5> kResultOptionals{{
    {"entry_time_s", &EntryResultRecord::entry_time_s},
    {"skip_out_time_s", &EntryResultRecord::skip_out_time_s},
    {"impact_time_s", &EntryResultRecord::impact_time_s},
    {"impact_latitude_deg", &EntryResultRecord::impact_latitude_deg},
    {"impact_longitude_deg", &EntryResultRecord::impact_longitude_deg},
}};

struct SeriesColumn {
    std::string_view key;
    std::vector<double> atmo::EntrySeries::* member = nullptr;
};

constexpr std::array<SeriesColumn, 15> kSeriesColumns{{
    {"time_s", &atmo::EntrySeries::time_s},
    {"altitude_m", &atmo::EntrySeries::altitude_m},
    {"speed_m_s", &atmo::EntrySeries::speed_m_s},
    {"flight_path_angle_deg", &atmo::EntrySeries::flight_path_angle_deg},
    {"airspeed_m_s", &atmo::EntrySeries::airspeed_m_s},
    {"mach", &atmo::EntrySeries::mach},
    {"cd", &atmo::EntrySeries::cd},
    {"density_kg_m3", &atmo::EntrySeries::density_kg_m3},
    {"dynamic_pressure_pa", &atmo::EntrySeries::dynamic_pressure_pa},
    {"g_load", &atmo::EntrySeries::g_load},
    {"heat_flux_w_m2", &atmo::EntrySeries::heat_flux_w_m2},
    {"latitude_deg", &atmo::EntrySeries::latitude_deg},
    {"longitude_deg", &atmo::EntrySeries::longitude_deg},
    {"drag_work_j", &atmo::EntrySeries::drag_work_j},
    {"energy_j", &atmo::EntrySeries::energy_j},
}};

constexpr std::string_view kTableMach = "cd_table_mach";
constexpr std::string_view kTableCd = "cd_table_cd";

} // namespace

std::string_view to_string(EntryAtmosphereModel model) noexcept {
    for (const ModelName& entry : kModels) {
        if (entry.model == model) {
            return entry.name;
        }
    }
    return "us76";
}

std::optional<EntryAtmosphereModel> entry_atmosphere_from_string(std::string_view name) noexcept {
    for (const ModelName& entry : kModels) {
        if (entry.name == name) {
            return entry.model;
        }
    }
    return std::nullopt;
}

namespace {

void write_inputs(const EntryExperiment& e, Experiment& out) {
    if (!e.name.empty()) {
        out.inputs.emplace("name", e.name);
    }
    out.inputs.emplace("planet", e.environment.planet);
    out.inputs.emplace("atmosphere", std::string(to_string(e.environment.atmosphere)));
    if (e.environment.atmosphere == EntryAtmosphereModel::exponential) {
        write_numbers(out.inputs, kExponentialNumbers, e.environment);
    }
    out.inputs.emplace("vehicle", e.vehicle.name);
    write_numbers(out.inputs, kVehicleNumbers, e.vehicle);
    out.series.emplace(std::string(kTableMach), e.vehicle.cd_table_mach);
    out.series.emplace(std::string(kTableCd), e.vehicle.cd_table_cd);
}

void write_conditions(const EntryConditions& c, Experiment& out) {
    write_numbers(out.conditions, kConditionNumbers, c);
    if (c.interface_altitude_m) {
        out.conditions.emplace("interface_altitude_m", *c.interface_altitude_m);
    }
    out.conditions.emplace("frame", std::string(atmo::to_string(c.frame)));
}

void write_solver(const EntrySolverSettings& s, Experiment& out) {
    out.solver.name = std::string(atmo::to_string(s.integrator));
    out.solver.version = std::string(kEntrySolverVersion);
    write_numbers(out.solver.settings, kSolverNumbers, s);
    out.solver.settings.emplace("stop_on_skip_out", s.stop_on_skip_out ? 1.0 : 0.0);
}

void write_result(const EntryResultRecord& r, Experiment& out) {
    out.result.emplace("outcome", std::string(atmo::to_string(r.outcome)));
    write_numbers(out.result, kResultNumbers, r);
    for (const OptionalField& field : kResultOptionals) {
        if (const std::optional<double>& value = r.*field.member) {
            out.result.emplace(std::string(field.key), *value);
        }
    }
    for (const SeriesColumn& column : kSeriesColumns) {
        out.series.emplace(std::string(column.key), r.series.*column.member);
    }
}

core::Result<std::shared_ptr<const atmo::IAtmosphere>, Error>
make_atmosphere(const EntryEnvironment& env) {
    switch (env.atmosphere) {
        case EntryAtmosphereModel::us76:
            return std::shared_ptr<const atmo::IAtmosphere>(
                std::make_shared<const atmo::UsStandardAtmosphere1976>());
        case EntryAtmosphereModel::mars:
            return std::shared_ptr<const atmo::IAtmosphere>(
                std::make_shared<const atmo::MarsAtmosphere>());
        case EntryAtmosphereModel::exponential:
            break;
    }
    atmo::ExponentialAtmosphereParameters p;
    p.surface_density = core::si::Density(env.surface_density_kg_m3);
    p.scale_height = core::si::Length(env.scale_height_m);
    p.temperature_k = env.temperature_k;
    auto model = atmo::ExponentialAtmosphere::create(p);
    if (!model) {
        return core::failure("inputs.atmosphere: " + model.error().message);
    }
    return std::shared_ptr<const atmo::IAtmosphere>(
        std::make_shared<const atmo::ExponentialAtmosphere>(model.value()));
}

core::Result<atmo::EntryVehicle, Error> make_vehicle(const EntryVehicleRecord& v) {
    if (v.cd_table_mach.size() != v.cd_table_cd.size()) {
        return core::failure(std::string("series: cd_table_mach and cd_table_cd must have the "
                                         "same length"));
    }
    std::vector<atmo::CdSample> samples;
    samples.reserve(v.cd_table_mach.size());
    for (std::size_t i = 0; i < v.cd_table_mach.size(); ++i) {
        samples.push_back(atmo::CdSample{v.cd_table_mach[i], v.cd_table_cd[i]});
    }
    auto table =
        atmo::AeroCoefficientTable::create(std::move(samples), core::si::Area(v.reference_area_m2));
    if (!table) {
        return core::failure("series.cd_table: " + table.error().message);
    }
    atmo::EntryVehicle vehicle;
    vehicle.name = v.name;
    vehicle.mass = core::si::Mass(v.mass_kg);
    vehicle.nose_radius = core::si::Length(v.nose_radius_m);
    vehicle.aero = std::move(table).value();
    return vehicle;
}

atmo::EntryInitialState make_initial(const EntryConditions& c) {
    atmo::EntryInitialState s;
    s.altitude = core::si::Length(c.altitude_m);
    s.speed = core::si::Velocity(c.speed_m_s);
    s.flight_path_angle = core::Angle::from_degrees(c.flight_path_angle_deg);
    s.latitude = core::Angle::from_degrees(c.latitude_deg);
    s.longitude = core::Angle::from_degrees(c.longitude_deg);
    s.heading = core::Angle::from_degrees(c.heading_deg);
    return s;
}

void record_events(std::span<const atmo::EntryEvent> events, EntryResultRecord& r) {
    for (const atmo::EntryEvent& event : events) {
        if (const auto* entry = std::get_if<atmo::AtmosphericEntryEvent>(&event)) {
            r.entry_time_s = r.entry_time_s.value_or(entry->time_s);
        } else if (const auto* skip = std::get_if<atmo::SkipOutEvent>(&event)) {
            r.skip_out_time_s = r.skip_out_time_s.value_or(skip->time_s);
        } else if (const auto* impact = std::get_if<atmo::ImpactEvent>(&event)) {
            r.impact_time_s = impact->time_s;
            r.impact_latitude_deg = impact->latitude_deg;
            r.impact_longitude_deg = impact->longitude_deg;
        }
    }
}

core::Result<EntryEnvironment, Error> read_environment(const Reader& in) {
    EntryEnvironment env;
    auto planet = in.text("planet", env.planet);
    auto model = in.text("atmosphere", std::string(to_string(env.atmosphere)));
    if (!planet || !model) {
        return core::failure(!planet ? planet.error() : model.error());
    }
    if (planet.value() != "earth" && planet.value() != "mars") {
        return core::failure(in.path("planet") + ": unknown planet '" + planet.value() +
                             "' (allowed: earth, mars)");
    }
    env.planet = planet.value();
    const auto kind = entry_atmosphere_from_string(model.value());
    if (!kind) {
        return core::failure(in.path("atmosphere") + ": unknown atmosphere '" + model.value() +
                             "' (allowed: us76, mars, exponential)");
    }
    env.atmosphere = *kind;
    if (env.atmosphere != EntryAtmosphereModel::exponential) {
        for (const auto& field : kExponentialNumbers) {
            if (in.has(field.key)) {
                return core::failure(in.path(field.key) +
                                     ": only the exponential atmosphere takes this key");
            }
        }
        return env;
    }
    if (auto problem = read_numbers(in, kExponentialNumbers, env)) {
        return core::failure(std::move(*problem));
    }
    return env;
}

core::Result<EntryVehicleRecord, Error> read_vehicle(const Reader& in, const Series& series) {
    EntryVehicleRecord v;
    auto name = in.text("vehicle", v.name);
    if (!name) {
        return core::failure(name.error());
    }
    v.name = name.value();
    if (auto problem = read_numbers(in, kVehicleNumbers, v)) {
        return core::failure(std::move(*problem));
    }
    const auto mach = series.find(kTableMach);
    const auto cd = series.find(kTableCd);
    if ((mach == series.end()) != (cd == series.end())) {
        return core::failure(std::string("series: cd_table_mach and cd_table_cd go together"));
    }
    if (mach != series.end()) {
        v.cd_table_mach = mach->second;
        v.cd_table_cd = cd->second;
    }
    auto checked = make_vehicle(v);
    if (!checked) {
        return core::failure(checked.error());
    }
    return v;
}

core::Result<EntryConditions, Error> read_conditions(const Parameters& parameters) {
    const Reader in(parameters, "conditions");
    constexpr std::array<std::string_view, 8> kKeys{
        "altitude_m",    "speed_m_s",   "flight_path_angle_deg", "latitude_deg",
        "longitude_deg", "heading_deg", "interface_altitude_m",  "frame"};
    if (auto problem = in.unknown_key(kKeys)) {
        return core::failure(std::move(*problem));
    }
    EntryConditions c;
    if (auto problem = read_numbers(in, kConditionNumbers, c)) {
        return core::failure(std::move(*problem));
    }
    if (in.has("interface_altitude_m")) {
        auto value = in.number("interface_altitude_m", 0.0, Rule::positive);
        if (!value) {
            return core::failure(value.error());
        }
        c.interface_altitude_m = value.value();
    }
    auto frame = in.text("frame", std::string(atmo::to_string(c.frame)));
    if (!frame) {
        return core::failure(frame.error());
    }
    const auto chosen = from_name(kFrames, frame.value(),
                                  [](atmo::AtmosphereFrame f) { return atmo::to_string(f); });
    if (!chosen) {
        return core::failure(in.path("frame") + ": unknown frame '" + frame.value() +
                             "' (allowed: inertial, co_rotating)");
    }
    c.frame = *chosen;
    return c;
}

core::Result<EntrySolverSettings, Error> read_solver(const SolverInfo& info) {
    EntrySolverSettings s;
    const auto name = [](atmo::EntryIntegrator i) { return atmo::to_string(i); };
    const auto integrator = from_name(kIntegrators, info.name, name);
    if (!integrator) {
        return core::failure("solver.name: unknown integrator '" + info.name +
                             "' (allowed: " + names(kIntegrators, name) + ")");
    }
    s.integrator = *integrator;
    if (!info.version.empty() && info.version != kEntrySolverVersion) {
        return core::failure("solver.version: '" + info.version + "' is not supported");
    }
    const Reader in(info.settings, "solver.settings");
    constexpr std::array<std::string_view, 5> kKeys{"tolerance", "time_step_s", "max_coast_step_s",
                                                    "max_duration_s", "stop_on_skip_out"};
    if (auto problem = in.unknown_key(kKeys)) {
        return core::failure(std::move(*problem));
    }
    if (auto problem = read_numbers(in, kSolverNumbers, s)) {
        return core::failure(std::move(*problem));
    }
    if (s.max_coast_step_s < s.time_step_s) {
        return core::failure(in.path("max_coast_step_s") + ": must be at least time_step_s");
    }
    auto stop = in.number("stop_on_skip_out", 1.0, Rule::flag);
    if (!stop) {
        return core::failure(stop.error());
    }
    s.stop_on_skip_out = stop.value() == 1.0;
    return s;
}

std::optional<Error> read_result_series(const Series& series, EntryResultRecord& r) {
    std::size_t rows = 0;
    bool first = true;
    for (const SeriesColumn& column : kSeriesColumns) {
        const auto it = series.find(column.key);
        std::vector<double> values = it == series.end() ? std::vector<double>{} : it->second;
        if (!first && values.size() != rows) {
            return "series." + std::string(column.key) + ": every result series has " +
                   std::to_string(rows) + " values";
        }
        rows = values.size();
        first = false;
        r.series.*column.member = std::move(values);
    }
    return std::nullopt;
}

core::Result<std::optional<EntryResultRecord>, Error> read_result(const Experiment& experiment) {
    if (experiment.result.empty()) {
        return std::optional<EntryResultRecord>{};
    }
    const Reader in(experiment.result, "result");
    EntryResultRecord r;
    auto outcome = in.text("outcome", "");
    if (!outcome) {
        return core::failure(outcome.error());
    }
    const auto name = [](atmo::EntryOutcome o) { return atmo::to_string(o); };
    const auto kind = from_name(kOutcomes, outcome.value(), name);
    if (!kind) {
        return core::failure(in.path("outcome") + ": unknown outcome '" + outcome.value() +
                             "' (allowed: " + names(kOutcomes, name) + ")");
    }
    r.outcome = *kind;
    if (auto problem = read_numbers(in, kResultNumbers, r)) {
        return core::failure(std::move(*problem));
    }
    for (const OptionalField& field : kResultOptionals) {
        if (in.has(field.key)) {
            auto value = in.number(field.key, 0.0, Rule::finite);
            if (!value) {
                return core::failure(value.error());
            }
            r.*field.member = value.value();
        }
    }
    if (auto problem = read_result_series(experiment.series, r)) {
        return core::failure(std::move(*problem));
    }
    return std::optional<EntryResultRecord>(std::move(r));
}

constexpr std::array<std::string_view, 10> kInputKeys{"name",           "planet",
                                                      "atmosphere",     "vehicle",
                                                      "mass_kg",        "reference_area_m2",
                                                      "nose_radius_m",  "surface_density_kg_m3",
                                                      "scale_height_m", "temperature_k"};

constexpr std::array<std::string_view, 19> kResultKeys{"outcome",
                                                       "duration_s",
                                                       "peak_g_load",
                                                       "peak_g_time_s",
                                                       "peak_heat_flux_w_m2",
                                                       "peak_heat_flux_time_s",
                                                       "peak_dynamic_pressure_pa",
                                                       "peak_dynamic_pressure_time_s",
                                                       "heat_load_j_m2",
                                                       "min_altitude_m",
                                                       "final_speed_m_s",
                                                       "drag_work_j",
                                                       "energy_change_j",
                                                       "energy_relative_error",
                                                       "entry_time_s",
                                                       "skip_out_time_s",
                                                       "impact_time_s",
                                                       "impact_latitude_deg",
                                                       "impact_longitude_deg"};

static_assert(kResultKeys.size() == 1 + kResultNumbers.size() + kResultOptionals.size());
static_assert(kInputKeys.size() == 4 + kVehicleNumbers.size() + kExponentialNumbers.size());

// True for a result series column name.
bool is_series_column(std::string_view name) noexcept {
    for (const SeriesColumn& column : kSeriesColumns) {
        if (column.key == name) {
            return true;
        }
    }
    return false;
}

// Empty when every series key is a table or, with a result, a column.
std::optional<Error> unknown_series(const Experiment& experiment) {
    const bool with_result = !experiment.result.empty();
    for (const auto& entry : experiment.series) {
        const std::string_view key = entry.first;
        if (key == kTableMach || key == kTableCd || (with_result && is_series_column(key))) {
            continue;
        }
        return "series." + entry.first + ": unknown key";
    }
    return std::nullopt;
}

std::optional<Error> check_keys(const Experiment& experiment) {
    if (auto problem = Reader(experiment.inputs, "inputs").unknown_key(kInputKeys)) {
        return problem;
    }
    if (auto problem = Reader(experiment.result, "result").unknown_key(kResultKeys)) {
        return problem;
    }
    return unknown_series(experiment);
}

// Name, environment and vehicle into `out`; the first error, if any.
std::optional<Error> read_inputs(const Experiment& experiment, EntryExperiment& out) {
    const Reader inputs(experiment.inputs, "inputs");
    auto name = inputs.text("name", "");
    if (!name) {
        return name.error();
    }
    out.name = name.value();
    auto environment = read_environment(inputs);
    if (!environment) {
        return environment.error();
    }
    out.environment = environment.value();
    auto vehicle = read_vehicle(inputs, experiment.series);
    if (!vehicle) {
        return vehicle.error();
    }
    out.vehicle = vehicle.value();
    return std::nullopt;
}

// Conditions, solver settings and result into `out`.
std::optional<Error> read_rest(const Experiment& experiment, EntryExperiment& out) {
    auto conditions = read_conditions(experiment.conditions);
    if (!conditions) {
        return conditions.error();
    }
    out.conditions = conditions.value();
    auto solver = read_solver(experiment.solver);
    if (!solver) {
        return solver.error();
    }
    out.solver = solver.value();
    auto result = read_result(experiment);
    if (!result) {
        return result.error();
    }
    out.result = result.value();
    return std::nullopt;
}

} // namespace

Experiment to_experiment(const EntryExperiment& experiment) {
    Experiment out;
    out.domain = Domain::entry;
    write_inputs(experiment, out);
    write_conditions(experiment.conditions, out);
    write_solver(experiment.solver, out);
    if (experiment.result) {
        write_result(*experiment.result, out);
    }
    return out;
}

core::Result<EntryExperiment, std::string> entry_from_experiment(const Experiment& experiment) {
    if (experiment.domain != Domain::entry) {
        return core::failure("domain: expected 'entry', found '" +
                             std::string(to_string(experiment.domain)) + "'");
    }
    if (auto problem = check_keys(experiment)) {
        return core::failure(std::move(*problem));
    }
    EntryExperiment out;
    if (auto problem = read_inputs(experiment, out)) {
        return core::failure(std::move(*problem));
    }
    if (auto problem = read_rest(experiment, out)) {
        return core::failure(std::move(*problem));
    }
    return out;
}

core::Result<EntryExperiment, std::string> parse_entry_experiment(std::string_view json) {
    const auto parsed = parse_experiment(json);
    if (!parsed) {
        return core::failure(to_string(parsed.error()));
    }
    return entry_from_experiment(parsed.value());
}

std::string save_entry_experiment(const EntryExperiment& experiment) {
    return save_experiment(to_experiment(experiment));
}

core::Result<atmo::EntryConfig, std::string> to_entry_config(const EntryExperiment& experiment) {
    const EntryEnvironment& env = experiment.environment;
    if (env.planet != "earth" && env.planet != "mars") {
        return core::failure("inputs.planet: unknown planet '" + env.planet + "'");
    }
    atmo::EntryConfig c;
    c.planet = env.planet == "mars" ? atmo::Planet::mars() : atmo::Planet::earth();
    auto atmosphere = make_atmosphere(env);
    if (!atmosphere) {
        return core::failure(atmosphere.error());
    }
    c.planet.atmosphere = std::move(atmosphere).value();
    if (experiment.conditions.interface_altitude_m) {
        c.planet.entry_interface_altitude =
            core::si::Length(*experiment.conditions.interface_altitude_m);
    }
    auto vehicle = make_vehicle(experiment.vehicle);
    if (!vehicle) {
        return core::failure(vehicle.error());
    }
    c.vehicle = std::move(vehicle).value();
    c.initial = make_initial(experiment.conditions);
    c.frame = experiment.conditions.frame;
    const EntrySolverSettings& s = experiment.solver;
    c.integrator = s.integrator;
    c.ias15_tolerance = s.tolerance;
    c.time_step = core::si::Time(s.time_step_s);
    c.max_coast_step = core::si::Time(s.max_coast_step_s);
    c.max_duration = core::si::Time(s.max_duration_s);
    c.stop_on_skip_out = s.stop_on_skip_out;
    return c;
}

EntryResultRecord entry_result(const atmo::EntryExperiment& run) {
    EntryResultRecord r;
    const atmo::EntrySummary& s = run.summary();
    r.outcome = s.outcome;
    r.duration_s = s.duration_s;
    r.peak_g_load = s.peak_g_load;
    r.peak_g_time_s = s.peak_g_time_s;
    r.peak_heat_flux_w_m2 = s.peak_heat_flux_w_m2;
    r.peak_heat_flux_time_s = s.peak_heat_flux_time_s;
    r.peak_dynamic_pressure_pa = s.peak_dynamic_pressure_pa;
    r.peak_dynamic_pressure_time_s = s.peak_dynamic_pressure_time_s;
    r.heat_load_j_m2 = s.heat_load_j_m2;
    r.min_altitude_m = s.min_altitude_m;
    r.final_speed_m_s = s.final_speed_m_s;
    const atmo::EntryEnergyBalance b = run.energy_balance();
    r.drag_work_j = b.drag_work_j;
    r.energy_change_j = b.energy_change_j;
    r.energy_relative_error = b.relative_error;
    record_events(run.events(), r);
    r.series = run.series();
    return r;
}

} // namespace orrery::scene
