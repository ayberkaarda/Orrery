#include "orrery/sim/space_experiment.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <optional>
#include <system_error>
#include <utility>
#include <variant>

#include "orrery/scene/error.h"
#include "orrery/scene/experiment.h"

namespace orrery::sim {

namespace {

// Rejects keys outside `allowed` (the experiment format itself accepts any
// parameter name; a space experiment knows exactly which ones it reads).
std::optional<std::string> unknown_key(const scene::Parameters& parameters,
                                       std::span<const std::string_view> allowed,
                                       std::string_view where) {
    for (const auto& [key, value] : parameters) {
        if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
            std::string list;
            for (const std::string_view name : allowed) {
                list += (list.empty() ? "" : ", ") + std::string(name);
            }
            return std::string(where) + "." + key +
                   ": unknown key (allowed: " + (list.empty() ? std::string("none") : list) + ")";
        }
    }
    return std::nullopt;
}

core::Result<std::optional<double>, std::string>
number(const scene::Parameters& parameters, std::string_view key, std::string_view where) {
    const auto it = parameters.find(key);
    if (it == parameters.end()) {
        return std::optional<double>{};
    }
    if (const auto* value = std::get_if<double>(&it->second)) {
        return std::optional<double>(*value);
    }
    return core::failure(std::string(where) + "." + std::string(key) + ": expected a number");
}

core::Result<std::optional<std::string>, std::string>
text(const scene::Parameters& parameters, std::string_view key, std::string_view where) {
    const auto it = parameters.find(key);
    if (it == parameters.end()) {
        return std::optional<std::string>{};
    }
    if (const auto* value = std::get_if<std::string>(&it->second)) {
        return std::optional<std::string>(*value);
    }
    return core::failure(std::string(where) + "." + std::string(key) + ": expected a string");
}

// An integral number in [low, high].
core::Result<std::int64_t, std::string> integral(double value, double low, double high,
                                                 std::string_view what) {
    if (!(value >= low && value <= high) || std::floor(value) != value) {
        return core::failure(std::string(what) + " must be an integer in [" +
                             std::to_string(static_cast<std::int64_t>(low)) + ", " +
                             std::to_string(static_cast<std::int64_t>(high)) + "]");
    }
    return static_cast<std::int64_t>(value);
}

core::Result<SimulationConfig, std::string> read_config(const scene::Experiment& experiment) {
    constexpr std::array<std::string_view, 2> kConditions{"paused", "time_warp"};
    constexpr std::array<std::string_view, 4> kSettings{"dt_days", "force", "max_steps_per_frame",
                                                        "softening_au"};
    if (auto problem = unknown_key(experiment.conditions, kConditions, "conditions")) {
        return core::failure(std::move(*problem));
    }
    if (auto problem = unknown_key(experiment.solver.settings, kSettings, "solver.settings")) {
        return core::failure(std::move(*problem));
    }
    SimulationConfig config;
    const auto integrator = integrator_from_string(experiment.solver.name);
    if (!integrator) {
        return core::failure("solver.name: unknown integrator '" + experiment.solver.name +
                             "' (allowed: leapfrog_kdk, yoshida4, rk4, ias15)");
    }
    config.integrator = *integrator;

    const auto warp = number(experiment.conditions, "time_warp", "conditions");
    if (!warp) {
        return core::failure(warp.error());
    }
    if (const auto& requested = warp.value(); requested.has_value()) {
        const auto value = integral(*requested, static_cast<double>(kMinTimeWarp),
                                    static_cast<double>(kMaxTimeWarp), "conditions.time_warp");
        if (!value) {
            return core::failure(value.error());
        }
        config.time_warp = value.value();
    }
    const auto paused = number(experiment.conditions, "paused", "conditions");
    if (!paused) {
        return core::failure(paused.error());
    }
    if (const auto& requested = paused.value(); requested.has_value()) {
        const auto value = integral(*requested, 0.0, 1.0, "conditions.paused");
        if (!value) {
            return core::failure(value.error());
        }
        config.paused = value.value() == 1;
    }

    const scene::Parameters& settings = experiment.solver.settings;
    const auto dt = number(settings, "dt_days", "solver.settings");
    if (!dt) {
        return core::failure(dt.error());
    }
    config.dt_days = dt.value().value_or(config.dt_days);
    const auto softening = number(settings, "softening_au", "solver.settings");
    if (!softening) {
        return core::failure(softening.error());
    }
    config.softening_au = softening.value().value_or(config.softening_au);
    const auto budget = number(settings, "max_steps_per_frame", "solver.settings");
    if (!budget) {
        return core::failure(budget.error());
    }
    if (const auto& requested = budget.value(); requested.has_value()) {
        const auto value = integral(*requested, 1.0, 1.0e7, "solver.settings.max_steps_per_frame");
        if (!value) {
            return core::failure(value.error());
        }
        config.max_steps_per_advance = value.value();
    }
    const auto force = text(settings, "force", "solver.settings");
    if (!force) {
        return core::failure(force.error());
    }
    if (const auto& requested = force.value(); requested.has_value()) {
        const auto choice = force_from_string(*requested);
        if (!choice) {
            return core::failure("solver.settings.force: unknown force model '" + *requested +
                                 "' (allowed: auto, newtonian, barnes_hut, paczynski_wiita)");
        }
        config.force = *choice;
    }
    if (auto problem = validate(config)) {
        return core::failure("solver: " + *problem);
    }
    return config;
}

bool is_file(const std::filesystem::path& path) {
    std::error_code error;
    return std::filesystem::is_regular_file(path, error);
}

} // namespace

core::Result<SpaceExperiment, std::string>
load_space_experiment(const std::filesystem::path& path) {
    const std::string file = path.string();
    const auto text_read = scene::read_text_file(path);
    if (!text_read) {
        return core::failure("experiment " + file + ": " + scene::to_string(text_read.error()));
    }
    const auto parsed = scene::parse_experiment(text_read.value());
    if (!parsed) {
        return core::failure("experiment " + file + ": " + scene::to_string(parsed.error()));
    }
    const scene::Experiment& experiment = parsed.value();
    if (experiment.domain != scene::Domain::space) {
        return core::failure("experiment " + file + ": domain '" +
                             std::string(scene::to_string(experiment.domain)) +
                             "' is not a space experiment (aero and entry experiments open "
                             "in their own workspaces)");
    }
    constexpr std::array<std::string_view, 1> kInputs{"scenario"};
    if (auto problem = unknown_key(experiment.inputs, kInputs, "inputs")) {
        return core::failure("experiment " + file + ": " + *problem);
    }
    const auto scenario = text(experiment.inputs, "scenario", "inputs");
    if (!scenario) {
        return core::failure("experiment " + file + ": " + scenario.error());
    }
    const std::string scenario_name = scenario.value().value_or(std::string());
    if (scenario_name.empty()) {
        return core::failure("experiment " + file + ": inputs.scenario: missing scenario file");
    }
    auto config = read_config(experiment);
    if (!config) {
        return core::failure("experiment " + file + ": " + config.error());
    }

    std::filesystem::path scenario_path(scenario_name);
    if (scenario_path.is_relative()) {
        scenario_path = path.parent_path() / scenario_path;
    }
    scenario_path = scenario_path.lexically_normal();
    auto loaded = scene::load_scene_file(scenario_path);
    if (!loaded) {
        return core::failure("scenario " + scenario_path.string() + " (from experiment " + file +
                             "): " + scene::to_string(loaded.error()));
    }
    return SpaceExperiment{path, std::move(scenario_path), std::move(loaded).value(),
                           config.value()};
}

core::Result<std::filesystem::path, std::string>
resolve_experiment(std::string_view name_or_path,
                   std::span<const std::filesystem::path> search_dirs) {
    if (name_or_path.empty()) {
        return core::failure(std::string("empty experiment name"));
    }
    const std::filesystem::path given{std::string(name_or_path)};
    if (is_file(given)) {
        return given;
    }
    std::string tried = "\n  " + given.string();
    const bool has_extension = given.extension() == ".json";
    for (const std::filesystem::path& dir : search_dirs) {
        std::filesystem::path candidate = dir / given;
        if (!has_extension) {
            candidate += ".json";
        }
        if (is_file(candidate)) {
            return candidate;
        }
        tried += "\n  " + candidate.string();
    }
    return core::failure("experiment '" + std::string(name_or_path) +
                         "' not found; tried:" + tried);
}

std::vector<ScenarioEntry> list_scenarios(const std::filesystem::path& directory) {
    std::vector<ScenarioEntry> entries;
    std::error_code error;
    std::filesystem::directory_iterator it(directory, error);
    if (error) {
        return entries;
    }
    for (const std::filesystem::directory_iterator end; it != end; it.increment(error)) {
        if (error) {
            break;
        }
        const std::filesystem::path& path = it->path();
        if (path.extension() == ".json" && is_file(path)) {
            entries.push_back({path.stem().string(), path});
        }
    }
    std::sort(entries.begin(), entries.end(),
              [](const ScenarioEntry& a, const ScenarioEntry& b) { return a.name < b.name; });
    return entries;
}

namespace {

// `path` relative to `directory` when both are on the same root, else
// absolute; '/' separators either way.
std::string portable_reference(const std::filesystem::path& path,
                               const std::filesystem::path& directory) {
    std::error_code error;
    const std::filesystem::path target = std::filesystem::absolute(path, error).lexically_normal();
    if (error) {
        return path.generic_string();
    }
    const std::filesystem::path base =
        std::filesystem::absolute(directory, error).lexically_normal();
    if (!error && target.root_name() == base.root_name()) {
        const std::filesystem::path relative = target.lexically_relative(base);
        if (!relative.empty()) {
            return relative.generic_string();
        }
    }
    return target.generic_string();
}

std::string summary_of(const SpaceExperiment& experiment) {
    return experiment.scenario_path.stem().string() + ", " +
           std::string(to_string(experiment.config.integrator));
}

} // namespace

scene::Experiment space_record(const std::filesystem::path& scenario, const SimStatus& status,
                               double sim_time_days, const std::filesystem::path& directory) {
    scene::Experiment record;
    record.domain = scene::Domain::space;
    record.inputs["scenario"] = portable_reference(scenario, directory);
    record.conditions["time_warp"] = static_cast<double>(status.time_warp);
    record.conditions["paused"] = status.paused ? 1.0 : 0.0;
    record.solver.name = std::string(to_string(status.settings.integrator));
    record.solver.version = "1";
    record.solver.settings["dt_days"] = status.dt_days;
    record.solver.settings["force"] = std::string(to_string(status.settings.force));
    record.solver.settings["softening_au"] = status.settings.softening_au;
    record.solver.settings["max_steps_per_frame"] =
        static_cast<double>(status.settings.max_steps_per_advance);
    if (std::isfinite(sim_time_days)) {
        record.result["sim_time_days"] = sim_time_days;
    }
    record.result["steps"] = static_cast<double>(status.steps);
    record.result["events"] = static_cast<double>(status.events);
    if (status.energy_drift && std::isfinite(*status.energy_drift)) {
        record.result["energy_drift"] = *status.energy_drift;
    }
    return record;
}

core::Result<std::filesystem::path, std::string>
save_space_experiment_file(const std::filesystem::path& path, const scene::Experiment& record) {
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
        const std::string text = scene::save_experiment(record);
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

SpaceListing list_space_experiments(const std::filesystem::path& directory) {
    SpaceListing listing;
    for (const ScenarioEntry& file : list_scenarios(directory)) {
        const auto text = scene::read_text_file(file.path);
        if (!text) {
            listing.problems.push_back(file.path.filename().string() + ": " +
                                       scene::to_string(text.error()));
            continue;
        }
        const auto generic = scene::parse_experiment(text.value());
        if (!generic) {
            listing.problems.push_back(file.path.filename().string() + ": " +
                                       scene::to_string(generic.error()));
            continue;
        }
        if (generic.value().domain != scene::Domain::space) {
            continue;
        }
        const auto loaded = load_space_experiment(file.path);
        if (!loaded) {
            listing.problems.push_back(file.path.filename().string() + ": " + loaded.error());
            continue;
        }
        listing.entries.push_back(StoredSpace{file.path, file.name, summary_of(loaded.value())});
    }
    return listing;
}

} // namespace orrery::sim
