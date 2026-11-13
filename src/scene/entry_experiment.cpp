#include "orrery/scene/entry_experiment.h"

#include <array>
#include <string>

namespace orrery::scene {

namespace {

struct ModelName {
    EntryAtmosphereModel model;
    std::string_view name;
};

constexpr std::array<ModelName, 3> kModels{{
    {EntryAtmosphereModel::us76, "us76"},
    {EntryAtmosphereModel::mars, "mars"},
    {EntryAtmosphereModel::exponential, "exponential"},
}};

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

// Interface stub: the full mapping arrives with the implementation commit.
Experiment to_experiment(const EntryExperiment& experiment) {
    Experiment out;
    out.domain = Domain::entry;
    if (!experiment.name.empty()) {
        out.inputs.emplace("name", experiment.name);
    }
    out.solver.name = std::string(atmo::to_string(experiment.solver.integrator));
    out.solver.version = std::string(kEntrySolverVersion);
    return out;
}

core::Result<EntryExperiment, std::string> entry_from_experiment(const Experiment& experiment) {
    if (experiment.domain != Domain::entry) {
        return core::failure("domain: expected 'entry', found '" +
                             std::string(to_string(experiment.domain)) + "'");
    }
    return EntryExperiment{};
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

core::Result<atmo::EntryConfig, std::string>
to_entry_config(const EntryExperiment& /*experiment*/) {
    return atmo::EntryConfig{};
}

EntryResultRecord entry_result(const atmo::EntryExperiment& run) {
    EntryResultRecord r;
    r.outcome = run.outcome();
    r.series = run.series();
    return r;
}

} // namespace orrery::scene
