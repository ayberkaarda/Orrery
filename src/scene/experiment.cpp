#include "orrery/scene/experiment.h"

#include <array>
#include <cmath>
#include <optional>
#include <utility>

#include "json_support.h"

namespace orrery::scene {

namespace {

using detail::fail;
using detail::Json;
using detail::ObjectReader;
using detail::OrderedJson;

constexpr std::array<Domain, 3> kDomains{Domain::space, Domain::aero, Domain::entry};

std::optional<Domain> domain_from_name(std::string_view name) {
    for (const Domain domain : kDomains) {
        if (to_string(domain) == name) {
            return domain;
        }
    }
    return std::nullopt;
}

Parameters read_parameters(const ObjectReader& parent, std::string_view key) {
    Parameters out;
    if (!parent.has(key)) {
        return out;
    }
    const ObjectReader object(parent.required(key), parent.path_of(key));
    for (const auto& item : parent.required(key).items()) {
        const std::string path = object.path_of(item.key());
        const Json& value = item.value();
        if (value.is_string()) {
            out.emplace(item.key(), value.get<std::string>());
        } else if (value.is_number()) {
            const double number = value.get<double>();
            if (!std::isfinite(number)) {
                fail(SceneErrorCode::invalid_value, path, "value must be finite");
            }
            out.emplace(item.key(), number);
        } else {
            fail(SceneErrorCode::wrong_type, path,
                 "expected number or string, found " + std::string(detail::type_name(value)));
        }
    }
    return out;
}

Experiment read_experiment(const Json& document) {
    const ObjectReader root(document, "");
    const long long version = root.required_integer("version");
    if (version != experiment_schema_version) {
        fail(SceneErrorCode::unsupported_version, root.path_of("version"),
             "experiment schema version " + std::to_string(version) +
                 " is not supported (supported: " + std::to_string(experiment_schema_version) +
                 ")");
    }
    root.allow_only({"version", "domain", "inputs", "conditions", "solver", "result"});

    Experiment experiment;
    const std::string domain = root.required_string("domain");
    const auto parsed = domain_from_name(domain);
    if (!parsed) {
        fail(SceneErrorCode::invalid_value, root.path_of("domain"),
             "unsupported domain '" + domain + "' (allowed: space, aero, entry)");
    }
    experiment.domain = *parsed;
    experiment.inputs = read_parameters(root, "inputs");
    experiment.conditions = read_parameters(root, "conditions");

    const ObjectReader solver(root.required("solver"), root.path_of("solver"));
    solver.allow_only({"name", "version", "settings"});
    experiment.solver.name = solver.required_string("name");
    experiment.solver.version = solver.optional_string("version").value_or("");
    experiment.solver.settings = read_parameters(solver, "settings");

    experiment.result = read_parameters(root, "result");
    return experiment;
}

OrderedJson parameters_to_json(const Parameters& parameters) {
    OrderedJson out = OrderedJson::object();
    for (const auto& [key, value] : parameters) {
        if (const auto* number = std::get_if<double>(&value)) {
            out[key] = detail::number_to_json(*number);
        } else {
            out[key] = std::get<std::string>(value);
        }
    }
    return out;
}

} // namespace

std::string_view to_string(Domain domain) noexcept {
    switch (domain) {
        case Domain::space:
            return "space";
        case Domain::aero:
            return "aero";
        case Domain::entry:
            return "entry";
    }
    return "unknown";
}

core::Result<Experiment, SceneError> parse_experiment(std::string_view json) {
    return detail::guarded<Experiment>(
        [json] { return read_experiment(detail::parse_or_throw(json).value()); });
}

std::string save_experiment(const Experiment& experiment) {
    OrderedJson root = OrderedJson::object();
    root["version"] = experiment_schema_version;
    root["domain"] = std::string(to_string(experiment.domain));
    root["inputs"] = parameters_to_json(experiment.inputs);
    root["conditions"] = parameters_to_json(experiment.conditions);
    OrderedJson solver = OrderedJson::object();
    solver["name"] = experiment.solver.name;
    solver["version"] = experiment.solver.version;
    solver["settings"] = parameters_to_json(experiment.solver.settings);
    root["solver"] = std::move(solver);
    root["result"] = parameters_to_json(experiment.result);
    return root.dump(2) + "\n";
}

} // namespace orrery::scene
