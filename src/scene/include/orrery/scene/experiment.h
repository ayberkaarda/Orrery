#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "orrery/core/result.h"
#include "orrery/scene/error.h"

namespace orrery::scene {

// Experiment schema version written by save_experiment and the only one
// accepted by parse_experiment.
inline constexpr int experiment_schema_version = 1;

// Domain of an experiment. The JSON field "domain" accepts exactly these
// labels: "space", "aero", "entry".
enum class Domain : std::uint8_t {
    space,
    aero,
    entry,
};

[[nodiscard]] std::string_view to_string(Domain domain) noexcept;

// A named scalar or text value. Keys are kept sorted so that serialisation is
// deterministic.
using ParameterValue = std::variant<double, std::string>;
using Parameters = std::map<std::string, ParameterValue, std::less<>>;

// Named series of finite numbers, e.g. a convergence history. Keys are kept
// sorted like Parameters.
using Series = std::map<std::string, std::vector<double>, std::less<>>;

struct SolverInfo {
    std::string name;
    std::string version;
    Parameters settings;

    friend bool operator==(const SolverInfo&, const SolverInfo&) = default;
};

// Record of one run: what went in, under which conditions, which solver
// produced which result.
struct Experiment {
    Domain domain = Domain::space;
    // References to the inputs, e.g. a scenario or geometry file.
    Parameters inputs;
    // Physical conditions, e.g. Mach number or angle of attack.
    Parameters conditions;
    SolverInfo solver;
    Parameters result;
    // Optional series of the result (JSON key "series", added in Phase 8
    // without a schema version change: files without it load unchanged and
    // an empty map is not written).
    Series series;

    friend bool operator==(const Experiment&, const Experiment&) = default;
};

[[nodiscard]] core::Result<Experiment, SceneError> parse_experiment(std::string_view json);

// Shortest round-trip number formatting: parse_experiment(save_experiment(e))
// == e for finite values (series values must be finite as well).
[[nodiscard]] std::string save_experiment(const Experiment& experiment);

} // namespace orrery::scene
