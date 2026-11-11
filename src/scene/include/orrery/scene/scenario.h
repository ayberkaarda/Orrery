#pragma once

#include <cstdint>
#include <filesystem>
#include <iosfwd>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "orrery/core/body.h"
#include "orrery/core/kepler.h"
#include "orrery/core/quantity.h"
#include "orrery/core/result.h"
#include "orrery/core/sim_state.h"
#include "orrery/core/vec3.h"
#include "orrery/scene/error.h"
#include "orrery/scene/limits.h"

namespace orrery::scene {

// Scenario schema version written by save_scenario and the only one accepted
// by parse_scenario.
inline constexpr int scenario_schema_version = 1;

// Angle unit declared by a scenario for its orbital elements. Lengths, masses
// and times are always AU, solar mass and day (ADR-001).
enum class AngleUnit : std::uint8_t {
    degrees,
    radians,
};

// Provenance of the initial conditions.
struct ScenarioMetadata {
    // Where the data comes from (catalogue, ephemeris, paper, "synthetic").
    std::string source;
    // Epoch the state refers to, free text such as "J2000.0 TDB".
    std::string epoch;
    // Optional human-readable description.
    std::string description;

    friend bool operator==(const ScenarioMetadata&, const ScenarioMetadata&) = default;
};

// Absolute position and velocity in the scenario's inertial frame.
struct CartesianSpec {
    core::Vec3<core::space::Length> position{};
    core::Vec3<core::space::Velocity> velocity{};
};

// Osculating elements relative to the named parent body, with
// mu = G (M_parent + m_body).
struct KeplerianSpec {
    std::string parent;
    core::KeplerElements elements;
};

using OrbitSpec = std::variant<CartesianSpec, KeplerianSpec>;

struct BodySpec {
    std::string name;
    core::BodyKindTag kind = core::BodyKindTag::planet;
    core::space::Mass mass{};
    // Black holes only, in [0, 1].
    double spin = 0.0;
    // Probes only: name of the aerodynamic coefficient table.
    std::optional<std::string> aero_ref;
    OrbitSpec orbit = CartesianSpec{};
};

// In-memory form of a scenario document.
struct Scenario {
    std::string name;
    ScenarioMetadata metadata;
    // Angle unit used when the scenario is written back to JSON. Elements are
    // always held in radians in memory.
    AngleUnit angle_unit = AngleUnit::radians;
    // Simulation time, days since the epoch.
    core::space::Time time{};
    std::vector<BodySpec> bodies;
};

// A scenario resolved into simulation state. Probes reference their aero table
// through core::AeroRef: value 0 means no table, value k refers to
// aero_tables[k - 1].
struct Scene {
    std::string name;
    ScenarioMetadata metadata;
    core::SimState state;
    std::vector<std::string> aero_tables;
};

// Checks names, kinds, masses, spins, eccentricities, parents, parent cycles
// and the body and aero-table limits of orrery/scene/limits.h. parse_scenario
// and build_scene call this; it is exposed for scenarios assembled in code.
// Every function of this header that returns a Result or an optional error
// reports exceptions through it as well (out_of_memory, internal_error);
// only save_scenario, which returns plain text, can throw std::bad_alloc.
[[nodiscard]] std::optional<SceneError> validate(const Scenario& scenario);

// Parses and validates a scenario document.
[[nodiscard]] core::Result<Scenario, SceneError> parse_scenario(std::string_view json);

// Resolves Keplerian elements (parents first) and fills a BodyStore in the
// order of scenario.bodies. Kepler and body store errors are propagated with
// the path of the offending body.
[[nodiscard]] core::Result<Scene, SceneError> build_scene(const Scenario& scenario);

// parse_scenario followed by build_scene.
[[nodiscard]] core::Result<Scene, SceneError> load_scene(std::string_view json);
[[nodiscard]] core::Result<Scene, SceneError> load_scene_file(const std::filesystem::path& path);

// Reads a whole file. io_error if it cannot be opened or read,
// limit_exceeded beyond limits::max_document_bytes. An empty file reads as an
// empty string.
[[nodiscard]] core::Result<std::string, SceneError>
read_text_file(const std::filesystem::path& path);

// Reads a stream to its end, as read_text_file does for a file. A read error
// of the underlying stream buffer (badbit, or a failure that is not the end of
// the stream) is an io_error naming `name`; nothing read before it is
// returned.
[[nodiscard]] core::Result<std::string, SceneError> read_text_stream(std::istream& in,
                                                                     std::string_view name);

// Writes a scenario as JSON. Numbers use the shortest representation that
// reads back to the same double, so parse_scenario(save_scenario(s))
// reproduces s exactly when angles are in radians. Non-finite numbers cannot
// be represented in JSON and are written as null (rejected on load).
[[nodiscard]] std::string save_scenario(const Scenario& scenario);

// Converts a scene back to a scenario with Cartesian states only. Fails with
// invalid_value if a probe refers to an aero table outside aero_tables.
[[nodiscard]] core::Result<Scenario, SceneError> to_scenario(const Scene& scene);

// to_scenario followed by save_scenario.
[[nodiscard]] core::Result<std::string, SceneError> save_scene(const Scene& scene);

} // namespace orrery::scene
