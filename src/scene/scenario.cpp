#include "orrery/scene/scenario.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <ios>
#include <istream>
#include <map>
#include <optional>
#include <ranges>
#include <set>
#include <streambuf>
#include <string_view>
#include <utility>

#include "json_support.h"
#include "orrery/core/angle.h"
#include "orrery/core/units.h"

namespace orrery::scene {

namespace {

using detail::fail;
using detail::Json;
using detail::ObjectReader;
using detail::OrderedJson;
using detail::SceneFailure;

namespace space = core::space;

struct KindName {
    core::BodyKindTag tag;
    std::string_view name;
};

// Kind labels of the scenario format. They differ from core::to_string for
// black holes ("blackhole" here, "black_hole" in core).
constexpr std::array<KindName, 6> kKindNames{{
    {core::BodyKindTag::star, "star"},
    {core::BodyKindTag::planet, "planet"},
    {core::BodyKindTag::moon, "moon"},
    {core::BodyKindTag::asteroid, "asteroid"},
    {core::BodyKindTag::black_hole, "blackhole"},
    {core::BodyKindTag::probe, "probe"},
}};

std::string_view kind_name(core::BodyKindTag tag) {
    for (const KindName& entry : kKindNames) {
        if (entry.tag == tag) {
            return entry.name;
        }
    }
    return "unknown";
}

std::optional<core::BodyKindTag> kind_from_name(std::string_view name) {
    for (const KindName& entry : kKindNames) {
        if (entry.name == name) {
            return entry.tag;
        }
    }
    return std::nullopt;
}

bool requires_positive_mass(core::BodyKindTag tag) {
    return tag == core::BodyKindTag::probe || tag == core::BodyKindTag::black_hole;
}

// Keys of the "elements" object, in output order.
constexpr std::string_view kKeyA = "a";
constexpr std::string_view kKeyE = "e";
constexpr std::string_view kKeyI = "i";
constexpr std::string_view kKeyNode = "raan";
constexpr std::string_view kKeyPeri = "argp";
constexpr std::string_view kKeyMean = "ma";

std::string body_path(std::size_t index) {
    return detail::index_path("bodies", index);
}

std::string elements_path(std::size_t index, std::string_view key) {
    return detail::child_path(detail::child_path(body_path(index), "elements"), key);
}

// ---------------------------------------------------------------------------
// Validation of the in-memory scenario
// ---------------------------------------------------------------------------

void require_finite(double value, const std::string& path) {
    if (!std::isfinite(value)) {
        fail(SceneErrorCode::invalid_value, path, "value must be finite");
    }
}

template <typename Q> void require_finite_vector(const core::Vec3<Q>& v, const std::string& path) {
    require_finite(v.x.value(), detail::index_path(path, 0));
    require_finite(v.y.value(), detail::index_path(path, 1));
    require_finite(v.z.value(), detail::index_path(path, 2));
}

void validate_body(const BodySpec& body, std::size_t index) {
    const std::string path = body_path(index);
    if (body.name.empty()) {
        fail(SceneErrorCode::invalid_value, detail::child_path(path, "name"),
             "body name must not be empty");
    }
    if (kind_name(body.kind) == "unknown") {
        fail(SceneErrorCode::invalid_value, detail::child_path(path, "kind"), "unknown body kind");
    }

    const std::string mass_path = detail::child_path(path, "mass");
    const double mass = body.mass.value();
    if (!std::isfinite(mass)) {
        fail(SceneErrorCode::invalid_value, mass_path, "mass must be finite");
    }
    if (mass < 0.0) {
        fail(SceneErrorCode::invalid_value, mass_path, "mass must not be negative");
    }
    if (mass == 0.0 && requires_positive_mass(body.kind)) {
        fail(SceneErrorCode::invalid_value, mass_path,
             "mass of a " + std::string(kind_name(body.kind)) + " must be positive");
    }

    const std::string spin_path = detail::child_path(path, "spin");
    if (body.kind == core::BodyKindTag::black_hole) {
        if (!(body.spin >= 0.0 && body.spin <= 1.0)) {
            fail(SceneErrorCode::invalid_value, spin_path, "black hole spin must lie in [0, 1]");
        }
    } else if (body.spin != 0.0) {
        fail(SceneErrorCode::invalid_value, spin_path, "only black holes have a spin");
    }

    if (body.aero_ref) {
        const std::string aero_path = detail::child_path(path, "aero_ref");
        if (body.kind != core::BodyKindTag::probe) {
            fail(SceneErrorCode::invalid_value, aero_path, "only probes have an aero_ref");
        }
        if (body.aero_ref->empty()) {
            fail(SceneErrorCode::invalid_value, aero_path, "aero_ref must not be empty");
        }
    }

    if (const auto* cartesian = std::get_if<CartesianSpec>(&body.orbit)) {
        const std::string state_path = detail::child_path(path, "state");
        require_finite_vector(cartesian->position, detail::child_path(state_path, "position"));
        require_finite_vector(cartesian->velocity, detail::child_path(state_path, "velocity"));
        return;
    }
    const auto& keplerian = std::get<KeplerianSpec>(body.orbit);
    const core::KeplerElements& el = keplerian.elements;
    require_finite(el.semi_major_axis.value(), elements_path(index, kKeyA));
    require_finite(el.eccentricity, elements_path(index, kKeyE));
    require_finite(el.inclination.radians(), elements_path(index, kKeyI));
    require_finite(el.longitude_of_ascending_node.radians(), elements_path(index, kKeyNode));
    require_finite(el.argument_of_periapsis.radians(), elements_path(index, kKeyPeri));
    require_finite(el.mean_anomaly.radians(), elements_path(index, kKeyMean));
    if (el.eccentricity < 0.0) {
        fail(SceneErrorCode::invalid_value, elements_path(index, kKeyE),
             "eccentricity must not be negative");
    }
}

// Index of every body by name; fails on duplicates.
std::map<std::string, std::size_t, std::less<>> index_names(const Scenario& scenario) {
    std::map<std::string, std::size_t, std::less<>> by_name;
    for (std::size_t i = 0; i < scenario.bodies.size(); ++i) {
        const auto [it, inserted] = by_name.emplace(scenario.bodies[i].name, i);
        if (!inserted) {
            fail(SceneErrorCode::duplicate_name, detail::child_path(body_path(i), "name"),
                 "body name '" + scenario.bodies[i].name + "' is already used by " +
                     body_path(it->second));
        }
    }
    return by_name;
}

// Parent index of each body, or nullopt for Cartesian bodies. Fails on
// unknown parents, non-gravitating parents and cycles.
std::vector<std::optional<std::size_t>>
resolve_parents(const Scenario& scenario,
                const std::map<std::string, std::size_t, std::less<>>& by_name) {
    const std::size_t n = scenario.bodies.size();
    std::vector<std::optional<std::size_t>> parent(n);
    for (std::size_t i = 0; i < n; ++i) {
        const auto* keplerian = std::get_if<KeplerianSpec>(&scenario.bodies[i].orbit);
        if (keplerian == nullptr) {
            continue;
        }
        const std::string path = elements_path(i, "parent");
        const auto it = by_name.find(keplerian->parent);
        if (it == by_name.end()) {
            fail(SceneErrorCode::unknown_parent, path,
                 "parent '" + keplerian->parent + "' is not a body of this scenario");
        }
        if (scenario.bodies[it->second].kind == core::BodyKindTag::probe) {
            fail(SceneErrorCode::invalid_value, path,
                 "parent '" + keplerian->parent + "' is a probe and exerts no gravity");
        }
        parent[i] = it->second;
    }

    // Walk each chain; with state 0 = unvisited, 1 = on the current chain,
    // 2 = known to end at a Cartesian body.
    std::vector<int> mark(n, 0);
    for (std::size_t start = 0; start < n; ++start) {
        std::vector<std::size_t> chain;
        std::optional<std::size_t> current = start;
        while (current && mark[*current] == 0) {
            mark[*current] = 1;
            chain.push_back(*current);
            current = parent[*current];
        }
        if (current && mark[*current] == 1) {
            // The chain reached a body already on it: report the cycle.
            const auto first = std::ranges::find(chain, *current);
            std::string cycle;
            for (auto it = first; it != chain.end(); ++it) {
                cycle += scenario.bodies[*it].name + " -> ";
            }
            cycle += scenario.bodies[*current].name;
            fail(SceneErrorCode::parent_cycle, elements_path(*current, "parent"),
                 "parent chain forms a cycle: " + cycle);
        }
        for (const std::size_t i : chain) {
            mark[i] = 2;
        }
    }
    return parent;
}

void require_body_count(std::size_t count) {
    if (count > limits::max_bodies) {
        fail(SceneErrorCode::limit_exceeded, "bodies",
             std::to_string(count) + " bodies exceed the limit of " +
                 std::to_string(limits::max_bodies));
    }
}

// Distinct aero_ref names become AeroRef values 1..n; n stays within
// limits::max_aero_tables, far below the 2^32 range of AeroRef.
void require_aero_table_count(const Scenario& scenario) {
    std::set<std::string_view> names;
    for (std::size_t i = 0; i < scenario.bodies.size(); ++i) {
        const auto& ref = scenario.bodies[i].aero_ref;
        if (ref && names.insert(*ref).second && names.size() > limits::max_aero_tables) {
            fail(SceneErrorCode::limit_exceeded, detail::child_path(body_path(i), "aero_ref"),
                 "more than " + std::to_string(limits::max_aero_tables) +
                     " distinct aero_ref names");
        }
    }
}

void validate_or_throw(const Scenario& scenario) {
    require_body_count(scenario.bodies.size());
    if (!std::isfinite(scenario.time.value())) {
        fail(SceneErrorCode::invalid_value, "time", "value must be finite");
    }
    for (std::size_t i = 0; i < scenario.bodies.size(); ++i) {
        validate_body(scenario.bodies[i], i);
    }
    require_aero_table_count(scenario);
    const auto by_name = index_names(scenario);
    (void)resolve_parents(scenario, by_name);
}

// ---------------------------------------------------------------------------
// JSON -> Scenario
// ---------------------------------------------------------------------------

template <typename Q> core::Vec3<Q> read_vector(const Json& value, const std::string& path) {
    if (!value.is_array()) {
        detail::fail(SceneErrorCode::wrong_type, path,
                     "expected array of 3 numbers, found " + std::string(detail::type_name(value)));
    }
    if (value.size() != 3) {
        fail(SceneErrorCode::invalid_value, path,
             "expected 3 components, found " + std::to_string(value.size()));
    }
    return core::Vec3<Q>{Q(detail::as_number(value[0], detail::index_path(path, 0))),
                         Q(detail::as_number(value[1], detail::index_path(path, 1))),
                         Q(detail::as_number(value[2], detail::index_path(path, 2)))};
}

core::Angle read_angle(const ObjectReader& reader, std::string_view key, AngleUnit unit) {
    const double v = reader.required_number(key);
    return unit == AngleUnit::degrees ? core::Angle::from_degrees(v) : core::Angle::from_radians(v);
}

void read_units(const ObjectReader& root, Scenario& scenario) {
    const ObjectReader units(root.required("units"), root.path_of("units"));
    units.allow_only({"length", "mass", "time", "angle"});
    const auto expect = [&units](std::string_view key, std::string_view wanted) {
        const std::string value = units.required_string(key);
        if (value != wanted) {
            fail(SceneErrorCode::invalid_value, units.path_of(key),
                 "unsupported unit '" + value + "', expected '" + std::string(wanted) + "'");
        }
    };
    expect("length", "AU");
    expect("mass", "Msun");
    expect("time", "day");
    const std::string angle = units.required_string("angle");
    if (angle == "deg") {
        scenario.angle_unit = AngleUnit::degrees;
    } else if (angle == "rad") {
        scenario.angle_unit = AngleUnit::radians;
    } else {
        fail(SceneErrorCode::invalid_value, units.path_of("angle"),
             "unsupported angle unit '" + angle + "', expected 'deg' or 'rad'");
    }
}

void read_metadata(const ObjectReader& root, Scenario& scenario) {
    const ObjectReader meta(root.required("metadata"), root.path_of("metadata"));
    meta.allow_only({"source", "epoch", "description"});
    scenario.metadata.source = meta.required_string("source");
    scenario.metadata.epoch = meta.required_string("epoch");
    scenario.metadata.description = meta.optional_string("description").value_or("");
}

BodySpec read_body(const Json& value, const std::string& path, AngleUnit angle_unit) {
    const ObjectReader body(value, path);
    BodySpec spec;
    spec.name = body.required_string("name");
    const std::string kind = body.required_string("kind");
    const auto tag = kind_from_name(kind);
    if (!tag) {
        fail(SceneErrorCode::invalid_value, body.path_of("kind"),
             "unknown kind '" + kind +
                 "' (allowed: star, planet, moon, asteroid, blackhole, probe)");
    }
    spec.kind = *tag;

    if (spec.kind == core::BodyKindTag::black_hole) {
        body.allow_only({"name", "kind", "mass", "spin", "state", "elements"});
    } else if (spec.kind == core::BodyKindTag::probe) {
        body.allow_only({"name", "kind", "mass", "aero_ref", "state", "elements"});
    } else {
        body.allow_only({"name", "kind", "mass", "state", "elements"});
    }

    spec.mass = space::Mass(body.required_number("mass"));
    spec.spin = body.optional_number("spin").value_or(0.0);
    spec.aero_ref = body.optional_string("aero_ref");

    const bool has_state = body.has("state");
    const bool has_elements = body.has("elements");
    if (has_state == has_elements) {
        fail(has_state ? SceneErrorCode::invalid_value : SceneErrorCode::missing_key, path,
             "a body needs exactly one of 'state' (Cartesian) or 'elements' (Keplerian)");
    }
    if (has_state) {
        const ObjectReader state(body.required("state"), body.path_of("state"));
        state.allow_only({"position", "velocity"});
        spec.orbit = CartesianSpec{
            read_vector<space::Length>(state.required("position"), state.path_of("position")),
            read_vector<space::Velocity>(state.required("velocity"), state.path_of("velocity"))};
        return spec;
    }
    const ObjectReader elements(body.required("elements"), body.path_of("elements"));
    elements.allow_only({"parent", kKeyA, kKeyE, kKeyI, kKeyNode, kKeyPeri, kKeyMean});
    KeplerianSpec keplerian;
    keplerian.parent = elements.required_string("parent");
    keplerian.elements.semi_major_axis = space::Length(elements.required_number(kKeyA));
    keplerian.elements.eccentricity = elements.required_number(kKeyE);
    keplerian.elements.inclination = read_angle(elements, kKeyI, angle_unit);
    keplerian.elements.longitude_of_ascending_node = read_angle(elements, kKeyNode, angle_unit);
    keplerian.elements.argument_of_periapsis = read_angle(elements, kKeyPeri, angle_unit);
    keplerian.elements.mean_anomaly = read_angle(elements, kKeyMean, angle_unit);
    spec.orbit = std::move(keplerian);
    return spec;
}

Scenario read_scenario(const Json& document) {
    const ObjectReader root(document, "");
    // The version decides the schema, so it is checked before any other key.
    const long long version = root.required_integer("version");
    if (version != scenario_schema_version) {
        fail(SceneErrorCode::unsupported_version, root.path_of("version"),
             "scenario schema version " + std::to_string(version) +
                 " is not supported (supported: " + std::to_string(scenario_schema_version) + ")");
    }
    // Checked before any body is read, so an oversized array costs no more
    // than its (already bounded) JSON form.
    if (root.has("bodies") && root.required("bodies").is_array()) {
        require_body_count(root.required("bodies").size());
    }
    root.allow_only({"version", "name", "units", "metadata", "time", "bodies"});

    Scenario scenario;
    scenario.name = root.optional_string("name").value_or("");
    read_units(root, scenario);
    read_metadata(root, scenario);
    scenario.time = space::Time(root.optional_number("time").value_or(0.0));

    const Json& bodies = root.required("bodies");
    if (!bodies.is_array()) {
        fail(SceneErrorCode::wrong_type, "bodies",
             "expected array, found " + std::string(detail::type_name(bodies)));
    }
    scenario.bodies.reserve(bodies.size());
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        scenario.bodies.push_back(read_body(bodies[i], body_path(i), scenario.angle_unit));
    }
    validate_or_throw(scenario);
    return scenario;
}

// ---------------------------------------------------------------------------
// Scenario -> Scene
// ---------------------------------------------------------------------------

std::string kepler_error_path(std::size_t index, core::KeplerError error) {
    switch (error) {
        case core::KeplerError::invalid_eccentricity:
        case core::KeplerError::parabolic_orbit:
            return elements_path(index, kKeyE);
        case core::KeplerError::invalid_inclination:
            return elements_path(index, kKeyI);
        case core::KeplerError::inconsistent_semi_major_axis:
            return elements_path(index, kKeyA);
        case core::KeplerError::invalid_gravitational_parameter:
            return elements_path(index, "parent");
        case core::KeplerError::non_finite_input:
        case core::KeplerError::degenerate_state:
        case core::KeplerError::beyond_asymptote:
        case core::KeplerError::no_convergence:
            break;
    }
    return detail::child_path(body_path(index), "elements");
}

core::BodyKind make_kind(const BodySpec& body, core::AeroRef aero) {
    switch (body.kind) {
        case core::BodyKindTag::star:
            return core::Star{};
        case core::BodyKindTag::planet:
            return core::Planet{};
        case core::BodyKindTag::moon:
            return core::Moon{};
        case core::BodyKindTag::asteroid:
            return core::Asteroid{};
        case core::BodyKindTag::black_hole:
            return core::BlackHole{body.spin};
        case core::BodyKindTag::probe:
            return core::Probe{aero};
    }
    return core::Planet{};
}

Scene build_or_throw(const Scenario& scenario) {
    validate_or_throw(scenario);
    const auto parents = resolve_parents(scenario, index_names(scenario));
    const std::size_t n = scenario.bodies.size();

    // Absolute states, resolved parents first (the parent graph is acyclic).
    std::vector<std::optional<core::CartesianState>> absolute(n);
    for (std::size_t start = 0; start < n; ++start) {
        std::vector<std::size_t> chain;
        for (std::optional<std::size_t> i = start; i && !absolute[*i]; i = parents[*i]) {
            chain.push_back(*i);
        }
        for (const std::size_t i : std::views::reverse(chain)) {
            const BodySpec& body = scenario.bodies[i];
            if (const auto* cartesian = std::get_if<CartesianSpec>(&body.orbit)) {
                absolute[i] = core::CartesianState{cartesian->position, cartesian->velocity};
                continue;
            }
            const auto& keplerian = std::get<KeplerianSpec>(body.orbit);
            const std::size_t p = *parents[i];
            const space::GravitationalParameter mu = core::constants::gravitational_constant_space *
                                                     (scenario.bodies[p].mass + body.mass);
            const auto relative = core::to_cartesian(keplerian.elements, mu);
            if (!relative) {
                throw SceneFailure{
                    SceneError{SceneErrorCode::kepler_error,
                               kepler_error_path(i, relative.error()),
                               "elements relative to '" + keplerian.parent +
                                   "' rejected: " + std::string(core::to_string(relative.error())),
                               relative.error(),
                               {}}};
            }
            const core::CartesianState& origin = *absolute[p];
            absolute[i] = core::CartesianState{origin.position + relative.value().position,
                                               origin.velocity + relative.value().velocity};
        }
    }

    Scene scene;
    scene.name = scenario.name;
    scene.metadata = scenario.metadata;
    scene.state.set_time(scenario.time);
    // AeroRef value of each table name, numbered in order of first use. The
    // validated count (limits::max_aero_tables) keeps the cast exact.
    std::map<std::string_view, std::uint32_t, std::less<>> aero_numbers;
    for (std::size_t i = 0; i < n; ++i) {
        const BodySpec& body = scenario.bodies[i];
        core::AeroRef aero{};
        if (body.aero_ref) {
            const auto [it, inserted] = aero_numbers.emplace(
                *body.aero_ref, static_cast<std::uint32_t>(scene.aero_tables.size() + 1));
            if (inserted) {
                scene.aero_tables.push_back(*body.aero_ref);
            }
            aero.value = it->second;
        }
        const std::optional<core::CartesianState>& resolved = absolute[i];
        if (!resolved.has_value()) {
            throw SceneFailure{
                SceneError{SceneErrorCode::internal_error, {}, "body state unresolved", {}, {}}};
        }
        const auto added = scene.state.bodies().add(core::BodyInit{
            body.name, make_kind(body, aero), body.mass, resolved->position, resolved->velocity});
        if (!added) {
            throw SceneFailure{
                SceneError{SceneErrorCode::body_error,
                           body_path(i),
                           "body rejected: " + std::string(core::to_string(added.error())),
                           {},
                           added.error()}};
        }
    }
    return scene;
}

// ---------------------------------------------------------------------------
// Scenario -> JSON
// ---------------------------------------------------------------------------

template <typename Q> OrderedJson vector_to_json(const core::Vec3<Q>& v) {
    OrderedJson out = OrderedJson::array();
    out.push_back(detail::number_to_json(v.x.value()));
    out.push_back(detail::number_to_json(v.y.value()));
    out.push_back(detail::number_to_json(v.z.value()));
    return out;
}

double angle_out(core::Angle angle, AngleUnit unit) {
    return unit == AngleUnit::degrees ? angle.degrees() : angle.radians();
}

OrderedJson body_to_json(const BodySpec& body, AngleUnit unit) {
    OrderedJson out = OrderedJson::object();
    out["name"] = body.name;
    out["kind"] = std::string(kind_name(body.kind));
    out["mass"] = detail::number_to_json(body.mass.value());
    if (body.kind == core::BodyKindTag::black_hole) {
        out["spin"] = detail::number_to_json(body.spin);
    }
    if (body.aero_ref) {
        out["aero_ref"] = *body.aero_ref;
    }
    if (const auto* cartesian = std::get_if<CartesianSpec>(&body.orbit)) {
        OrderedJson state = OrderedJson::object();
        state["position"] = vector_to_json(cartesian->position);
        state["velocity"] = vector_to_json(cartesian->velocity);
        out["state"] = std::move(state);
        return out;
    }
    const auto& keplerian = std::get<KeplerianSpec>(body.orbit);
    const core::KeplerElements& el = keplerian.elements;
    OrderedJson elements = OrderedJson::object();
    elements["parent"] = keplerian.parent;
    elements[std::string(kKeyA)] = detail::number_to_json(el.semi_major_axis.value());
    elements[std::string(kKeyE)] = detail::number_to_json(el.eccentricity);
    elements[std::string(kKeyI)] = detail::number_to_json(angle_out(el.inclination, unit));
    elements[std::string(kKeyNode)] =
        detail::number_to_json(angle_out(el.longitude_of_ascending_node, unit));
    elements[std::string(kKeyPeri)] =
        detail::number_to_json(angle_out(el.argument_of_periapsis, unit));
    elements[std::string(kKeyMean)] = detail::number_to_json(angle_out(el.mean_anomaly, unit));
    out["elements"] = std::move(elements);
    return out;
}

} // namespace

// ---------------------------------------------------------------------------
// Reading text
// ---------------------------------------------------------------------------

namespace {

// Read-only stdio file. Unlike std::filebuf, which reports a failed read as
// the end of the file, CheckedFileBuffer throws so that the reading istream
// sets badbit.
class CheckedFileBuffer : public std::streambuf {
public:
    explicit CheckedFileBuffer(const std::filesystem::path& path) {
#if defined(_WIN32)
        if (_wfopen_s(&file_, path.c_str(), L"rb") != 0) {
            file_ = nullptr;
        }
#else
        file_ = std::fopen(path.c_str(), "rb");
#endif
    }
    CheckedFileBuffer(const CheckedFileBuffer&) = delete;
    CheckedFileBuffer& operator=(const CheckedFileBuffer&) = delete;
    CheckedFileBuffer(CheckedFileBuffer&&) = delete;
    CheckedFileBuffer& operator=(CheckedFileBuffer&&) = delete;
    ~CheckedFileBuffer() override {
        if (file_ != nullptr) {
            // NOLINTNEXTLINE(cppcoreguidelines-owning-memory): FILE* is closed here by design.
            std::fclose(file_);
        }
    }

    [[nodiscard]] bool is_open() const noexcept { return file_ != nullptr; }

protected:
    int_type underflow() override {
        if (gptr() < egptr()) {
            return traits_type::to_int_type(*gptr());
        }
        const std::size_t count = std::fread(chunk_.data(), 1, chunk_.size(), file_);
        if (count == 0) {
            if (std::ferror(file_) != 0) {
                throw std::ios_base::failure("read error");
            }
            return traits_type::eof();
        }
        setg(chunk_.data(), chunk_.data(), chunk_.data() + count);
        return traits_type::to_int_type(*gptr());
    }

private:
    std::FILE* file_ = nullptr;
    std::array<char, 16384> chunk_{};
};

std::string read_stream_or_throw(std::istream& in, std::string_view name) {
    const auto read_error = [name] {
        fail(SceneErrorCode::io_error, {}, "cannot read '" + std::string(name) + "'");
    };
    std::string text;
    std::array<char, 16384> chunk{};
    for (;;) {
        // istream::read sets eofbit (with failbit) at a clean end of the
        // stream, failbit alone if it could not start, and badbit if the
        // stream buffer failed.
        in.read(chunk.data(), static_cast<std::streamsize>(chunk.size()));
        const auto count = static_cast<std::size_t>(in.gcount());
        if (in.bad()) {
            read_error();
        }
        if (count > limits::max_document_bytes - text.size()) {
            fail(SceneErrorCode::limit_exceeded, {},
                 "'" + std::string(name) + "' is larger than " +
                     std::to_string(limits::max_document_bytes) + " bytes");
        }
        text.append(chunk.data(), count);
        if (in.eof()) {
            return text;
        }
        if (in.fail()) {
            read_error();
        }
    }
}

std::string read_file_or_throw(const std::filesystem::path& path) {
    const std::string name = path.string();
    CheckedFileBuffer buffer(path);
    if (!buffer.is_open()) {
        fail(SceneErrorCode::io_error, {}, "cannot open '" + name + "'");
    }
    std::istream in(&buffer);
    return read_stream_or_throw(in, name);
}

} // namespace

std::optional<SceneError> validate(const Scenario& scenario) {
    try {
        validate_or_throw(scenario);
    } catch (SceneFailure& f) {
        return std::move(f.error);
    } catch (...) {
        return detail::exception_error(std::current_exception());
    }
    return std::nullopt;
}

core::Result<Scenario, SceneError> parse_scenario(std::string_view json) {
    return detail::guarded<Scenario>(
        [json] { return read_scenario(detail::parse_or_throw(json).value()); });
}

core::Result<Scene, SceneError> build_scene(const Scenario& scenario) {
    return detail::guarded<Scene>([&scenario] { return build_or_throw(scenario); });
}

core::Result<Scene, SceneError> load_scene(std::string_view json) {
    return detail::guarded<Scene>(
        [json] { return build_or_throw(read_scenario(detail::parse_or_throw(json).value())); });
}

core::Result<std::string, SceneError> read_text_file(const std::filesystem::path& path) {
    return detail::guarded<std::string>([&path] { return read_file_or_throw(path); });
}

core::Result<std::string, SceneError> read_text_stream(std::istream& in, std::string_view name) {
    return detail::guarded<std::string>([&in, name] { return read_stream_or_throw(in, name); });
}

core::Result<Scene, SceneError> load_scene_file(const std::filesystem::path& path) {
    return detail::guarded<Scene>([&path] {
        const std::string text = read_file_or_throw(path);
        return build_or_throw(read_scenario(detail::parse_or_throw(text).value()));
    });
}

std::string save_scenario(const Scenario& scenario) {
    OrderedJson root = OrderedJson::object();
    root["version"] = scenario_schema_version;
    root["name"] = scenario.name;
    root["units"] =
        OrderedJson{{"length", "AU"},
                    {"mass", "Msun"},
                    {"time", "day"},
                    {"angle", scenario.angle_unit == AngleUnit::degrees ? "deg" : "rad"}};
    OrderedJson metadata = OrderedJson::object();
    metadata["source"] = scenario.metadata.source;
    metadata["epoch"] = scenario.metadata.epoch;
    if (!scenario.metadata.description.empty()) {
        metadata["description"] = scenario.metadata.description;
    }
    root["metadata"] = std::move(metadata);
    root["time"] = detail::number_to_json(scenario.time.value());
    OrderedJson bodies = OrderedJson::array();
    for (const BodySpec& body : scenario.bodies) {
        bodies.push_back(body_to_json(body, scenario.angle_unit));
    }
    root["bodies"] = std::move(bodies);
    return root.dump(2) + "\n";
}

namespace {

Scenario to_scenario_or_throw(const Scene& scene) {
    Scenario scenario;
    scenario.name = scene.name;
    scenario.metadata = scene.metadata;
    scenario.angle_unit = AngleUnit::radians;
    scenario.time = scene.state.time();
    const core::BodyStore& store = scene.state.bodies();
    scenario.bodies.reserve(store.size());
    for (std::size_t i = 0; i < store.size(); ++i) {
        BodySpec body;
        body.name = store.name(i);
        body.kind = core::tag_of(store.kind(i));
        body.mass = store.mass(i);
        if (const auto* hole = std::get_if<core::BlackHole>(&store.kind(i))) {
            body.spin = hole->spin;
        }
        if (const auto* probe = std::get_if<core::Probe>(&store.kind(i))) {
            const std::uint32_t ref = probe->aero_ref.value;
            if (ref > scene.aero_tables.size()) {
                fail(SceneErrorCode::invalid_value, detail::child_path(body_path(i), "aero_ref"),
                     "aero_ref " + std::to_string(ref) + " has no entry in aero_tables");
            }
            if (ref != 0) {
                body.aero_ref = scene.aero_tables[ref - 1];
            }
        }
        body.orbit = CartesianSpec{store.position(i), store.velocity(i)};
        scenario.bodies.push_back(std::move(body));
    }
    return scenario;
}

} // namespace

core::Result<Scenario, SceneError> to_scenario(const Scene& scene) {
    return detail::guarded<Scenario>([&scene] { return to_scenario_or_throw(scene); });
}

core::Result<std::string, SceneError> save_scene(const Scene& scene) {
    return detail::guarded<std::string>(
        [&scene] { return save_scenario(to_scenario_or_throw(scene)); });
}

} // namespace orrery::scene
