#include "orrery/scene/aero_experiment.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <span>
#include <utility>
#include <variant>

#include "orrery/geom/naca.h"

namespace orrery::scene {

namespace {

constexpr double kDegree = std::numbers::pi / 180.0;
constexpr double kMaxStepsPerFrame = 1.0e6;

struct ShapeName {
    AeroShape shape;
    std::string_view key;
    std::string_view display;
};

constexpr std::array<ShapeName, kAeroShapeCount> kShapes{{
    {AeroShape::sphere, "sphere", "Sphere"},
    {AeroShape::cube, "cube", "Cube"},
    {AeroShape::cylinder, "cylinder", "Cylinder (cross-flow)"},
    {AeroShape::cone, "cone", "Cone"},
    {AeroShape::ellipsoid, "ellipsoid", "Ellipsoid"},
    {AeroShape::flat_plate, "flat_plate", "Flat plate"},
    {AeroShape::wedge, "wedge", "Wedge"},
    {AeroShape::capsule, "capsule", "Entry capsule"},
    {AeroShape::naca_wing, "naca_wing", "NACA wing"},
    {AeroShape::naca_section, "naca_section", "NACA section (2D)"},
}};

struct DimensionKey {
    std::string_view key;
    double ShapeDimensions::* member;
};

constexpr std::array<DimensionKey, 9> kDimensionKeys{{
    {"diameter_m", &ShapeDimensions::diameter_m},
    {"length_m", &ShapeDimensions::length_m},
    {"width_m", &ShapeDimensions::width_m},
    {"height_m", &ShapeDimensions::height_m},
    {"side_m", &ShapeDimensions::side_m},
    {"chord_m", &ShapeDimensions::chord_m},
    {"span_m", &ShapeDimensions::span_m},
    {"thickness_m", &ShapeDimensions::thickness_m},
    {"half_angle_deg", &ShapeDimensions::half_angle_deg},
}};

double ShapeDimensions::* dimension_member(std::string_view key) noexcept {
    for (const DimensionKey& entry : kDimensionKeys) {
        if (entry.key == key) {
            return entry.member;
        }
    }
    return nullptr;
}

// The value of a dimension key (shape_dimensions only names known keys).
double dimension_value(const ShapeDimensions& d, std::string_view key) noexcept {
    double ShapeDimensions::* member = dimension_member(key);
    return member != nullptr ? d.*member : 0.0;
}

struct AreaName {
    geom::ReferenceAreaKind kind;
    std::string_view name;
};

constexpr std::array<AreaName, 3> kAreas{{
    {geom::ReferenceAreaKind::frontal, "frontal"},
    {geom::ReferenceAreaKind::planform, "planform"},
    {geom::ReferenceAreaKind::wetted, "wetted"},
}};

constexpr std::array<aero::SolverKind, 3> kSolvers{
    aero::SolverKind::panel, aero::SolverKind::newtonian, aero::SolverKind::lattice_boltzmann};

std::string join(std::span<const std::string_view> names) {
    std::string list;
    for (const std::string_view name : names) {
        list += (list.empty() ? "" : ", ") + std::string(name);
    }
    return list.empty() ? std::string("none") : list;
}

// Typed access to one parameter map; every error names "<where>.<key>".
class Reader {
public:
    Reader(const Parameters& parameters, std::string where)
        : parameters_(&parameters), where_(std::move(where)) {}

    [[nodiscard]] std::string path(std::string_view key) const {
        return where_ + "." + std::string(key);
    }

    [[nodiscard]] std::optional<std::string>
    unknown_key(std::span<const std::string_view> allowed) const {
        for (const auto& entry : *parameters_) {
            if (std::find(allowed.begin(), allowed.end(), entry.first) == allowed.end()) {
                return path(entry.first) + ": unknown key (allowed: " + join(allowed) + ")";
            }
        }
        return std::nullopt;
    }

    [[nodiscard]] bool has(std::string_view key) const {
        return parameters_->find(key) != parameters_->end();
    }

    // Absent: an empty optional. Wrong type: an error.
    [[nodiscard]] core::Result<std::optional<double>, std::string>
    number(std::string_view key) const {
        const auto it = parameters_->find(key);
        if (it == parameters_->end()) {
            return std::optional<double>{};
        }
        if (const auto* value = std::get_if<double>(&it->second)) {
            return std::optional<double>(*value);
        }
        return core::failure(path(key) + ": expected a number");
    }

    [[nodiscard]] core::Result<std::optional<std::string>, std::string>
    text(std::string_view key) const {
        const auto it = parameters_->find(key);
        if (it == parameters_->end()) {
            return std::optional<std::string>{};
        }
        if (const auto* value = std::get_if<std::string>(&it->second)) {
            return std::optional<std::string>(*value);
        }
        return core::failure(path(key) + ": expected a string");
    }

    // A number accepted by `valid`, or `fallback` when absent.
    template <typename Valid>
    [[nodiscard]] core::Result<double, std::string>
    checked(std::string_view key, double fallback, Valid valid, std::string_view rule) const {
        auto value = number(key);
        if (!value) {
            return core::failure(std::string(value.error()));
        }
        const std::optional<double> given = value.value();
        if (!given) {
            return fallback;
        }
        if (!valid(*given)) {
            return core::failure(path(key) + ": " + std::string(rule));
        }
        return *given;
    }

    [[nodiscard]] core::Result<double, std::string> positive(std::string_view key,
                                                             double fallback) const {
        return checked(
            key, fallback, [](double v) { return v > 0.0 && std::isfinite(v); },
            "must be positive and finite");
    }

    [[nodiscard]] core::Result<std::string, std::string> text_or(std::string_view key,
                                                                 std::string fallback) const {
        auto value = text(key);
        if (!value) {
            return core::failure(std::string(value.error()));
        }
        return value.value().value_or(std::move(fallback));
    }

private:
    const Parameters* parameters_;
    std::string where_;
};

std::string_view up_axis_name(geom::UpAxis axis) noexcept {
    return axis == geom::UpAxis::y_up ? "y" : "z";
}

core::Result<GeometryRef, std::string> read_geometry(const Parameters& inputs) {
    const Reader in(inputs, "inputs");
    GeometryRef ref;
    auto source = in.text_or("geometry", "primitive");
    if (!source) {
        return core::failure(std::string(source.error()));
    }
    if (source.value() == "primitive") {
        ref.source = GeometrySource::primitive;
    } else if (source.value() == "file") {
        ref.source = GeometrySource::file;
    } else if (source.value() == "url") {
        ref.source = GeometrySource::url;
    } else {
        return core::failure(in.path("geometry") + ": unknown geometry source '" + source.value() +
                             "' (allowed: primitive, file, url)");
    }

    std::vector<std::string_view> allowed{"name", "geometry"};
    if (ref.source == GeometrySource::primitive) {
        auto shape_name = in.text_or("shape", "sphere");
        if (!shape_name) {
            return core::failure(std::string(shape_name.error()));
        }
        const auto shape = aero_shape_from_string(shape_name.value());
        if (!shape) {
            std::vector<std::string_view> names;
            names.reserve(kShapes.size());
            for (const ShapeName& entry : kShapes) {
                names.push_back(entry.key);
            }
            return core::failure(in.path("shape") + ": unknown shape '" + shape_name.value() +
                                 "' (allowed: " + join(names) + ")");
        }
        ref.shape = *shape;
        allowed.emplace_back("shape");
        for (const std::string_view key : shape_dimensions(ref.shape)) {
            allowed.push_back(key);
            double ShapeDimensions::* member = dimension_member(key);
            if (member == nullptr) {
                continue;
            }
            auto value = in.positive(key, ref.dimensions.*member);
            if (!value) {
                return core::failure(std::string(value.error()));
            }
            ref.dimensions.*member = value.value();
        }
        if (ref.dimensions.half_angle_deg >= 90.0) {
            return core::failure(in.path("half_angle_deg") + ": must be below 90 degrees");
        }
        if (ref.shape == AeroShape::naca_wing || ref.shape == AeroShape::naca_section) {
            allowed.emplace_back("naca");
            auto naca = in.text_or("naca", ref.naca);
            if (!naca) {
                return core::failure(std::string(naca.error()));
            }
            if (!geom::Naca4::from_designation(naca.value())) {
                return core::failure(in.path("naca") + ": '" + naca.value() +
                                     "' is not a NACA 4-digit designation");
            }
            ref.naca = naca.value();
        }
        if (ref.shape == AeroShape::capsule) {
            allowed.emplace_back("capsule");
            auto capsule = in.text_or("capsule", ref.capsule);
            if (!capsule) {
                return core::failure(std::string(capsule.error()));
            }
            if (capsule.value() != "apollo" && capsule.value() != "orion") {
                return core::failure(in.path("capsule") + ": unknown capsule '" + capsule.value() +
                                     "' (allowed: apollo, orion)");
            }
            ref.capsule = capsule.value();
        }
    } else {
        for (const std::string_view key : {"path", "unit_scale", "up_axis", "license"}) {
            allowed.push_back(key);
        }
        if (ref.source == GeometrySource::url) {
            allowed.emplace_back("url");
            allowed.emplace_back("sha256");
        }
        auto path = in.text_or("path", "");
        auto url = in.text_or("url", "");
        auto sha = in.text_or("sha256", "");
        auto license = in.text_or("license", "");
        auto up = in.text_or("up_axis", "z");
        auto scale = in.positive("unit_scale", 1.0);
        for (const auto* failed : {&path, &url, &sha, &license, &up}) {
            if (!*failed) {
                return core::failure(std::string(failed->error()));
            }
        }
        if (!scale) {
            return core::failure(std::string(scale.error()));
        }
        ref.path = path.value();
        ref.url = url.value();
        ref.sha256 = sha.value();
        ref.license_note = license.value();
        ref.unit_scale = scale.value();
        if (up.value() == "z") {
            ref.up_axis = geom::UpAxis::z_up;
        } else if (up.value() == "y") {
            ref.up_axis = geom::UpAxis::y_up;
        } else {
            return core::failure(in.path("up_axis") + ": unknown up axis '" + up.value() +
                                 "' (allowed: z, y)");
        }
        if (ref.path.empty() && ref.url.empty()) {
            return core::failure(in.path("path") + ": a " + source.value() +
                                 " geometry needs a model path");
        }
        if (ref.source == GeometrySource::url && ref.url.empty()) {
            return core::failure(in.path("url") + ": a url geometry needs its source URL");
        }
    }
    if (auto problem = in.unknown_key(allowed)) {
        return core::failure(std::move(*problem));
    }
    return ref;
}

core::Result<AeroConditions, std::string> read_conditions(const Parameters& parameters) {
    const Reader in(parameters, "conditions");
    constexpr std::array<std::string_view, 8> kKeys{
        "speed_m_s",           "density_kg_m3", "viscosity_pa_s", "temperature_k",
        "angle_of_attack_deg", "sideslip_deg",  "gamma",          "gas_constant_j_kg_k"};
    if (auto problem = in.unknown_key(kKeys)) {
        return core::failure(std::move(*problem));
    }
    AeroConditions c;
    const auto finite = [](double v) { return std::isfinite(v); };
    auto speed = in.positive("speed_m_s", c.speed_m_s);
    auto density = in.positive("density_kg_m3", c.density_kg_m3);
    auto viscosity = in.positive("viscosity_pa_s", c.viscosity_pa_s);
    auto temperature = in.positive("temperature_k", c.temperature_k);
    auto alpha = in.checked("angle_of_attack_deg", c.angle_of_attack_deg, finite, "must be finite");
    auto beta = in.checked("sideslip_deg", c.sideslip_deg, finite, "must be finite");
    auto gamma = in.checked(
        "gamma", c.gamma, [](double v) { return v > 1.0 && std::isfinite(v); },
        "must be above 1 and finite");
    auto gas = in.positive("gas_constant_j_kg_k", c.gas_constant_j_kg_k);
    for (const auto* value :
         {&speed, &density, &viscosity, &temperature, &alpha, &beta, &gamma, &gas}) {
        if (!*value) {
            return core::failure(std::string(value->error()));
        }
    }
    c.speed_m_s = speed.value();
    c.density_kg_m3 = density.value();
    c.viscosity_pa_s = viscosity.value();
    c.temperature_k = temperature.value();
    c.angle_of_attack_deg = alpha.value();
    c.sideslip_deg = beta.value();
    c.gamma = gamma.value();
    c.gas_constant_j_kg_k = gas.value();
    return c;
}

core::Result<AeroSolverSettings, std::string> read_solver(const SolverInfo& info) {
    AeroSolverSettings s;
    if (info.name != "auto") {
        const auto kind = solver_from_string(info.name);
        if (!kind) {
            return core::failure("solver.name: unknown solver '" + info.name +
                                 "' (allowed: auto, panel, newtonian, lbm)");
        }
        s.solver = *kind;
    }
    if (!info.version.empty() && info.version != kAeroSolverVersion) {
        return core::failure("solver.version: '" + info.version +
                             "' is not supported (supported: " + std::string(kAeroSolverVersion) +
                             ")");
    }
    const Reader in(info.settings, "solver.settings");
    constexpr std::array<std::string_view, 5> kKeys{"reference_area", "reference_length_m",
                                                    "lbm_cells_per_length", "lbm_duration",
                                                    "steps_per_frame"};
    if (auto problem = in.unknown_key(kKeys)) {
        return core::failure(std::move(*problem));
    }
    auto area = in.text_or("reference_area", "frontal");
    if (!area) {
        return core::failure(std::string(area.error()));
    }
    const auto kind = reference_area_from_string(area.value());
    if (!kind) {
        return core::failure(in.path("reference_area") + ": unknown reference area '" +
                             area.value() + "' (allowed: frontal, planform, wetted)");
    }
    s.reference_area = *kind;
    if (in.has("reference_length_m")) {
        auto length = in.positive("reference_length_m", 1.0);
        if (!length) {
            return core::failure(std::string(length.error()));
        }
        s.reference_length_m = length.value();
    }
    auto cells = in.positive("lbm_cells_per_length", 1.0);
    auto duration = in.positive("lbm_duration", 1.0);
    auto steps = in.checked(
        "steps_per_frame", static_cast<double>(s.steps_per_frame),
        [](double v) { return v >= 1.0 && v <= kMaxStepsPerFrame && std::floor(v) == v; },
        "must be an integer in [1, 1000000]");
    for (const auto* value : {&cells, &duration, &steps}) {
        if (!*value) {
            return core::failure(std::string(value->error()));
        }
    }
    s.lbm_cells_per_length = in.has("lbm_cells_per_length") ? cells.value() : 0.0;
    s.lbm_duration = in.has("lbm_duration") ? duration.value() : 0.0;
    s.steps_per_frame = static_cast<std::int64_t>(steps.value());
    return s;
}

std::vector<std::string> split_codes(const std::string& text) {
    std::vector<std::string> codes;
    std::size_t start = 0;
    while (start < text.size()) {
        std::size_t end = text.find(';', start);
        if (end == std::string::npos) {
            end = text.size();
        }
        std::string code = text.substr(start, end - start);
        const auto first = code.find_first_not_of(' ');
        const auto last = code.find_last_not_of(' ');
        if (first != std::string::npos) {
            codes.push_back(code.substr(first, last - first + 1));
        }
        start = end + 1;
    }
    return codes;
}

core::Result<std::optional<AeroResultRecord>, std::string>
read_result(const Experiment& experiment) {
    if (experiment.result.empty() && experiment.series.empty()) {
        return std::optional<AeroResultRecord>{};
    }
    const Reader in(experiment.result, "result");
    constexpr std::array<std::string_view, 16> kKeys{"solver",
                                                     "cd",
                                                     "cl",
                                                     "cy",
                                                     "cm",
                                                     "cd_pressure",
                                                     "cd_induced",
                                                     "cd_friction",
                                                     "reynolds",
                                                     "mach",
                                                     "reference_area_m2",
                                                     "reference_length_m",
                                                     "iterations",
                                                     "converged",
                                                     "warnings",
                                                     "strouhal"};
    if (auto problem = in.unknown_key(kKeys)) {
        return core::failure(std::move(*problem));
    }
    AeroResultRecord r;
    auto solver = in.text("solver");
    if (!solver) {
        return core::failure(std::string(solver.error()));
    }
    if (!solver.value()) {
        return core::failure(in.path("solver") + ": a result names the solver that produced it");
    }
    const auto kind = solver_from_string(*solver.value());
    if (!kind) {
        return core::failure(in.path("solver") + ": unknown solver '" + *solver.value() +
                             "' (allowed: panel, newtonian, lbm)");
    }
    r.solver = *kind;
    const auto any = [](double) { return true; };
    struct Field {
        std::string_view key;
        double AeroResultRecord::* member;
    };
    constexpr std::array<Field, 11> kFields{{
        {"cd", &AeroResultRecord::cd},
        {"cl", &AeroResultRecord::cl},
        {"cy", &AeroResultRecord::cy},
        {"cm", &AeroResultRecord::cm},
        {"cd_pressure", &AeroResultRecord::cd_pressure},
        {"cd_induced", &AeroResultRecord::cd_induced},
        {"cd_friction", &AeroResultRecord::cd_friction},
        {"reynolds", &AeroResultRecord::reynolds},
        {"mach", &AeroResultRecord::mach},
        {"reference_area_m2", &AeroResultRecord::reference_area_m2},
        {"reference_length_m", &AeroResultRecord::reference_length_m},
    }};
    for (const Field& field : kFields) {
        auto value = in.checked(field.key, 0.0, any, "");
        if (!value) {
            return core::failure(std::string(value.error()));
        }
        r.*field.member = value.value();
    }
    auto iterations = in.checked(
        "iterations", 0.0, [](double v) { return v >= 0.0 && v < 9.0e15 && std::floor(v) == v; },
        "must be a non-negative integer");
    auto converged = in.checked(
        "converged", 0.0, [](double v) { return v == 0.0 || v == 1.0; }, "must be 0 or 1");
    auto warnings = in.text_or("warnings", "");
    if (!iterations) {
        return core::failure(std::string(iterations.error()));
    }
    if (!converged) {
        return core::failure(std::string(converged.error()));
    }
    if (!warnings) {
        return core::failure(std::string(warnings.error()));
    }
    r.iterations = static_cast<std::int64_t>(iterations.value());
    r.converged = converged.value() == 1.0;
    r.warnings = split_codes(warnings.value());
    if (in.has("strouhal")) {
        auto strouhal = in.positive("strouhal", 1.0);
        if (!strouhal) {
            return core::failure(std::string(strouhal.error()));
        }
        r.strouhal = strouhal.value();
    }

    constexpr std::array<std::string_view, 4> kSeries{"residual", "history_step", "history_cd",
                                                      "history_cl"};
    for (const auto& entry : experiment.series) {
        if (std::find(kSeries.begin(), kSeries.end(), entry.first) == kSeries.end()) {
            return core::failure("series." + entry.first +
                                 ": unknown key (allowed: " + join(kSeries) + ")");
        }
    }
    const auto series = [&](std::string_view key) {
        const auto it = experiment.series.find(key);
        return it == experiment.series.end() ? std::vector<double>{} : it->second;
    };
    r.residuals = series("residual");
    r.history_step = series("history_step");
    r.history_cd = series("history_cd");
    r.history_cl = series("history_cl");
    if (r.history_cd.size() != r.history_step.size() ||
        r.history_cl.size() != r.history_step.size()) {
        return core::failure(
            std::string("series: history_step, history_cd and history_cl must have "
                        "the same length"));
    }
    return std::optional<AeroResultRecord>(std::move(r));
}

} // namespace

std::string_view to_string(AeroShape shape) noexcept {
    for (const ShapeName& entry : kShapes) {
        if (entry.shape == shape) {
            return entry.key;
        }
    }
    return "unknown";
}

std::optional<AeroShape> aero_shape_from_string(std::string_view name) noexcept {
    for (const ShapeName& entry : kShapes) {
        if (entry.key == name) {
            return entry.shape;
        }
    }
    return std::nullopt;
}

std::string_view display_name(AeroShape shape) noexcept {
    for (const ShapeName& entry : kShapes) {
        if (entry.shape == shape) {
            return entry.display;
        }
    }
    return "Unknown";
}

bool is_two_dimensional(AeroShape shape) noexcept {
    return shape == AeroShape::naca_section;
}

std::vector<std::string_view> shape_dimensions(AeroShape shape) {
    switch (shape) {
        case AeroShape::sphere:
        case AeroShape::capsule:
            return {"diameter_m"};
        case AeroShape::cube:
            return {"side_m"};
        case AeroShape::cylinder:
        case AeroShape::cone:
            return {"diameter_m", "length_m"};
        case AeroShape::ellipsoid:
            return {"length_m", "width_m", "height_m"};
        case AeroShape::flat_plate:
            return {"chord_m", "span_m", "thickness_m"};
        case AeroShape::wedge:
            return {"length_m", "half_angle_deg", "span_m"};
        case AeroShape::naca_wing:
            return {"chord_m", "span_m"};
        case AeroShape::naca_section:
            return {"chord_m"};
    }
    return {};
}

aero::FlowConditions AeroConditions::flow() const noexcept {
    aero::FlowConditions c;
    c.speed = speed_m_s;
    c.density = density_kg_m3;
    c.viscosity = viscosity_pa_s;
    c.temperature = temperature_k;
    c.angle_of_attack = angle_of_attack_deg * kDegree;
    c.sideslip = sideslip_deg * kDegree;
    c.gamma = gamma;
    c.gas_constant = gas_constant_j_kg_k;
    return c;
}

std::string_view to_string(geom::ReferenceAreaKind kind) noexcept {
    for (const AreaName& entry : kAreas) {
        if (entry.kind == kind) {
            return entry.name;
        }
    }
    return "frontal";
}

std::optional<geom::ReferenceAreaKind> reference_area_from_string(std::string_view name) noexcept {
    for (const AreaName& entry : kAreas) {
        if (entry.name == name) {
            return entry.kind;
        }
    }
    return std::nullopt;
}

std::optional<aero::SolverKind> solver_from_string(std::string_view name) noexcept {
    for (const aero::SolverKind kind : kSolvers) {
        if (aero::solver_name(kind) == name) {
            return kind;
        }
    }
    return std::nullopt;
}

Experiment to_experiment(const AeroExperiment& experiment) {
    Experiment out;
    out.domain = Domain::aero;
    const GeometryRef& g = experiment.geometry;
    if (!experiment.name.empty()) {
        out.inputs.emplace("name", experiment.name);
    }
    switch (g.source) {
        case GeometrySource::primitive:
            out.inputs.emplace("geometry", std::string("primitive"));
            out.inputs.emplace("shape", std::string(to_string(g.shape)));
            for (const std::string_view key : shape_dimensions(g.shape)) {
                out.inputs.emplace(std::string(key), dimension_value(g.dimensions, key));
            }
            if (g.shape == AeroShape::naca_wing || g.shape == AeroShape::naca_section) {
                out.inputs.emplace("naca", g.naca);
            }
            if (g.shape == AeroShape::capsule) {
                out.inputs.emplace("capsule", g.capsule);
            }
            break;
        case GeometrySource::file:
        case GeometrySource::url:
            out.inputs.emplace("geometry",
                               std::string(g.source == GeometrySource::file ? "file" : "url"));
            out.inputs.emplace("path", g.path);
            out.inputs.emplace("unit_scale", g.unit_scale);
            out.inputs.emplace("up_axis", std::string(up_axis_name(g.up_axis)));
            if (!g.license_note.empty()) {
                out.inputs.emplace("license", g.license_note);
            }
            if (g.source == GeometrySource::url) {
                out.inputs.emplace("url", g.url);
                if (!g.sha256.empty()) {
                    out.inputs.emplace("sha256", g.sha256);
                }
            }
            break;
    }

    const AeroConditions& c = experiment.conditions;
    out.conditions = {{"speed_m_s", c.speed_m_s},
                      {"density_kg_m3", c.density_kg_m3},
                      {"viscosity_pa_s", c.viscosity_pa_s},
                      {"temperature_k", c.temperature_k},
                      {"angle_of_attack_deg", c.angle_of_attack_deg},
                      {"sideslip_deg", c.sideslip_deg},
                      {"gamma", c.gamma},
                      {"gas_constant_j_kg_k", c.gas_constant_j_kg_k}};

    const AeroSolverSettings& s = experiment.solver;
    out.solver.name = s.solver ? std::string(aero::solver_name(*s.solver)) : std::string("auto");
    out.solver.version = std::string(kAeroSolverVersion);
    out.solver.settings.emplace("reference_area", std::string(to_string(s.reference_area)));
    if (s.reference_length_m) {
        out.solver.settings.emplace("reference_length_m", *s.reference_length_m);
    }
    if (s.lbm_cells_per_length > 0.0) {
        out.solver.settings.emplace("lbm_cells_per_length", s.lbm_cells_per_length);
    }
    if (s.lbm_duration > 0.0) {
        out.solver.settings.emplace("lbm_duration", s.lbm_duration);
    }
    out.solver.settings.emplace("steps_per_frame", static_cast<double>(s.steps_per_frame));

    if (experiment.result) {
        const AeroResultRecord& r = *experiment.result;
        out.result = {{"solver", std::string(aero::solver_name(r.solver))},
                      {"cd", r.cd},
                      {"cl", r.cl},
                      {"cy", r.cy},
                      {"cm", r.cm},
                      {"cd_pressure", r.cd_pressure},
                      {"cd_induced", r.cd_induced},
                      {"cd_friction", r.cd_friction},
                      {"reynolds", r.reynolds},
                      {"mach", r.mach},
                      {"reference_area_m2", r.reference_area_m2},
                      {"reference_length_m", r.reference_length_m},
                      {"iterations", static_cast<double>(r.iterations)},
                      {"converged", r.converged ? 1.0 : 0.0}};
        if (r.strouhal) {
            out.result.emplace("strouhal", *r.strouhal);
        }
        if (!r.warnings.empty()) {
            std::string codes;
            for (const std::string& code : r.warnings) {
                codes += (codes.empty() ? "" : "; ") + code;
            }
            out.result.emplace("warnings", std::move(codes));
        }
        const auto add_series = [&out](const char* key, const std::vector<double>& values) {
            if (!values.empty()) {
                out.series.emplace(key, values);
            }
        };
        add_series("residual", r.residuals);
        add_series("history_step", r.history_step);
        add_series("history_cd", r.history_cd);
        add_series("history_cl", r.history_cl);
    }
    return out;
}

core::Result<AeroExperiment, std::string> aero_from_experiment(const Experiment& experiment) {
    if (experiment.domain != Domain::aero) {
        return core::failure("domain: expected 'aero', found '" +
                             std::string(to_string(experiment.domain)) + "'");
    }
    AeroExperiment out;
    const Reader inputs(experiment.inputs, "inputs");
    auto name = inputs.text_or("name", "");
    if (!name) {
        return core::failure(std::string(name.error()));
    }
    out.name = name.value();
    auto geometry = read_geometry(experiment.inputs);
    if (!geometry) {
        return core::failure(std::string(geometry.error()));
    }
    out.geometry = std::move(geometry).value();
    auto conditions = read_conditions(experiment.conditions);
    if (!conditions) {
        return core::failure(std::string(conditions.error()));
    }
    out.conditions = conditions.value();
    auto solver = read_solver(experiment.solver);
    if (!solver) {
        return core::failure(std::string(solver.error()));
    }
    out.solver = solver.value();
    auto result = read_result(experiment);
    if (!result) {
        return core::failure(std::string(result.error()));
    }
    out.result = std::move(result).value();
    return out;
}

core::Result<AeroExperiment, std::string> parse_aero_experiment(std::string_view json) {
    const auto parsed = parse_experiment(json);
    if (!parsed) {
        return core::failure(to_string(parsed.error()));
    }
    return aero_from_experiment(parsed.value());
}

std::string save_aero_experiment(const AeroExperiment& experiment) {
    return save_experiment(to_experiment(experiment));
}

} // namespace orrery::scene
