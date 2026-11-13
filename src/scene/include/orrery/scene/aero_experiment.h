#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "orrery/aero/flow_conditions.h"
#include "orrery/aero/regime.h"
#include "orrery/core/result.h"
#include "orrery/geom/mesh_import.h"
#include "orrery/geom/reference_quantities.h"
#include "orrery/scene/experiment.h"

// Typed view of an experiment record of the "aero" domain (docs/experiments.md):
// a geometry reference, the free-stream conditions, the solver choice and,
// once a run finished, its result. The record is stored in the generic
// Experiment schema (version 1) with these keys; anything else in the maps is
// rejected by aero_from_experiment.
//
//   inputs      name (text), geometry ("primitive" | "file" | "url");
//               primitive: shape and its dimensions (below), naca, capsule;
//               file:      path, unit_scale, up_axis ("z" | "y"), license;
//               url:       url, path (cached file), sha256, unit_scale,
//                          up_axis, license
//   conditions  speed_m_s, density_kg_m3, viscosity_pa_s, temperature_k,
//               angle_of_attack_deg, sideslip_deg, gamma, gas_constant_j_kg_k
//   solver      name: "auto" (the regime recommendation), "panel",
//               "newtonian" or "lbm"; version "1"; settings: reference_area
//               ("frontal" | "planform" | "wetted"), reference_length_m,
//               lbm_cells_per_length, lbm_duration, steps_per_frame
//   result      solver, cd, cl, cy, cm, cd_pressure, cd_induced, cd_friction,
//               reynolds, mach, reference_area_m2, reference_length_m,
//               iterations, converged (0 or 1), strouhal, warnings (codes,
//               "; "-joined)
//   series      residual, history_step, history_cd, history_cl
namespace orrery::scene {

enum class GeometrySource : std::uint8_t { primitive, file, url };

// Parametric bodies of the Geometry panel (orrery/geom/primitives.h). Solids
// of revolution face the free stream: the cone and capsule noses point
// upstream (-x), the cylinder lies across the flow (axis along y).
enum class AeroShape : std::uint8_t {
    sphere,
    cube,
    cylinder,
    cone,
    ellipsoid,
    flat_plate,
    wedge,
    capsule,
    naca_wing,
    naca_section, // 2D: a section in the x-z plane, per unit span
};

inline constexpr std::size_t kAeroShapeCount = 10;

[[nodiscard]] std::string_view to_string(AeroShape shape) noexcept;
[[nodiscard]] std::optional<AeroShape> aero_shape_from_string(std::string_view name) noexcept;
// Human-readable name for the user interface ("NACA section (2D)").
[[nodiscard]] std::string_view display_name(AeroShape shape) noexcept;
[[nodiscard]] bool is_two_dimensional(AeroShape shape) noexcept;

// Dimensions of a primitive, metres and degrees. Each shape reads a subset
// (shape_dimensions); the others keep their defaults and are not stored.
struct ShapeDimensions {
    double diameter_m = 1.0; // sphere, cylinder, cone, capsule (maximum diameter)
    double length_m = 2.0;   // cylinder (span), cone, ellipsoid (x), wedge
    double width_m = 1.0;    // ellipsoid (y)
    double height_m = 1.0;   // ellipsoid (z)
    double side_m = 1.0;     // cube
    double chord_m = 1.0;    // flat plate, NACA wing and section
    double span_m = 4.0;     // flat plate, wedge, NACA wing
    double thickness_m = 0.02;
    double half_angle_deg = 10.0; // wedge

    friend bool operator==(const ShapeDimensions&, const ShapeDimensions&) = default;
};

// Dimension keys a shape stores, e.g. {"diameter_m"} for a sphere.
[[nodiscard]] std::vector<std::string_view> shape_dimensions(AeroShape shape);

struct GeometryRef {
    GeometrySource source = GeometrySource::primitive;
    AeroShape shape = AeroShape::sphere;
    ShapeDimensions dimensions;
    std::string naca = "0012";      // NACA wing and section: 4-digit designation
    std::string capsule = "apollo"; // capsule: "apollo" or "orion" proportions
    // file: the model file (relative paths are resolved against the
    // experiment file's directory); url: the cached download.
    std::string path;
    std::string url;
    std::string sha256;
    // Licence or source note entered by the user (never checked).
    std::string license_note;
    // Confirmed scale from file units to metres and the file's up axis.
    double unit_scale = 1.0;
    geom::UpAxis up_axis = geom::UpAxis::z_up;

    // True for user models (file or URL): they carry no validation case and
    // the user interface marks them "unvalidated geometry".
    [[nodiscard]] bool user_model() const noexcept { return source != GeometrySource::primitive; }

    friend bool operator==(const GeometryRef&, const GeometryRef&) = default;
};

// Free-stream conditions as the user enters and the file stores them
// (angles in degrees, so that a record round-trips exactly).
struct AeroConditions {
    double speed_m_s = 30.0;
    double density_kg_m3 = 1.225;
    double viscosity_pa_s = 1.789e-5;
    double temperature_k = 288.15;
    double angle_of_attack_deg = 0.0;
    double sideslip_deg = 0.0;
    double gamma = 1.4;
    double gas_constant_j_kg_k = 287.05;

    // The solver's view (radians).
    [[nodiscard]] aero::FlowConditions flow() const noexcept;

    friend bool operator==(const AeroConditions&, const AeroConditions&) = default;
};

struct AeroSolverSettings {
    // Solver to run; nullopt runs the regime recommendation (aero::select_solver).
    std::optional<aero::SolverKind> solver;
    geom::ReferenceAreaKind reference_area = geom::ReferenceAreaKind::frontal;
    // Overrides the geometry's reference length (its extent along the flow).
    std::optional<double> reference_length_m;
    // Lattice Boltzmann: cells per reference length and simulated convective
    // times (0: the solver defaults).
    double lbm_cells_per_length = 0.0;
    double lbm_duration = 0.0;
    // Lattice steps advanced per frame in the interactive window.
    std::int64_t steps_per_frame = 100;

    friend bool operator==(const AeroSolverSettings&, const AeroSolverSettings&) = default;
};

struct AeroResultRecord {
    aero::SolverKind solver = aero::SolverKind::panel;
    double cd = 0.0;
    double cl = 0.0;
    double cy = 0.0;
    double cm = 0.0;
    double cd_pressure = 0.0;
    double cd_induced = 0.0;
    double cd_friction = 0.0;
    double reynolds = 0.0;
    double mach = 0.0;
    double reference_area_m2 = 0.0;
    double reference_length_m = 0.0;
    std::int64_t iterations = 0;
    bool converged = false;
    // Strouhal number of the lift oscillation (2D lattice Boltzmann), when found.
    std::optional<double> strouhal;
    std::vector<std::string> warnings; // ValidityWarning codes
    std::vector<double> residuals;
    // Force history of a time-marching solver: lattice step, cd, cl.
    std::vector<double> history_step;
    std::vector<double> history_cd;
    std::vector<double> history_cl;

    friend bool operator==(const AeroResultRecord&, const AeroResultRecord&) = default;
};

struct AeroExperiment {
    std::string name;
    GeometryRef geometry;
    AeroConditions conditions;
    AeroSolverSettings solver;
    std::optional<AeroResultRecord> result;

    friend bool operator==(const AeroExperiment&, const AeroExperiment&) = default;
};

inline constexpr std::string_view kAeroSolverVersion = "1";

[[nodiscard]] std::string_view to_string(geom::ReferenceAreaKind kind) noexcept;
[[nodiscard]] std::optional<geom::ReferenceAreaKind>
reference_area_from_string(std::string_view name) noexcept;
// "panel", "newtonian", "lbm" (aero::solver_name) back to the kind.
[[nodiscard]] std::optional<aero::SolverKind> solver_from_string(std::string_view name) noexcept;

// The generic record of an aero experiment (domain aero, schema version 1).
[[nodiscard]] Experiment to_experiment(const AeroExperiment& experiment);

// Reads an aero experiment back; the error names the offending key
// ("conditions.speed_m_s: must be positive and finite"). Conditions outside
// a solver's validity envelope are accepted (they produce warnings when the
// solver runs); only values no solver can use are rejected.
[[nodiscard]] core::Result<AeroExperiment, std::string>
aero_from_experiment(const Experiment& experiment);

// Parses and saves aero experiments as JSON text (parse_experiment /
// save_experiment plus the typed mapping).
[[nodiscard]] core::Result<AeroExperiment, std::string>
parse_aero_experiment(std::string_view json);
[[nodiscard]] std::string save_aero_experiment(const AeroExperiment& experiment);

} // namespace orrery::scene
