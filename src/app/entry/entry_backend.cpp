#include "orrery/sim/entry_backend.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

#include "orrery/atmo/aero_table.h"
#include "orrery/atmo/atmosphere.h"
#include "orrery/scene/aero_experiment.h"
#include "orrery/sim/aero_flow.h"
#include "orrery/sim/aero_geometry.h"

namespace orrery::sim {

namespace {

constexpr double kDegree = std::numbers::pi / 180.0;
// Specific gas constant and ratio of specific heats of Mars' CO2 (the
// exponential model's gas on Mars; Earth uses the library's air default).
constexpr double kMarsGasConstant = 192.1;
constexpr double kMarsGamma = 1.29;

core::Result<std::shared_ptr<const atmo::IAtmosphere>, std::string>
exponential_atmosphere(const EntrySetup& s) {
    atmo::ExponentialAtmosphereParameters p;
    p.surface_density = core::si::Density{s.exponential_density_kg_m3};
    p.scale_height = core::si::Length{s.exponential_scale_height_m};
    p.temperature_k = s.exponential_temperature_k;
    if (s.planet == EntryPlanet::mars) {
        p.gas.gamma = kMarsGamma;
        p.gas.gas_constant_j_kg_k = kMarsGasConstant;
    }
    auto made = atmo::ExponentialAtmosphere::create(p);
    if (!made) {
        return core::failure("exponential atmosphere: " + made.error().message);
    }
    std::shared_ptr<const atmo::IAtmosphere> shared =
        std::make_shared<const atmo::ExponentialAtmosphere>(std::move(made).value());
    return shared;
}

// Great-circle angle between two points given in degrees.
double central_angle(double lat0, double lon0, double lat1, double lon1) noexcept {
    const double a = lat0 * kDegree;
    const double b = lat1 * kDegree;
    const double dl = (lon1 - lon0) * kDegree;
    const double c = std::sin(a) * std::sin(b) + std::cos(a) * std::cos(b) * std::cos(dl);
    return std::acos(std::clamp(c, -1.0, 1.0));
}

// Appends rows [from, size) of the library's series.
void append_rows(const atmo::EntrySeries& in, std::size_t from, double radius, EntrySeries& out) {
    for (std::size_t i = from; i < in.size(); ++i) {
        EntrySample s;
        s.time_s = in.time_s[i];
        s.altitude_m = in.altitude_m[i];
        s.speed_m_s = in.speed_m_s[i];
        s.flight_path_angle_deg = in.flight_path_angle_deg[i];
        s.g_load = in.g_load[i];
        s.dynamic_pressure_pa = in.dynamic_pressure_pa[i];
        s.heat_flux_w_m2 = in.heat_flux_w_m2[i];
        s.mach = in.mach[i];
        s.latitude_deg = in.latitude_deg[i];
        s.longitude_deg = in.longitude_deg[i];
        s.downrange_m = in.size() > 0 ? radius * central_angle(in.latitude_deg.front(),
                                                               in.longitude_deg.front(),
                                                               s.latitude_deg, s.longitude_deg)
                                      : 0.0;
        out.push(s);
    }
}

// True when every column the workspace reads has the same length.
bool usable(const atmo::EntrySeries& s) noexcept {
    const std::size_t n = s.time_s.size();
    return s.altitude_m.size() == n && s.speed_m_s.size() == n &&
           s.flight_path_angle_deg.size() == n && s.g_load.size() == n &&
           s.dynamic_pressure_pa.size() == n && s.heat_flux_w_m2.size() == n &&
           s.mach.size() == n && s.latitude_deg.size() == n && s.longitude_deg.size() == n;
}

void apply_integration(const EntrySetup& s, atmo::EntryConfig& config) {
    const EntryIntegration& i = s.integration;
    config.frame = s.co_rotating_atmosphere ? atmo::AtmosphereFrame::co_rotating
                                            : atmo::AtmosphereFrame::inertial;
    config.integrator = to_atmo(i.integrator);
    config.ias15_tolerance = i.tolerance;
    config.time_step = core::si::Time{i.time_step_s};
    config.max_coast_step = core::si::Time{i.max_coast_step_s};
    config.stop_on_skip_out = i.stop_on_skip_out;
    if (i.interface_altitude_m) {
        config.planet.entry_interface_altitude = core::si::Length{*i.interface_altitude_m};
    }
}

void read_integration(const atmo::EntryConfig& config, EntrySetup& s) {
    EntryIntegration& i = s.integration;
    s.co_rotating_atmosphere = config.frame == atmo::AtmosphereFrame::co_rotating;
    i.integrator = from_atmo(config.integrator);
    i.tolerance = config.ias15_tolerance;
    i.time_step_s = config.time_step.value();
    i.max_coast_step_s = config.max_coast_step.value();
    i.stop_on_skip_out = config.stop_on_skip_out;
}
class AtmoStepper final : public EntryStepper {
public:
    AtmoStepper(atmo::EntryExperiment run, double radius) : run_(std::move(run)), radius_(radius) {}

    bool advance(double seconds, EntrySeries& out) override {
        if (!run_.finished()) {
            auto advanced = run_.advance(core::si::Time{std::max(seconds, 0.0)});
            if (!advanced) {
                throw std::runtime_error(advanced.error().message);
            }
            if (advanced.value() == atmo::EntryOutcome::failed) {
                throw std::runtime_error("the integrator failed");
            }
        }
        const atmo::EntrySeries& series = run_.series();
        if (!usable(series)) {
            throw std::runtime_error("the entry series have different lengths");
        }
        append_rows(series, copied_, radius_, out);
        copied_ = series.size();
        return run_.finished();
    }

    [[nodiscard]] EntryOutcome outcome() const noexcept override {
        return outcome_from(run_.outcome());
    }
    [[nodiscard]] double time_s() const noexcept override { return run_.time().value(); }
    [[nodiscard]] std::shared_ptr<const scene::EntryResultRecord> record() const override {
        return std::make_shared<const scene::EntryResultRecord>(scene::entry_result(run_));
    }

private:
    atmo::EntryExperiment run_;
    double radius_ = 0.0;
    std::size_t copied_ = 0;
};

} // namespace

atmo::EntryIntegrator to_atmo(EntryIntegrator integrator) noexcept {
    switch (integrator) {
        case EntryIntegrator::yoshida4:
            return atmo::EntryIntegrator::yoshida4;
        case EntryIntegrator::leapfrog_kdk:
            return atmo::EntryIntegrator::leapfrog_kdk;
        case EntryIntegrator::ias15:
            return atmo::EntryIntegrator::ias15;
    }
    return atmo::EntryIntegrator::ias15;
}

EntryIntegrator from_atmo(atmo::EntryIntegrator integrator) noexcept {
    switch (integrator) {
        case atmo::EntryIntegrator::yoshida4:
            return EntryIntegrator::yoshida4;
        case atmo::EntryIntegrator::leapfrog_kdk:
            return EntryIntegrator::leapfrog_kdk;
        case atmo::EntryIntegrator::ias15:
            return EntryIntegrator::ias15;
    }
    return EntryIntegrator::ias15;
}

EntryOutcome outcome_from(atmo::EntryOutcome outcome) noexcept {
    switch (outcome) {
        case atmo::EntryOutcome::impact:
            return EntryOutcome::landed;
        case atmo::EntryOutcome::skip_out:
            return EntryOutcome::skip_out;
        case atmo::EntryOutcome::time_limit:
            return EntryOutcome::orbiting;
        case atmo::EntryOutcome::running:
        case atmo::EntryOutcome::failed:
            return EntryOutcome::none;
    }
    return EntryOutcome::none;
}

EntrySeries series_from(const atmo::EntrySeries& series, double planet_radius_m) {
    EntrySeries out;
    if (!usable(series)) {
        return out;
    }
    out.reserve(series.size());
    append_rows(series, 0, planet_radius_m, out);
    return out;
}

core::Result<atmo::EntryConfig, std::string> entry_config(const EntrySetup& s) {
    if (const std::vector<std::string> errors = setup_errors(s); !errors.empty()) {
        return core::failure(errors.front());
    }
    const std::optional<double> gamma = effective_flight_path_angle(s);
    if (!gamma) {
        return core::failure(std::string("no flight-path angle for this start"));
    }
    atmo::EntryConfig config;
    config.planet = s.planet == EntryPlanet::mars ? atmo::Planet::mars() : atmo::Planet::earth();
    if (s.atmosphere == EntryAtmosphere::exponential) {
        auto made = exponential_atmosphere(s);
        if (!made) {
            return core::failure(std::string(made.error()));
        }
        config.planet.atmosphere = std::move(made).value();
    }
    std::vector<atmo::CdSample> samples;
    samples.reserve(s.cd_table.size());
    for (const MachCd& p : s.cd_table) {
        samples.push_back({p.mach, p.cd});
    }
    auto table =
        atmo::AeroCoefficientTable::create(std::move(samples), core::si::Area{s.reference_area_m2});
    if (!table) {
        return core::failure("Cd(Mach) table: " + table.error().message);
    }
    config.vehicle.name = s.name;
    config.vehicle.mass = core::si::Mass{s.mass_kg};
    config.vehicle.aero = std::move(table).value();
    config.vehicle.nose_radius = core::si::Length{s.nose_radius_m};
    config.initial.altitude = core::si::Length{s.altitude_m};
    config.initial.speed = core::si::Velocity{s.speed_m_s};
    config.initial.flight_path_angle = core::Angle::from_degrees(*gamma);
    config.initial.latitude = core::Angle::from_degrees(s.latitude_deg);
    config.initial.longitude = core::Angle::from_degrees(s.longitude_deg);
    config.initial.heading = core::Angle::from_degrees(s.heading_deg);
    config.max_duration = core::si::Time{s.max_time_s};
    apply_integration(s, config);
    return config;
}

core::Result<std::unique_ptr<EntryStepper>, std::string>
make_entry_stepper(const EntrySetup& setup) {
    auto config = entry_config(setup);
    if (!config) {
        return core::failure(std::string(config.error()));
    }
    const double radius = config.value().planet.radius.value();
    auto run = atmo::EntryExperiment::create(std::move(config).value());
    if (!run) {
        return core::failure(run.error().message);
    }
    std::unique_ptr<EntryStepper> stepper =
        std::make_unique<AtmoStepper>(std::move(run).value(), radius);
    return stepper;
}

core::Result<EntryRunResult, std::string> run_entry(const EntrySetup& setup) {
    auto made = make_entry_stepper(setup);
    if (!made) {
        return core::failure(std::string(made.error()));
    }
    EntryStepper& stepper = *made.value();
    EntryRunResult result;
    try {
        while (!stepper.advance(60.0, result.series)) {
        }
    } catch (const std::exception& error) {
        return core::failure(std::string(error.what()));
    }
    result.summary = summarize(result.series, stepper.outcome());
    result.record = stepper.record();
    return result;
}

EntrySetup setup_from_config(const atmo::EntryConfig& config, std::string name) {
    EntrySetup s;
    s.name = std::move(name);
    s.planet = config.planet.name == "mars" ? EntryPlanet::mars : EntryPlanet::earth;
    s.atmosphere = EntryAtmosphere::standard;
    s.mass_kg = config.vehicle.mass.value();
    s.reference_area_m2 = config.vehicle.aero.reference_area().value();
    s.nose_radius_m = config.vehicle.nose_radius.value();
    s.cd_table.clear();
    for (const atmo::CdSample& sample : config.vehicle.aero.samples()) {
        s.cd_table.push_back({sample.mach, sample.cd});
    }
    s.cd_source = CdSource::table;
    s.start = EntryStart::state;
    s.altitude_m = config.initial.altitude.value();
    s.speed_m_s = config.initial.speed.value();
    s.flight_path_angle_deg = config.initial.flight_path_angle.degrees();
    s.latitude_deg = config.initial.latitude.degrees();
    s.longitude_deg = config.initial.longitude.degrees();
    s.heading_deg = config.initial.heading.degrees();
    s.max_time_s = config.max_duration.value();
    read_integration(config, s);
    return s;
}

std::vector<EntryPreset> entry_presets() {
    // The physics library's scenarios (docs/atmosphere.md, "Scenarios"): the
    // ballistic Apollo-like lunar return and its skip-out variant.
    // Placed near Apollo 11's entry interface over the western Pacific,
    // heading north-east; the air rests in the inertial frame, so the place
    // changes the ground track only, not the dynamics.
    const auto pacific = [](EntrySetup s) {
        s.latitude_deg = -3.2;
        s.longitude_deg = 171.6;
        s.heading_deg = 52.0;
        return s;
    };
    const EntrySetup apollo =
        pacific(setup_from_config(atmo::apollo_like_entry(), "Apollo-like entry"));
    const EntrySetup skip =
        pacific(setup_from_config(atmo::skip_out_entry(), "Grazing lunar return (skip-out)"));
    EntrySetup leo = apollo;
    leo.name = "Low Earth orbit return";
    leo.speed_m_s = 7'800.0;
    leo.flight_path_angle_deg = -1.5;
    // Mars Science Laboratory: 4.5 m 70-degree sphere-cone, 1.125 m nose
    // radius, about 3.3 t, 5.8 km/s at the entry interface.
    EntrySetup mars;
    mars.name = "Mars entry (MSL-like)";
    mars.planet = EntryPlanet::mars;
    mars.mass_kg = 3'300.0;
    mars.reference_area_m2 = 15.9;
    mars.nose_radius_m = 1.125;
    mars.cd_table = {{0.5, 1.00}, {1.5, 1.40}, {3.0, 1.60}, {10.0, 1.68}, {30.0, 1.68}};
    mars.altitude_m = 125'000.0;
    mars.speed_m_s = 5'800.0;
    mars.flight_path_angle_deg = -15.5;
    mars.latitude_deg = -4.6;
    mars.longitude_deg = 137.4;
    mars.heading_deg = 100.0;
    return {{"Apollo-like lunar return", apollo},
            {"Grazing lunar return (skip-out)", skip},
            {"Low Earth orbit return", leo},
            {"Mars entry (MSL-like)", mars}};
}
std::string_view cd_body_name(CdBody body) noexcept {
    switch (body) {
        case CdBody::apollo_capsule:
            return "Apollo-type capsule";
        case CdBody::orion_capsule:
            return "Orion-type capsule";
        case CdBody::sphere:
            return "Sphere";
    }
    return "Sphere";
}

core::Result<CdSamples, std::string> sample_cd_table(const CdSampling& sampling) {
    if (!(sampling.reference_area_m2 > 0.0) || !std::isfinite(sampling.reference_area_m2)) {
        return core::failure(std::string("the reference area must be positive"));
    }
    scene::GeometryRef ref;
    ref.source = scene::GeometrySource::primitive;
    ref.shape =
        sampling.body == CdBody::sphere ? scene::AeroShape::sphere : scene::AeroShape::capsule;
    ref.capsule = sampling.body == CdBody::orion_capsule ? "orion" : "apollo";
    ref.dimensions.diameter_m = std::sqrt(4.0 * sampling.reference_area_m2 / std::numbers::pi);
    auto built = build_primitive(ref);
    if (!built) {
        return core::failure(built.error().title + ": " + built.error().detail);
    }
    scene::AeroSolverSettings settings;
    settings.reference_area = geom::ReferenceAreaKind::frontal;
    atmo::CdSamplingRequest request;
    request.geometry = to_aero_geometry(built.value(), settings);
    request.solver_override = aero::SolverKind::newtonian;
    // Cold upper-atmosphere air: Newtonian Cd depends on Mach only.
    request.freestream.temperature = 250.0;
    request.freestream.density = 1.0e-3;
    auto sampled = atmo::sample_cd_table(request, atmo::default_aero_solver_factory());
    if (!sampled) {
        return core::failure(sampled.error().message);
    }
    CdSamples samples;
    samples.label = built.value().label;
    for (const atmo::CdSamplingPoint& point : sampled.value().points) {
        samples.table.push_back({point.mach, point.cd});
        for (const std::string& warning : point.warnings) {
            std::string line = "Mach " + format_significant(point.mach, 3);
            line += ": ";
            line += warning;
            samples.warnings.push_back(std::move(line));
        }
    }
    return samples;
}

} // namespace orrery::sim
