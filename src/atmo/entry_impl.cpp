#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "convert.h"
#include "entry_internal.h"
#include "orrery/core/ias15.h"
#include "orrery/core/units.h"

namespace orrery::atmo {

namespace {

using Vec3L = core::Vec3<core::si::Length>;
using Vec3V = core::Vec3<core::si::Velocity>;

bool positive_finite(double v) noexcept {
    return v > 0.0 && std::isfinite(v);
}

// Empty when the planet can carry an entry.
std::string planet_problem(const Planet& p) {
    if (!p.atmosphere) {
        return "planet: no atmosphere";
    }
    if (!positive_finite(p.gm.value()) || !positive_finite(p.radius.value())) {
        return "planet: GM and radius must be positive and finite";
    }
    if (!std::isfinite(p.rotation_rate.value())) {
        return "planet: rotation rate must be finite";
    }
    if (!(p.sutton_graves_k >= 0.0) || !std::isfinite(p.sutton_graves_k)) {
        return "planet: Sutton-Graves constant must be non-negative and finite";
    }
    if (!positive_finite(p.entry_interface_altitude.value())) {
        return "planet: entry interface altitude must be positive and finite";
    }
    return {};
}

// Empty when the vehicle, initial state and settings are usable.
std::string config_problem(const EntryConfig& c) {
    if (std::string problem = planet_problem(c.planet); !problem.empty()) {
        return problem;
    }
    if (!positive_finite(c.vehicle.mass.value()) ||
        !positive_finite(c.vehicle.nose_radius.value())) {
        return "vehicle: mass and nose radius must be positive and finite";
    }
    const EntryInitialState& s = c.initial;
    if (!positive_finite(s.altitude.value())) {
        return "initial: altitude must be positive and finite";
    }
    if (!(s.speed.value() >= 0.0) || !std::isfinite(s.speed.value())) {
        return "initial: speed must be non-negative and finite";
    }
    for (const core::Angle a : {s.flight_path_angle, s.latitude, s.longitude, s.heading}) {
        if (!std::isfinite(a.radians())) {
            return "initial: angles must be finite";
        }
    }
    if (!positive_finite(c.time_step.value()) || !positive_finite(c.max_duration.value()) ||
        !std::isfinite(c.max_coast_step.value()) || c.max_coast_step < c.time_step) {
        return "settings: time step and duration must be positive and finite, max coast step "
               "at least the time step";
    }
    if (c.integrator == EntryIntegrator::ias15 && !positive_finite(c.ias15_tolerance)) {
        return "settings: IAS15 tolerance must be positive and finite";
    }
    return {};
}

// Planet-centred inertial position and velocity of the initial state.
std::pair<Vec3L, Vec3V> initial_vectors(const Planet& planet, const EntryInitialState& s) {
    const double lat = s.latitude.radians();
    const double lon = s.longitude.radians();
    const double gamma = s.flight_path_angle.radians();
    const double psi = s.heading.radians();
    const double up[3] = {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon),
                          std::sin(lat)};
    const double east[3] = {-std::sin(lon), std::cos(lon), 0.0};
    const double north[3] = {-std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon),
                             std::cos(lat)};
    const double r = planet.radius.value() + s.altitude.value();
    const double v = s.speed.value();
    double dir[3] = {};
    for (int i = 0; i < 3; ++i) {
        dir[i] = std::sin(gamma) * up[i] +
                 std::cos(gamma) * (std::cos(psi) * north[i] + std::sin(psi) * east[i]);
    }
    return {Vec3L{core::si::Length(r * up[0]), core::si::Length(r * up[1]),
                  core::si::Length(r * up[2])},
            Vec3V{core::si::Velocity(v * dir[0]), core::si::Velocity(v * dir[1]),
                  core::si::Velocity(v * dir[2])}};
}

// The two-body state at time 0 in `state`; false if the store rejected a
// body (not reachable after config_problem, kept as a guard).
bool initial_state(const EntryConfig& c, core::SimState& state) {
    const auto [r, v] = initial_vectors(c.planet, c.initial);
    // M = GM / G in space units, so that G M reproduces GM exactly.
    const core::space::GravitationalParameter gm = core::to_space(c.planet.gm);
    const core::space::Mass planet_mass(gm.value() /
                                        core::constants::gravitational_constant_space.value());
    core::BodyStore bodies;
    const auto planet =
        bodies.add(core::BodyInit{c.planet.name, core::Planet{}, planet_mass, {}, {}});
    const auto probe = bodies.add(core::BodyInit{c.vehicle.name, core::Probe{core::AeroRef{0}},
                                                 core::to_space(c.vehicle.mass),
                                                 detail::to_space_vec(r), detail::to_space_vec(v)});
    if (!planet || !probe) {
        return false;
    }
    state = core::SimState(core::space::Time(0.0), std::move(bodies));
    return true;
}

std::unique_ptr<core::IIntegrator> make_integrator(const EntryConfig& c) {
    switch (c.integrator) {
        case EntryIntegrator::ias15: {
            auto ias15 = core::IAS15::create(c.ias15_tolerance);
            return std::make_unique<core::IAS15>(std::move(ias15).value());
        }
        case EntryIntegrator::yoshida4:
            return core::make_integrator(core::IntegratorKind::yoshida4);
        case EntryIntegrator::leapfrog_kdk:
            return core::make_integrator(core::IntegratorKind::leapfrog_kdk);
    }
    return core::make_integrator(core::IntegratorKind::leapfrog_kdk);
}

} // namespace

EntryExperiment::Impl::Impl(EntryConfig config_in, core::SimState state_in,
                            std::unique_ptr<core::IIntegrator> integrator_in,
                            AtmosphericDrag drag_in)
    : config(std::move(config_in)), state(std::move(state_in)),
      integrator(std::move(integrator_in)), drag(std::move(drag_in)), stage_bodies(state.bodies()),
      stage_positions(state.bodies().size()), stage_accelerations(state.bodies().size()),
      mass_kg(config.vehicle.mass.value()) {}

core::Result<std::unique_ptr<EntryExperiment::Impl>, AtmoError>
EntryExperiment::Impl::create(EntryConfig config) {
    if (std::string problem = config_problem(config); !problem.empty()) {
        return core::failure(AtmoError{AtmoErrorCode::invalid_parameter, std::move(problem)});
    }
    core::SimState state;
    if (!initial_state(config, state)) {
        return core::failure(
            AtmoError{AtmoErrorCode::invalid_parameter, "the initial bodies are not valid"});
    }
    auto drag = AtmosphericDrag::create(config.planet, state.bodies().id(kPlanetIndex),
                                        std::vector<AeroCoefficientTable>{config.vehicle.aero},
                                        config.frame);
    if (!drag) {
        return core::failure(drag.error());
    }
    std::unique_ptr<core::IIntegrator> integrator = make_integrator(config);
    auto impl = std::make_unique<Impl>(std::move(config), std::move(state), std::move(integrator),
                                       std::move(drag).value());
    impl->start();
    return impl;
}

core::Result<EntryOutcome, AtmoError> EntryExperiment::Impl::advance(double duration_s) {
    if (summary.outcome != EntryOutcome::running) {
        return summary.outcome;
    }
    if (!(duration_s >= 0.0) || !std::isfinite(duration_s)) {
        return core::failure(AtmoError{AtmoErrorCode::invalid_parameter,
                                       "advance: duration must be non-negative and finite"});
    }
    const double limit = config.max_duration.value();
    const double end = std::min(time_s + duration_s, limit);
    while (summary.outcome == EntryOutcome::running && time_s < end) {
        const double remaining = limit - time_s;
        const double next = next_step();
        const bool last = remaining <= next;
        if (std::optional<AtmoError> failed =
                step(last ? remaining : next, last ? limit : time_s + next)) {
            return core::failure(std::move(*failed));
        }
    }
    if (summary.outcome == EntryOutcome::running && time_s >= limit) {
        summary.outcome = EntryOutcome::time_limit;
    }
    return summary.outcome;
}

std::optional<AtmoError> EntryExperiment::Impl::step(double dt_s, double new_time_s) {
    const EntryRow before = last_row();
    const core::SimState backup = state;
    const double work_before = drag_work_j;
    kick(0.5 * dt_s);
    const auto gravity_step =
        integrator->step(state, core::to_space(core::si::Time(dt_s)), gravity);
    if (!gravity_step) {
        state = backup;
        drag_work_j = work_before;
        summary.outcome = EntryOutcome::failed;
        return AtmoError{AtmoErrorCode::integration_failed,
                         "gravity step: " + std::string(core::to_string(gravity_step.error()))};
    }
    kick(0.5 * dt_s);
    time_s = new_time_s;
    append(measure());
    detect_events(before, last_row());
    update_summary(before, last_row());
    return std::nullopt;
}

void EntryExperiment::Impl::kick(double tau_s) {
    const core::space::Time tau = core::to_space(core::si::Time(tau_s));
    const core::BodyStore& bodies = state.bodies();
    stage_positions[kPlanetIndex] = bodies.position(kPlanetIndex);
    stage_positions[kProbeIndex] = bodies.position(kProbeIndex);
    const core::Vec3<core::space::Velocity> v0 = bodies.velocity(kProbeIndex);
    const auto k1 = drag_acceleration(v0);
    if (k1 == core::Vec3<core::space::Acceleration>{}) {
        return; // above the atmosphere: nothing to integrate
    }
    const auto k2 = drag_acceleration(v0 + k1 * (0.5 * tau));
    const auto k3 = drag_acceleration(v0 + k2 * (0.5 * tau));
    const auto k4 = drag_acceleration(v0 + k3 * tau);
    const core::Vec3<core::space::Velocity> v1 = v0 + (k1 + 2.0 * k2 + 2.0 * k3 + k4) * (tau / 6.0);
    if (!state.bodies().set_velocity(kProbeIndex, v1)) {
        return; // non-finite drag: the velocity is left unchanged
    }
    const double speed0 = core::to_si(core::norm(v0)).value();
    const double speed1 = core::to_si(core::norm(v1)).value();
    drag_work_j += 0.5 * mass_kg * (speed1 * speed1 - speed0 * speed0);
}

core::Vec3<core::space::Acceleration>
EntryExperiment::Impl::drag_acceleration(const core::Vec3<core::space::Velocity>& velocity) {
    if (!stage_bodies.set_velocity(kProbeIndex, velocity)) {
        return core::Vec3<core::space::Acceleration>{};
    }
    drag.accelerations(stage_bodies, stage_positions, stage_accelerations);
    return stage_accelerations[kProbeIndex];
}

double EntryExperiment::Impl::next_step() const {
    const double dt = config.time_step.value();
    const double max_coast = config.max_coast_step.value();
    const core::BodyStore& bodies = state.bodies();
    const Vec3L r = detail::to_si_vec(bodies.position(kProbeIndex));
    const Vec3V v = detail::to_si_vec(bodies.velocity(kProbeIndex));
    const double radius = core::norm(r).value();
    const double above_top =
        radius - config.planet.radius.value() - config.planet.atmosphere->top().value();
    if (!(above_top > 0.0)) {
        return dt;
    }
    const double radial_speed = core::dot(r, v).value() / radius;
    const double coast = radial_speed < 0.0 ? 0.5 * above_top / -radial_speed : max_coast;
    return std::clamp(coast, dt, max_coast);
}

} // namespace orrery::atmo
