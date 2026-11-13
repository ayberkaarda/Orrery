#include "orrery/atmo/drag.h"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>

#include "convert.h"
#include "orrery/core/units.h"

namespace orrery::atmo {

namespace {

bool is_probe(const core::BodyKind& kind) noexcept {
    return std::holds_alternative<core::Probe>(kind);
}

} // namespace

std::string_view to_string(AtmosphereFrame frame) noexcept {
    switch (frame) {
        case AtmosphereFrame::inertial:
            return "inertial";
        case AtmosphereFrame::co_rotating:
            return "co_rotating";
    }
    return "unknown";
}

core::Vec3<core::si::Velocity> air_relative_velocity(const core::Vec3<core::si::Length>& position,
                                                     const core::Vec3<core::si::Velocity>& velocity,
                                                     core::si::Frequency rotation_rate,
                                                     AtmosphereFrame frame) noexcept {
    if (frame == AtmosphereFrame::inertial) {
        return velocity;
    }
    // Wind of a rigidly rotating atmosphere: omega z x r = (-w y, w x, 0).
    const core::Vec3<core::si::Velocity> wind{-rotation_rate * position.y,
                                              rotation_rate * position.x, core::si::Velocity(0.0)};
    return velocity - wind;
}

DragForce drag_force(const Planet& planet, const AeroCoefficientTable& table,
                     const core::Vec3<core::si::Length>& position,
                     const core::Vec3<core::si::Velocity>& velocity,
                     AtmosphereFrame frame) noexcept {
    DragForce out;
    out.altitude = core::norm(position) - planet.radius;
    out.atmosphere = planet.atmosphere->sample(out.altitude);
    const core::Vec3<core::si::Velocity> relative =
        air_relative_velocity(position, velocity, planet.rotation_rate, frame);
    out.airspeed = core::norm(relative);
    out.mach = mach_number(out.atmosphere, out.airspeed);
    const CdLookup cd = table.lookup(out.mach);
    out.cd = cd.cd;
    out.cd_clamped = cd.clamped;
    const double rho = out.atmosphere.density.value();
    const double speed = out.airspeed.value();
    out.dynamic_pressure = core::si::Pressure(0.5 * rho * speed * speed);
    if (rho > 0.0 && speed > 0.0) {
        // F = -q Cd A v_rel / |v_rel|.
        const double k =
            -out.dynamic_pressure.value() * out.cd * table.reference_area().value() / speed;
        out.force = core::Vec3<core::si::Force>{core::si::Force(k * relative.x.value()),
                                                core::si::Force(k * relative.y.value()),
                                                core::si::Force(k * relative.z.value())};
    }
    return out;
}

AtmosphericDrag::AtmosphericDrag(Planet planet, core::BodyId planet_body,
                                 std::vector<AeroCoefficientTable> tables,
                                 AtmosphereFrame frame) noexcept
    : planet_(std::move(planet)), planet_body_(planet_body), tables_(std::move(tables)),
      frame_(frame) {}

core::Result<AtmosphericDrag, AtmoError>
AtmosphericDrag::create(Planet planet, core::BodyId planet_body,
                        std::vector<AeroCoefficientTable> tables, AtmosphereFrame frame) {
    if (!planet.atmosphere) {
        return core::failure(
            AtmoError{AtmoErrorCode::invalid_parameter, "the planet has no atmosphere"});
    }
    const double radius = planet.radius.value();
    if (!(radius > 0.0) || !std::isfinite(radius) || !std::isfinite(planet.rotation_rate.value())) {
        return core::failure(AtmoError{AtmoErrorCode::invalid_parameter,
                                       "the planet radius must be positive and finite and its "
                                       "rotation rate finite"});
    }
    return AtmosphericDrag(std::move(planet), planet_body, std::move(tables), frame);
}

DragForce AtmosphericDrag::evaluate(core::AeroRef ref, const core::Vec3<core::si::Length>& position,
                                    const core::Vec3<core::si::Velocity>& velocity) const {
    return drag_force(planet_, tables_.at(ref.value), position, velocity, frame_);
}

void AtmosphericDrag::accelerations(const core::BodyStore& bodies,
                                    std::span<const core::Vec3<core::space::Length>> positions,
                                    std::span<core::Vec3<core::space::Acceleration>> out) const {
    if (positions.size() != bodies.size() || out.size() != bodies.size()) {
        throw std::invalid_argument("AtmosphericDrag: positions and out must match the bodies");
    }
    const auto planet_index = bodies.index_of(planet_body_);
    if (!planet_index) {
        throw std::invalid_argument("AtmosphericDrag: the planet body is not in the store");
    }
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        const auto* probe = std::get_if<core::Probe>(&bodies.kind(i));
        if (probe != nullptr && i != *planet_index && probe->aero_ref.value >= tables_.size()) {
            throw std::invalid_argument("AtmosphericDrag: probe '" + bodies.name(i) +
                                        "' has no Cd table");
        }
    }
    const std::size_t p = *planet_index;
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        out[i] = core::Vec3<core::space::Acceleration>{};
        if (i == p || !is_probe(bodies.kind(i))) {
            continue;
        }
        const core::AeroRef ref = std::get<core::Probe>(bodies.kind(i)).aero_ref;
        const DragForce drag = evaluate(ref, detail::to_si_vec(positions[i] - positions[p]),
                                        detail::to_si_vec(bodies.velocity(i) - bodies.velocity(p)));
        const double mass = core::to_si(bodies.mass(i)).value();
        out[i] = detail::to_space_vec(core::Vec3<core::si::Acceleration>{
            core::si::Acceleration(drag.force.x.value() / mass),
            core::si::Acceleration(drag.force.y.value() / mass),
            core::si::Acceleration(drag.force.z.value() / mass)});
    }
}

core::space::Energy AtmosphericDrag::potential_energy(
    const core::BodyStore& bodies,
    std::span<const core::Vec3<core::space::Length>> positions) const {
    if (positions.size() != bodies.size()) {
        throw std::invalid_argument("AtmosphericDrag: positions must match the bodies");
    }
    return core::space::Energy(0.0);
}

} // namespace orrery::atmo
