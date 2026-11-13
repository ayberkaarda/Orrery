#pragma once

#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

#include "orrery/atmo/aero_table.h"
#include "orrery/atmo/atmosphere.h"
#include "orrery/atmo/error.h"
#include "orrery/atmo/planet.h"
#include "orrery/core/body.h"
#include "orrery/core/body_store.h"
#include "orrery/core/force_model.h"
#include "orrery/core/quantity.h"
#include "orrery/core/result.h"
#include "orrery/core/vec3.h"

namespace orrery::atmo {

// Motion of the air the drag is computed against.
//   inertial:    the atmosphere is at rest in the planet-centred inertial
//                frame (the default, ADR-012: the simplest model, and the one
//                whose drag work balance needs no frame bookkeeping).
//   co_rotating: the atmosphere turns rigidly with the planet, wind
//                omega x r (no winds relative to the surface).
// APPROXIMATION (E7): no winds; the inertial default misses up to omega R
// (465 m/s at Earth's equator) of airspeed - small against entry speeds
// (4 percent at 11 km/s) - use co_rotating for ground-relative studies.
enum class AtmosphereFrame : std::uint8_t { inertial, co_rotating };

[[nodiscard]] std::string_view to_string(AtmosphereFrame frame) noexcept;

// Velocity of a vehicle relative to the air at planet-centred position r for
// inertial velocity v: v for an inertial atmosphere, v - omega z x r for a
// co-rotating one.
[[nodiscard]] core::Vec3<core::si::Velocity>
air_relative_velocity(const core::Vec3<core::si::Length>& position,
                      const core::Vec3<core::si::Velocity>& velocity,
                      core::si::Frequency rotation_rate, AtmosphereFrame frame) noexcept;

// The drag on one vehicle at one instant: the typed value that leaves the
// atmosphere domain (ADR-014), with the quantities it was computed from.
struct DragForce {
    // F = -1/2 rho |v_rel| v_rel Cd(Mach) A, planet-centred inertial axes, N.
    core::Vec3<core::si::Force> force{};
    core::si::Length altitude{};
    // |v_rel|.
    core::si::Velocity airspeed{};
    double mach = 0.0;
    double cd = 0.0;
    // The Mach number was outside the Cd table (end value used).
    bool cd_clamped = false;
    // 1/2 rho |v_rel|^2.
    core::si::Pressure dynamic_pressure{};
    AtmosphereSample atmosphere{};
};

// Drag of one vehicle with Cd(Mach) table `table` at planet-centred position
// `position` and inertial velocity `velocity` (SI).
// APPROXIMATION (ADR-012): drag only (no lift or side force), heat shield
// always forward (no attitude dynamics), Cd a function of Mach only.
// APPROXIMATION (E6): no Reynolds or Knudsen dependence, continuum Cd in the
// rarefied upper atmosphere - the deceleration and heating pulses happen
// below about 90 km - valid for ballistic bodies there.
[[nodiscard]] DragForce drag_force(const Planet& planet, const AeroCoefficientTable& table,
                                   const core::Vec3<core::si::Length>& position,
                                   const core::Vec3<core::si::Velocity>& velocity,
                                   AtmosphereFrame frame) noexcept;

// Atmospheric drag on the probes of a BodyStore as a core force model.
//
// The atmosphere belongs to body `planet_body`; every probe whose
// Probe::aero_ref.value indexes `tables` is decelerated by drag_force()
// relative to that body; every other body gets zero. Accelerations are in
// space units (the force divided by the probe's mass, converted from SI).
//
// Velocity dependence: core force models are functions of positions only, so
// this model reads the velocities from the BodyStore it is given (those of
// the instant the store describes, not of an integrator's trial stage). It is
// therefore applied by EntryExperiment as a velocity kick at fixed positions
// (Strang splitting around the gravity step, docs/atmosphere.md), never by
// handing it to an integrator as an ordinary position-dependent force.
// Drag is dissipative: potential_energy() is zero.
class AtmosphericDrag final : public core::IForceModel {
public:
    // Fails with invalid_parameter for a planet without an atmosphere or with
    // a non-finite or non-positive radius.
    [[nodiscard]] static core::Result<AtmosphericDrag, AtmoError>
    create(Planet planet, core::BodyId planet_body, std::vector<AeroCoefficientTable> tables,
           AtmosphereFrame frame = AtmosphereFrame::inertial);

    [[nodiscard]] std::string_view name() const noexcept override { return "atmospheric_drag"; }

    // Throws std::invalid_argument for mismatched spans (IForceModel), a
    // missing planet body, or a probe whose aero_ref has no table.
    void accelerations(const core::BodyStore& bodies,
                       std::span<const core::Vec3<core::space::Length>> positions,
                       std::span<core::Vec3<core::space::Acceleration>> out) const override;

    [[nodiscard]] core::space::Energy
    potential_energy(const core::BodyStore& bodies,
                     std::span<const core::Vec3<core::space::Length>> positions) const override;

    // drag_force() for the table of `ref` (std::out_of_range if it has none).
    [[nodiscard]] DragForce evaluate(core::AeroRef ref,
                                     const core::Vec3<core::si::Length>& position,
                                     const core::Vec3<core::si::Velocity>& velocity) const;

    [[nodiscard]] const Planet& planet() const noexcept { return planet_; }
    [[nodiscard]] core::BodyId planet_body() const noexcept { return planet_body_; }
    [[nodiscard]] AtmosphereFrame frame() const noexcept { return frame_; }
    [[nodiscard]] std::span<const AeroCoefficientTable> tables() const noexcept { return tables_; }

private:
    AtmosphericDrag(Planet planet, core::BodyId planet_body,
                    std::vector<AeroCoefficientTable> tables, AtmosphereFrame frame) noexcept;

    Planet planet_;
    core::BodyId planet_body_;
    std::vector<AeroCoefficientTable> tables_;
    AtmosphereFrame frame_ = AtmosphereFrame::inertial;
};

} // namespace orrery::atmo
