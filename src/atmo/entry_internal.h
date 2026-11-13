#pragma once

// Private state of EntryExperiment. Not part of the public include directory.

#include <memory>
#include <optional>
#include <vector>

#include "orrery/atmo/entry.h"
#include "orrery/core/body_store.h"
#include "orrery/core/force_model.h"
#include "orrery/core/integrator.h"
#include "orrery/core/sim_state.h"
#include "orrery/core/vec3.h"

namespace orrery::atmo {

// Index of the planet and of the probe in the entry's BodyStore.
inline constexpr std::size_t kPlanetIndex = 0;
inline constexpr std::size_t kProbeIndex = 1;

// One row of the series before it is appended, plus the drag power used by
// the work quadrature.
struct EntryRow {
    double time_s = 0.0;
    double altitude_m = 0.0;
    double speed_m_s = 0.0;
    double flight_path_angle_deg = 0.0;
    double airspeed_m_s = 0.0;
    double mach = 0.0;
    double cd = 0.0;
    double density_kg_m3 = 0.0;
    double dynamic_pressure_pa = 0.0;
    double g_load = 0.0;
    double heat_flux_w_m2 = 0.0;
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double drag_work_j = 0.0;
    double energy_j = 0.0;
    // F . v (inertial velocity), W.
    double drag_power_w = 0.0;
};

struct EntryExperiment::Impl {
    EntryConfig config;
    core::SimState state;
    core::NewtonianGravity gravity;
    std::unique_ptr<core::IIntegrator> integrator;
    AtmosphericDrag drag;
    // Copy of the bodies whose probe velocity is set per kick stage.
    core::BodyStore stage_bodies;
    std::vector<core::Vec3<core::space::Length>> stage_positions;
    std::vector<core::Vec3<core::space::Acceleration>> stage_accelerations;

    EntrySeries series;
    std::vector<double> drag_power_w;
    std::vector<EntryEvent> events;
    EntrySummary summary;
    double time_s = 0.0;
    double drag_work_j = 0.0;
    double mass_kg = 0.0;
    bool inside_interface = false;

    Impl(EntryConfig config, core::SimState state, std::unique_ptr<core::IIntegrator> integrator,
         AtmosphericDrag drag);

    [[nodiscard]] static core::Result<std::unique_ptr<Impl>, AtmoError> create(EntryConfig config);

    [[nodiscard]] core::Result<EntryOutcome, AtmoError> advance(double duration_s);
    [[nodiscard]] EntryEnergyBalance energy_balance() const noexcept;

    // Records the initial row and the entry event of a start inside the
    // interface.
    void start();
    // Split step of length dt (seconds): kick, gravity, kick; the clock then
    // reads new_time_s. On a failed kick or gravity step the state is the one
    // before the step, the outcome is failed and the error is returned.
    [[nodiscard]] std::optional<AtmoError> step(double dt_s, double new_time_s);
    [[nodiscard]] std::optional<AtmoError> split_step(double dt_s);
    // Velocity kick of the drag over tau seconds at fixed position: RK4 in
    // substeps of at most kDragSubstepFraction of the drag timescale. Fails
    // on a non-finite stage instead of leaving the velocity unchanged.
    [[nodiscard]] std::optional<AtmoError> kick(double tau_s);
    // One RK4 substep of h seconds from v (first stage k1); false on a
    // non-finite stage.
    [[nodiscard]] bool rk4_drag(core::Vec3<core::space::Velocity>& v,
                                const core::Vec3<core::space::Acceleration>& k1, double h_s);
    // Longest substep [s] at `velocity` under drag deceleration `acceleration`.
    [[nodiscard]] double
    drag_substep_limit(const core::Vec3<core::space::Velocity>& velocity,
                       const core::Vec3<core::space::Acceleration>& acceleration) const;
    // Drag acceleration of the probe at the staged position; nullopt when it
    // or the velocity is not finite.
    [[nodiscard]] std::optional<core::Vec3<core::space::Acceleration>>
    drag_acceleration(const core::Vec3<core::space::Velocity>& velocity);
    [[nodiscard]] double next_step() const;

    [[nodiscard]] EntryRow measure() const;
    void append(const EntryRow& row);
    void replace_last(const EntryRow& row);
    [[nodiscard]] EntryRow last_row() const;
    void detect_events(const EntryRow& before, const EntryRow& after);
    void update_summary(const EntryRow& before, const EntryRow& after);
};

} // namespace orrery::atmo
