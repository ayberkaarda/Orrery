#include "orrery/core/integrator.h"

#include <array>
#include <cstddef>
#include <span>
#include <stdexcept>

namespace orrery::core {

namespace {

// Copies positions and velocities out of the store and sizes the scratch
// columns. Allocation (if any) happens here, before the state is modified.
void load(const BodyStore& bodies, detail::PhaseBuffers& buffers) {
    const std::size_t n = bodies.size();
    buffers.position.resize(n);
    buffers.velocity.resize(n);
    buffers.acceleration.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        buffers.position[i] = bodies.position(i);
        buffers.velocity[i] = bodies.velocity(i);
    }
}

// Writes the new phase-space state back, all or nothing: every component is
// checked first, so a non-finite result leaves the state unchanged.
Result<space::Time, IntegrationError> commit(SimState& state, const detail::PhaseBuffers& buffers,
                                             space::Time dt) {
    const std::size_t n = buffers.position.size();
    for (std::size_t i = 0; i < n; ++i) {
        if (!isfinite(buffers.position[i]) || !isfinite(buffers.velocity[i])) {
            return failure(IntegrationError::non_finite_state);
        }
    }
    const space::Time new_time = state.time() + dt;
    if (!isfinite(new_time)) {
        return failure(IntegrationError::non_finite_state);
    }
    BodyStore& bodies = state.bodies();
    for (std::size_t i = 0; i < n; ++i) {
        // Cannot fail: every component was checked above.
        if (!bodies.set_position(i, buffers.position[i]) ||
            !bodies.set_velocity(i, buffers.velocity[i])) {
            return failure(IntegrationError::non_finite_state);
        }
    }
    state.set_time(new_time);
    return new_time;
}

void kick(std::span<Vec3<space::Velocity>> velocity,
          std::span<const Vec3<space::Acceleration>> acceleration, space::Time h) {
    for (std::size_t i = 0; i < velocity.size(); ++i) {
        velocity[i] += acceleration[i] * h;
    }
}

void drift(std::span<Vec3<space::Length>> position, std::span<const Vec3<space::Velocity>> velocity,
           space::Time h) {
    for (std::size_t i = 0; i < position.size(); ++i) {
        position[i] += velocity[i] * h;
    }
}

void evaluate(const IForceModel& forces, const BodyStore& bodies, detail::PhaseBuffers& buffers) {
    forces.accelerations(bodies, buffers.position, buffers.acceleration);
}

} // namespace

std::string_view to_string(IntegrationError error) noexcept {
    switch (error) {
        case IntegrationError::invalid_time_step:
            return "time step is not finite";
        case IntegrationError::non_finite_state:
            return "integration produced a non-finite position or velocity";
    }
    return "unknown integration error";
}

Result<space::Time, IntegrationError> LeapfrogKdk::step(SimState& state, space::Time dt,
                                                        const IForceModel& forces) {
    if (!isfinite(dt)) {
        return failure(IntegrationError::invalid_time_step);
    }
    const BodyStore& bodies = state.bodies();
    load(bodies, buffers_);
    const space::Time half = dt * 0.5;

    evaluate(forces, bodies, buffers_);
    kick(buffers_.velocity, buffers_.acceleration, half);
    drift(buffers_.position, buffers_.velocity, dt);
    evaluate(forces, bodies, buffers_);
    kick(buffers_.velocity, buffers_.acceleration, half);

    return commit(state, buffers_, dt);
}

Result<space::Time, IntegrationError> Yoshida4::step(SimState& state, space::Time dt,
                                                     const IForceModel& forces) {
    if (!isfinite(dt)) {
        return failure(IntegrationError::invalid_time_step);
    }
    const BodyStore& bodies = state.bodies();
    load(bodies, buffers_);

    // Kick and drift coefficients of the merged triple jump (see header).
    const space::Time kick_outer = dt * (0.5 * w1);
    const space::Time kick_inner = dt * (0.5 * (w1 + w0));
    const space::Time drift_outer = dt * w1;
    const space::Time drift_inner = dt * w0;

    evaluate(forces, bodies, buffers_);
    kick(buffers_.velocity, buffers_.acceleration, kick_outer);
    drift(buffers_.position, buffers_.velocity, drift_outer);
    evaluate(forces, bodies, buffers_);
    kick(buffers_.velocity, buffers_.acceleration, kick_inner);
    drift(buffers_.position, buffers_.velocity, drift_inner);
    evaluate(forces, bodies, buffers_);
    kick(buffers_.velocity, buffers_.acceleration, kick_inner);
    drift(buffers_.position, buffers_.velocity, drift_outer);
    evaluate(forces, bodies, buffers_);
    kick(buffers_.velocity, buffers_.acceleration, kick_outer);

    return commit(state, buffers_, dt);
}

Result<space::Time, IntegrationError> Rk4::step(SimState& state, space::Time dt,
                                                const IForceModel& forces) {
    if (!isfinite(dt)) {
        return failure(IntegrationError::invalid_time_step);
    }
    const BodyStore& bodies = state.bodies();
    load(bodies, buffers_);
    const std::size_t n = bodies.size();
    stage_position_.resize(n);
    stage_velocity_.resize(n);
    sum_kx_.resize(n);
    sum_kv_.resize(n);

    const std::vector<Vec3<space::Length>>& x0 = buffers_.position;
    const std::vector<Vec3<space::Velocity>>& v0 = buffers_.velocity;
    std::vector<Vec3<space::Acceleration>>& a = buffers_.acceleration;
    const space::Time half = dt * 0.5;

    // Stage 1: k_x1 = v0, k_v1 = a(x0).
    forces.accelerations(bodies, x0, a);
    for (std::size_t i = 0; i < n; ++i) {
        stage_velocity_[i] = v0[i];
        sum_kx_[i] = v0[i];
        sum_kv_[i] = a[i];
    }

    // Stages 2 to 4: x_s = x0 + k_x(prev) h_s, k_x = v0 + k_v(prev) h_s,
    // k_v = a(x_s); weights 2, 2, 1.
    const std::array<space::Time, 3> stage_step = {half, half, dt};
    const std::array<double, 3> weight = {2.0, 2.0, 1.0};
    for (std::size_t s = 0; s < 3; ++s) {
        for (std::size_t i = 0; i < n; ++i) {
            stage_position_[i] = x0[i] + stage_velocity_[i] * stage_step[s];
            stage_velocity_[i] = v0[i] + a[i] * stage_step[s];
        }
        forces.accelerations(bodies, stage_position_, a);
        for (std::size_t i = 0; i < n; ++i) {
            sum_kx_[i] += stage_velocity_[i] * weight[s];
            sum_kv_[i] += a[i] * weight[s];
        }
    }

    const space::Time sixth = dt / 6.0;
    for (std::size_t i = 0; i < n; ++i) {
        buffers_.position[i] += sum_kx_[i] * sixth;
        buffers_.velocity[i] += sum_kv_[i] * sixth;
    }
    return commit(state, buffers_, dt);
}

std::unique_ptr<IIntegrator> make_integrator(IntegratorKind kind) {
    switch (kind) {
        case IntegratorKind::leapfrog_kdk:
            return std::make_unique<LeapfrogKdk>();
        case IntegratorKind::yoshida4:
            return std::make_unique<Yoshida4>();
        case IntegratorKind::rk4:
            return std::make_unique<Rk4>();
    }
    throw std::invalid_argument("make_integrator: unknown integrator kind");
}

} // namespace orrery::core
