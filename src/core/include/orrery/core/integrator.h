#pragma once

#include <cstdint>
#include <memory>
#include <string_view>
#include <vector>

#include "orrery/core/force_model.h"
#include "orrery/core/quantity.h"
#include "orrery/core/result.h"
#include "orrery/core/sim_state.h"
#include "orrery/core/vec3.h"

namespace orrery::core {

enum class IntegrationError : std::uint8_t {
    // The time step is NaN or infinite.
    invalid_time_step,
    // A position or velocity became NaN or infinite during the step (for
    // example coincident sources without softening). The state is unchanged.
    non_finite_state,
};

[[nodiscard]] std::string_view to_string(IntegrationError error) noexcept;

// Fixed-step integrator of the equations of motion r'' = a(r) in space units.
//
// Why a symplectic method is the default (ADR-002): the gravitational N-body
// problem is Hamiltonian. A symplectic integrator at fixed step exactly
// conserves a "shadow" Hamiltonian that differs from the true one by
// O(dt^p), so the energy error stays bounded and oscillates for
// exponentially long times instead of growing. Non-symplectic schemes such as
// RK4 have a smaller local error per step but their energy error accumulates
// secularly (linearly in time), which dominates long runs. Leapfrog KDK is
// therefore the default; Yoshida4 is the higher-order symplectic option; RK4
// is kept for tests and comparisons only.
//
// Contract of step():
//   - dt must be finite (negative steps integrate backwards; Leapfrog and
//     Yoshida4 are time-reversible, so a step of -dt undoes a step of dt up to
//     rounding).
//   - On success the positions, velocities and time of the state are advanced
//     by dt and the new time is returned.
//   - On failure the state is left unchanged (strong guarantee).
//   - Deterministic: the same state, dt and force model give a bit-identical
//     result. Operations are performed in a fixed order on doubles only.
//   - Not thread-safe: an integrator owns scratch buffers reused across steps.
//
// APPROXIMATION: fixed-step time discretisation - a constant dt keeps the
// symplectic schemes symplectic and the results reproducible - the global
// error scales as dt^order and is controlled by the shortest dynamical time
// (pericentre passage); valid when dt is a small fraction of it (measured
// errors in docs/physics.md). Close encounters need adaptive steps, which are
// outside this interface.
class IIntegrator {
public:
    virtual ~IIntegrator() = default;

    [[nodiscard]] virtual std::string_view name() const noexcept = 0;
    // Order of the global error in dt.
    [[nodiscard]] virtual int order() const noexcept = 0;
    [[nodiscard]] virtual bool is_symplectic() const noexcept = 0;
    // Force evaluations per step.
    [[nodiscard]] virtual int force_evaluations_per_step() const noexcept = 0;

    [[nodiscard]] virtual Result<space::Time, IntegrationError>
    step(SimState& state, space::Time dt, const IForceModel& forces) = 0;

protected:
    IIntegrator() = default;
    IIntegrator(const IIntegrator&) = default;
    IIntegrator(IIntegrator&&) = default;
    IIntegrator& operator=(const IIntegrator&) = default;
    IIntegrator& operator=(IIntegrator&&) = default;
};

namespace detail {

// Working copy of the dynamical state used by the integrators.
struct PhaseBuffers {
    std::vector<Vec3<space::Length>> position;
    std::vector<Vec3<space::Velocity>> velocity;
    std::vector<Vec3<space::Acceleration>> acceleration;
};

} // namespace detail

// Leapfrog, kick-drift-kick (velocity Verlet):
//   v += a(x) dt/2;  x += v dt;  v += a(x) dt/2.
// Second order, symplectic, time-reversible. Two force evaluations per step
// (accelerations are not cached between steps, so a step depends only on the
// state passed in). Default integrator.
class LeapfrogKdk final : public IIntegrator {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "leapfrog_kdk"; }
    [[nodiscard]] int order() const noexcept override { return 2; }
    [[nodiscard]] bool is_symplectic() const noexcept override { return true; }
    [[nodiscard]] int force_evaluations_per_step() const noexcept override { return 2; }

    [[nodiscard]] Result<space::Time, IntegrationError> step(SimState& state, space::Time dt,
                                                             const IForceModel& forces) override;

private:
    detail::PhaseBuffers buffers_;
};

// Yoshida (1990) fourth-order symplectic integrator: the triple-jump
// composition of three Leapfrog KDK substeps with weights (w1, w0, w1),
//   w1 = 1 / (2 - 2^(1/3)),  w0 = -2^(1/3) / (2 - 2^(1/3)),  2 w1 + w0 = 1.
// Adjacent half-kicks of consecutive substeps are merged, giving four kicks
// and three drifts:
//   kick w1/2, drift w1, kick (w1 + w0)/2, drift w0, kick (w0 + w1)/2,
//   drift w1, kick w1/2   (all multiplied by dt).
// Fourth order, symplectic, time-reversible. Four force evaluations per step.
class Yoshida4 final : public IIntegrator {
public:
    // 2^(1/3), correctly rounded to double (written as a literal so that the
    // coefficients do not depend on the platform's cbrt).
    static constexpr double cube_root_two = 1.2599210498948731647672106;
    static constexpr double w1 = 1.0 / (2.0 - cube_root_two);
    static constexpr double w0 = -cube_root_two / (2.0 - cube_root_two);

    [[nodiscard]] std::string_view name() const noexcept override { return "yoshida4"; }
    [[nodiscard]] int order() const noexcept override { return 4; }
    [[nodiscard]] bool is_symplectic() const noexcept override { return true; }
    [[nodiscard]] int force_evaluations_per_step() const noexcept override { return 4; }

    [[nodiscard]] Result<space::Time, IntegrationError> step(SimState& state, space::Time dt,
                                                             const IForceModel& forces) override;

private:
    detail::PhaseBuffers buffers_;
};

// Classical fourth-order Runge-Kutta on (x, v). Not symplectic: its energy
// error grows secularly. For tests and comparisons only (ADR-002); never the
// production integrator for long runs.
class Rk4 final : public IIntegrator {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "rk4"; }
    [[nodiscard]] int order() const noexcept override { return 4; }
    [[nodiscard]] bool is_symplectic() const noexcept override { return false; }
    [[nodiscard]] int force_evaluations_per_step() const noexcept override { return 4; }

    [[nodiscard]] Result<space::Time, IntegrationError> step(SimState& state, space::Time dt,
                                                             const IForceModel& forces) override;

private:
    detail::PhaseBuffers buffers_;
    // Stage position and velocity, and the weighted sums of the stage slopes
    // (k_x = velocity, k_v = acceleration).
    std::vector<Vec3<space::Length>> stage_position_;
    std::vector<Vec3<space::Velocity>> stage_velocity_;
    std::vector<Vec3<space::Velocity>> sum_kx_;
    std::vector<Vec3<space::Acceleration>> sum_kv_;
};

enum class IntegratorKind : std::uint8_t {
    leapfrog_kdk,
    yoshida4,
    rk4,
};

// The production default (ADR-002).
inline constexpr IntegratorKind default_integrator = IntegratorKind::leapfrog_kdk;

[[nodiscard]] std::unique_ptr<IIntegrator> make_integrator(IntegratorKind kind);

} // namespace orrery::core
