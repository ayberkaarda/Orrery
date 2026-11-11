#pragma once

#include <array>
#include <cstdint>
#include <string_view>
#include <vector>

#include "orrery/core/force_model.h"
#include "orrery/core/integrator.h"
#include "orrery/core/quantity.h"
#include "orrery/core/result.h"
#include "orrery/core/sim_state.h"
#include "orrery/core/vec3.h"

namespace orrery::core {

enum class Ias15Error : std::uint8_t {
    // The tolerance is NaN, infinite, zero or negative.
    invalid_tolerance,
};

[[nodiscard]] std::string_view to_string(Ias15Error error) noexcept;

// IAS15: the 15th-order adaptive integrator of Rein and Spiegel (2015, MNRAS
// 446, 1424), an implicit Gauss-Radau scheme after Everhart (1985).
//
// Within one internal step of length h the acceleration is approximated by
// the degree-7 polynomial a(t0 + s h) = a0 + b0 s + b1 s^2 + ... + b6 s^7,
// s in [0, 1], fitted at the 8 Gauss-Radau spacings s = h0 (0), h1, ..., h7.
// The coefficients are found by predictor-corrector iteration: positions are
// predicted at each spacing from the current b, accelerations are evaluated
// there, and the divided differences g_k are converted to b. Iteration stops
// when the change of b6 relative to the largest acceleration falls below
// 1e-16, stops decreasing, or after 12 iterations. The step is then
//   x1 = x0 + h v0 + h^2 (a0/2 + b0/6 + b1/12 + b2/20 + b3/30 + b4/42
//                          + b5/56 + b6/72)
//   v1 = v0 + h (a0 + b0/2 + b1/3 + b2/4 + b3/5 + b4/6 + b5/7 + b6/8),
// accumulated with compensated (Kahan) summation.
//
// Step-size control (Rein and Spiegel 2015, section 3.4): with
// err = max|b6| / max|a| over all coordinates, the next step is
// h' = h (epsilon / err)^(1/7). If h' / h < 0.25 the step is rejected and
// redone with h'; otherwise h' is limited to 4 h. The b of the next step are
// predicted from the converged b of the last step, re-expanded for the new
// step length, plus the last prediction's correction.
//
// Adaptive entry through the fixed interface: step(state, dt, forces)
// integrates exactly to t + dt with as many internal steps as the controller
// needs, the last one shortened to land on t + dt. The controller's step and
// the b prediction carry over to the next call when that call starts from
// the state this integrator committed (bit-identical positions and
// velocities); any other state, a change of body count or a change of
// direction restarts them. The sequence of calls therefore determines the
// result, which is reproducible bit for bit: only +, -, *, /, sqrt, frexp
// and ldexp are used (the seventh root is a fixed Newton iteration, not pow).
//
// Forces are functions of positions only (IForceModel), so no velocity
// predictor is needed.
//
// APPROXIMATION: truncation of the acceleration to a degree-7 polynomial per
// step - the local error is O(h^16) and the step control keeps it near
// machine precision for epsilon = 1e-9 (Rein and Spiegel 2015) - valid while
// the forces are smooth over a step; discontinuous forces (impacts, switching
// force models) must be handled at the step boundary by the caller.
class IAS15 final : public IIntegrator {
public:
    // Rein and Spiegel (2015) recommended precision parameter.
    static constexpr double default_tolerance = 1.0e-9;
    // Step rejection and growth limits of the controller.
    static constexpr double safety_factor = 0.25;

    IAS15() = default;

    [[nodiscard]] static Result<IAS15, Ias15Error> create(double tolerance);

    [[nodiscard]] std::string_view name() const noexcept override { return "ias15"; }
    [[nodiscard]] int order() const noexcept override { return 15; }
    [[nodiscard]] bool is_symplectic() const noexcept override { return false; }
    // Lower bound per internal step (one start evaluation plus one pass over
    // the seven substeps); the measured total is force_evaluations().
    [[nodiscard]] int force_evaluations_per_step() const noexcept override { return 8; }

    [[nodiscard]] Result<space::Time, IntegrationError> step(SimState& state, space::Time dt,
                                                             const IForceModel& forces) override;

    // Reports every accepted internal step to `observer` before anything is
    // committed (IIntegrator::step_observed). Without a stop the arithmetic
    // is that of step(). A stop commits the state at the end of the stopping
    // internal step; the controller's step proposal and b prediction carry
    // over to the next call as after any committed call.
    [[nodiscard]] Result<space::Time, IntegrationError>
    step_observed(SimState& state, space::Time dt, const IForceModel& forces,
                  IStepObserver& observer) override;

    [[nodiscard]] double tolerance() const noexcept { return tolerance_; }

    // Cumulative counters since construction.
    [[nodiscard]] long long force_evaluations() const noexcept { return force_evaluations_; }
    [[nodiscard]] long long accepted_steps() const noexcept { return accepted_steps_; }
    [[nodiscard]] long long rejected_steps() const noexcept { return rejected_steps_; }

    // Step size the controller proposes for the next internal step (0 before
    // the first step and after a restart).
    [[nodiscard]] space::Time suggested_step() const noexcept { return space::Time(next_step_); }

    // Forgets the controller state and the b prediction.
    void restart() noexcept;

private:
    using Coefficients = std::array<std::vector<double>, 7>;

    struct Attempt {
        bool ok = false;       // false: non-finite acceleration
        bool accepted = false; // false: rejected, retry with next_step
        double next_step = 0.0;
    };

    explicit IAS15(double tolerance) noexcept : tolerance_(tolerance) {}

    void resize(std::size_t coordinates);
    bool matches_committed(const BodyStore& bodies) const;
    void predict_coefficients(double step);
    bool evaluate(const BodyStore& bodies, const IForceModel& forces, std::vector<double>& out);
    Attempt attempt(const BodyStore& bodies, const IForceModel& forces, double step);
    Result<space::Time, IntegrationError>
    advance(SimState& state, space::Time dt, const IForceModel& forces, IStepObserver* observer);

    double tolerance_ = default_tolerance;

    // Flat coordinates, 3 per body (x, y, z).
    std::vector<double> x0_;
    std::vector<double> v0_;
    std::vector<double> a0_;
    std::vector<double> at_;
    // Compensation terms of the Kahan sums of x0_ and v0_.
    std::vector<double> comp_x_;
    std::vector<double> comp_v_;
    Coefficients b_;
    Coefficients g_;
    Coefficients e_;
    // Converged b and its starting prediction e of the last accepted step.
    Coefficients b_last_;
    Coefficients e_last_;
    double last_step_ = 0.0;
    double next_step_ = 0.0;

    // State committed by the last successful call, to detect a foreign state.
    std::vector<Vec3<space::Length>> committed_position_;
    std::vector<Vec3<space::Velocity>> committed_velocity_;
    bool has_committed_ = false;

    // Force-model interface buffers.
    std::vector<Vec3<space::Length>> trial_position_;
    std::vector<Vec3<space::Acceleration>> trial_acceleration_;
    // Segment ends reported to an observer.
    std::vector<Vec3<space::Length>> segment_start_;
    std::vector<Vec3<space::Length>> segment_end_;

    long long force_evaluations_ = 0;
    long long accepted_steps_ = 0;
    long long rejected_steps_ = 0;
};

} // namespace orrery::core
