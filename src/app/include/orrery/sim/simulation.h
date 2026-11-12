#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "orrery/core/result.h"
#include "orrery/core/sim_snapshot.h"
#include "orrery/core/sim_state.h"
#include "orrery/scene/scenario.h"
#include "orrery/sim/sim_command.h"

// Deterministic space simulation driven by wall-clock time (docs/app.md,
// "Fixed-timestep accumulator"). Single-threaded: PhysicsThread owns one on
// its own thread; the headless renderer and the tests drive one directly.
namespace orrery::sim {

enum class SpaceIntegrator : std::uint8_t {
    leapfrog_kdk,
    yoshida4,
    rk4,
    ias15,
};

enum class ForceChoice : std::uint8_t {
    // Paczynski-Wiita when the system holds a black hole, Barnes-Hut above
    // kDirectSummationLimit bodies, Newtonian direct summation otherwise.
    automatic,
    newtonian,
    barnes_hut,
    paczynski_wiita,
};

// Largest body count integrated with direct summation by ForceChoice::automatic
// (the documented contract of core::NewtonianGravity).
inline constexpr std::size_t kDirectSummationLimit = 256;

// Names as used in experiment files ("leapfrog_kdk", "yoshida4", "rk4",
// "ias15"; "auto", "newtonian", "barnes_hut", "paczynski_wiita").
[[nodiscard]] std::string_view to_string(SpaceIntegrator integrator) noexcept;
[[nodiscard]] std::optional<SpaceIntegrator> integrator_from_string(std::string_view name) noexcept;
[[nodiscard]] std::string_view to_string(ForceChoice force) noexcept;
[[nodiscard]] std::optional<ForceChoice> force_from_string(std::string_view name) noexcept;

struct SimulationConfig {
    SpaceIntegrator integrator = SpaceIntegrator::leapfrog_kdk;
    ForceChoice force = ForceChoice::automatic;
    // Plummer softening of the force model [AU], finite and >= 0.
    double softening_au = 0.0;
    // Fixed physics step [day], in [kMinStepDays, kMaxStepDays].
    double dt_days = 0.05;
    // Simulated seconds per wall-clock second, in [kMinTimeWarp, kMaxTimeWarp].
    std::int64_t time_warp = 1'000'000;
    bool paused = false;
    // Step budget of one advance() call, >= 1: simulated time that would need
    // more steps is dropped and the status reports the warp as limited.
    std::int64_t max_steps_per_advance = 4096;
    // Optional wall-clock budget of one advance() call [s]; 0 disables it.
    // A positive budget makes the number of steps depend on the machine, so
    // deterministic runs (headless, tests) leave it at 0.
    double step_time_budget_s = 0.0;
};

inline constexpr double kMinStepDays = 1.0e-9;
inline constexpr double kMaxStepDays = 1.0e4;
// Wall-clock time credited by one advance() is clamped to this many
// nanoseconds, so a stalled caller (window dragged, debugger) cannot ask for
// an unbounded catch-up.
inline constexpr std::int64_t kMaxAdvanceWallNs = 250'000'000;

// Empty when the configuration is usable, else the reason.
[[nodiscard]] std::optional<std::string> validate(const SimulationConfig& config);

// Observable state of the simulation besides the bodies themselves.
struct SimStatus {
    std::string scenario;    // name of the loaded scenario
    std::string integrator;  // integrator name
    std::string force_model; // force model name
    double dt_days = 0.0;
    bool paused = false;
    std::int64_t time_warp = kMinTimeWarp; // requested
    // Simulated seconds per wall-clock second actually achieved by the last
    // advance() (0 while paused).
    double effective_warp = 0.0;
    // The step budget dropped simulated time in the last advance(): the
    // requested warp cannot be sustained.
    bool warp_limited = false;
    std::uint64_t steps = 0;      // physics steps since the scenario was loaded or reset
    std::uint64_t generation = 0; // incremented by every load and reset
    // Mean wall-clock time of one physics step in the last advance() that
    // stepped [ms].
    std::optional<double> step_ms;
    // Relative energy drift since the scenario was loaded or reset (absent
    // above kDriftBodyLimit bodies, where the O(N^2) potential is too costly
    // to evaluate at the publication rate).
    std::optional<double> energy_drift;
    std::uint64_t events = 0; // collisions, disruptions, horizon crossings so far
    // Last error or notice (empty: none). Errors pause the simulation.
    std::string message;

    friend bool operator==(const SimStatus&, const SimStatus&) = default;
};

inline constexpr std::size_t kDriftBodyLimit = 2048;

// Outcome of one advance() call.
struct AdvanceResult {
    std::int64_t steps = 0;
    bool warp_limited = false;
    bool failed = false; // an integration step failed; the simulation paused
};

class Simulation {
public:
    // Fails when the configuration is invalid (see validate) or the force
    // model cannot be built.
    [[nodiscard]] static core::Result<Simulation, std::string>
    create(scene::Scene scene, const SimulationConfig& config);

    Simulation(Simulation&&) noexcept;
    Simulation& operator=(Simulation&&) noexcept;
    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;
    ~Simulation();

    // Applies one command (LoadScenario reads its file here).
    void apply(const SimCommand& command);

    // Credits `wall_ns` nanoseconds of wall-clock time (clamped to
    // [0, kMaxAdvanceWallNs]) at the current warp and takes every whole
    // physics step now owed, within the step budget.
    //
    // The accumulator is integral: owed simulated time is kept in
    // nanoseconds, the credit is wall_ns * time_warp exactly, and a step
    // consumes round(dt_days * 86400e9) nanoseconds. The number of steps
    // taken after a sequence of calls therefore depends only on the sum of
    // the credited wall time (while the budget is not reached and no command
    // intervenes), not on how it was split into calls, and the integrator
    // always steps by the same dt_days. Paused: nothing is owed or taken.
    AdvanceResult advance(std::int64_t wall_ns);

    // Captures the current state (sequence = status().steps).
    [[nodiscard]] std::shared_ptr<const core::SimSnapshot> snapshot() const;

    // Refreshes the derived status fields (energy drift) and returns it.
    [[nodiscard]] const SimStatus& status();
    [[nodiscard]] const core::SimState& state() const noexcept;
    [[nodiscard]] const SimulationConfig& config() const noexcept;

private:
    struct Impl;
    explicit Simulation(std::unique_ptr<Impl> impl) noexcept;
    std::unique_ptr<Impl> impl_;
};

} // namespace orrery::sim
