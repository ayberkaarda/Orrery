#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <semaphore>
#include <stop_token>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include "orrery/aero/aero_result.h"
#include "orrery/aero/regime.h"
#include "orrery/scene/aero_experiment.h"
#include "orrery/sim/aero_solve.h"
#include "orrery/sim/triple_buffer.h"

// The solver thread of the Aero workspace (docs/aero-workspace.md, "Solver
// thread"). Like the physics thread of the space workspace (ADR-011), it owns
// its solver; the user interface talks to it only through
//
//   send(AeroCommand)  UI -> solver, lock-free queue
//   wake()             UI -> solver, "a frame started" (never blocks)
//   fetch()/latest()   solver -> UI, triple buffer of immutable progress
//
// so that a long solve (a dense panel system, thousands of lattice steps)
// never stalls a frame. Two pacing modes for time-marching solvers:
//   per frame          every wake advances `steps_per_frame` lattice steps
//                      (the view animates at the frame rate)
//   to convergence     steps back to back until the solver finishes
// Direct solvers (panel, Newtonian) finish in their first step either way.
namespace orrery::sim {

// Starts a new run, replacing any current one.
struct StartAeroRun {
    AeroRunSpec spec;
    bool to_convergence = false;
};
// Pauses or resumes the current run.
struct SetAeroPaused {
    bool paused = true;
};
// Lattice steps per frame in the per-frame mode (at least 1).
struct SetAeroStepsPerFrame {
    std::int64_t steps = 100;
};
// Switches between the per-frame and the run-to-convergence mode.
struct SetAeroToConvergence {
    bool enabled = true;
};
// Advances one frame's worth of steps and pauses.
struct StepAeroFrame {};
// Drops the current run (the last published result stays readable).
struct StopAeroRun {};

using AeroCommand = std::variant<StartAeroRun, SetAeroPaused, SetAeroStepsPerFrame,
                                 SetAeroToConvergence, StepAeroFrame, StopAeroRun>;

enum class AeroRunState : std::uint8_t {
    idle,       // no run yet, or the run was stopped
    setting_up, // voxelising, assembling (a long setup is reported as such)
    running,
    paused,
    finished, // the solver reported completion
    failed,   // setup or a step threw; `message` says why
};

[[nodiscard]] std::string_view to_string(AeroRunState state) noexcept;

struct AeroRunStatus {
    AeroRunState state = AeroRunState::idle;
    // Increments with every StartAeroRun, so the UI can tell runs apart.
    std::uint64_t run = 0;
    aero::SolverKind solver = aero::SolverKind::panel;
    bool two_dimensional = false;
    bool to_convergence = false;
    std::int64_t steps_per_frame = 100;
    std::string message;
    // Wall time of the last IAeroSolver::step() call, ms.
    double step_ms = 0.0;
    // Lattice steps per second of wall time over the last call (time-marching).
    double steps_per_second = 0.0;

    friend bool operator==(const AeroRunStatus&, const AeroRunStatus&) = default;
};

// What the solver thread publishes after each change.
struct AeroProgress {
    AeroRunStatus status;
    // Result and diagnostics of the latest step (null before the first).
    std::shared_ptr<const aero::AeroResult> result;
    std::shared_ptr<const RunDiagnostics> diagnostics;
    // The spec of the run they belong to (null when idle).
    std::shared_ptr<const AeroRunSpec> spec;
    FlowSummary flow;
    aero::ReferenceValues reference;
};

class AeroRunner {
public:
    AeroRunner();
    AeroRunner(const AeroRunner&) = delete;
    AeroRunner& operator=(const AeroRunner&) = delete;
    AeroRunner(AeroRunner&&) = delete;
    AeroRunner& operator=(AeroRunner&&) = delete;
    // Stops the run at the next step boundary and joins the thread.
    ~AeroRunner();

    // Any thread.
    void send(AeroCommand command);
    // Never blocks; called once per frame by the window loop.
    void wake();

    // Consumer side (one thread): newest progress, false if nothing new.
    bool fetch() noexcept;
    [[nodiscard]] const AeroProgress& latest() const noexcept { return progress_.read(); }

    // Completed IAeroSolver::step() calls (tests and diagnostics).
    [[nodiscard]] std::uint64_t step_calls() const noexcept {
        return step_calls_.load(std::memory_order_acquire);
    }

private:
    struct Queue;
    struct Run;

    void loop(const std::stop_token& stop);
    // Applies one command; true if it changed what is published.
    bool apply(AeroCommand& command, const std::stop_token& stop);
    void start(StartAeroRun& start, const std::stop_token& stop);
    // One IAeroSolver::step() call; false once the run cannot continue.
    bool step_once();
    void publish(bool with_result);
    void fail(const std::string& what);

    std::unique_ptr<Queue> queue_;
    std::unique_ptr<Run> run_; // solver thread only
    AeroRunStatus status_;     // solver thread only
    AeroProgress published_;   // solver thread copy of the last publication
    TripleBuffer<AeroProgress> progress_;
    std::counting_semaphore<> wake_{0};
    std::atomic<std::uint64_t> step_calls_{0};
    // Frames requested by wake() since the solver last advanced one.
    std::atomic<std::uint32_t> frames_pending_{0};
    std::jthread thread_; // last: started after, joined before the others
};

} // namespace orrery::sim
