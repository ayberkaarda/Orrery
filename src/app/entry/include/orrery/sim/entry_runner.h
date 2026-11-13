#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <semaphore>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>
#include <variant>

#include "orrery/sim/entry_backend.h"
#include "orrery/sim/entry_model.h"
#include "orrery/sim/triple_buffer.h"

// The solver thread of the Entry workspace (docs/entry-workspace.md, "Solver
// thread"), shaped like the aero runner (ADR-011): it owns the stepper; the
// user interface talks to it only through
//
//   send(EntryCommand)  UI -> solver, lock-free queue
//   wake()              UI -> solver, "a frame started" (never blocks)
//   fetch()/latest()    solver -> UI, triple buffer of immutable progress
//
// The run advances in slices of at most kEntrySliceSeconds of simulated time
// and polls its commands between slices, so Pause and Stop act within one
// slice. Pacing: `pace` simulated seconds per wall-clock second, or as fast
// as possible (pace 0).
namespace orrery::sim {

struct StartEntryRun {
    EntrySetup setup;
};
struct SetEntryPaused {
    bool paused = true;
};
// Simulated seconds per wall-clock second; 0 runs as fast as possible.
struct SetEntryPace {
    double pace = 0.0;
};
// Ends the run; its trace so far stays readable (state `stopped`).
struct StopEntryRun {};

using EntryCommand = std::variant<StartEntryRun, SetEntryPaused, SetEntryPace, StopEntryRun>;

enum class EntryRunState : std::uint8_t {
    idle, // no run yet
    running,
    paused,
    finished, // the entry ended (landed, skip-out, time limit)
    stopped,  // stopped by the user before the end
    failed,   // the stepper could not be built or threw; `message` says why
};

[[nodiscard]] std::string_view to_string(EntryRunState state) noexcept;

struct EntryRunStatus {
    EntryRunState state = EntryRunState::idle;
    // Increments with every StartEntryRun.
    std::uint64_t run = 0;
    double pace = 0.0;
    double sim_time_s = 0.0;
    std::string message;

    friend bool operator==(const EntryRunStatus&, const EntryRunStatus&) = default;
};

// An immutable copy of the run so far.
struct EntryTrace {
    EntrySeries series;
    EntrySummary summary;
};

struct EntryProgress {
    EntryRunStatus status;
    std::shared_ptr<const EntrySetup> setup; // of the run (null before any)
    std::shared_ptr<const EntryTrace> trace; // null before the first slice
    // Increments whenever `trace` is replaced.
    std::uint64_t revision = 0;
};

// Simulated time advanced per stepper call.
inline constexpr double kEntrySliceSeconds = 2.0;
// While running, progress is published at most this often [s] (and on every
// state change).
inline constexpr double kEntryPublishSeconds = 1.0 / 30.0;

class EntryRunner {
public:
    explicit EntryRunner(EntryStepperFactory factory = make_entry_stepper);
    EntryRunner(const EntryRunner&) = delete;
    EntryRunner& operator=(const EntryRunner&) = delete;
    EntryRunner(EntryRunner&&) = delete;
    EntryRunner& operator=(EntryRunner&&) = delete;
    // Stops the run at the next slice boundary and joins the thread.
    ~EntryRunner();

    // Any thread.
    void send(EntryCommand command);
    // Never blocks; called once per frame by the window loop.
    void wake();

    // Consumer side (one thread): newest progress, false if nothing new.
    bool fetch() noexcept;
    [[nodiscard]] const EntryProgress& latest() const noexcept { return progress_.read(); }

    // Completed stepper calls (tests).
    [[nodiscard]] std::uint64_t slices() const noexcept {
        return slices_.load(std::memory_order_acquire);
    }

private:
    struct Queue;
    struct Run;
    using Clock = std::chrono::steady_clock;

    void loop(const std::stop_token& stop);
    // Applies one command; true if it changed what is published.
    bool apply(EntryCommand& command);
    void start(StartEntryRun& start);
    // Advances the run by its pace since the last call (or as far as the
    // wall-clock batch allows in the fastest pace); stops early for control
    // commands.
    void advance(const std::stop_token& stop);
    // One stepper call; false once the run cannot continue.
    bool slice(double seconds);
    void publish(bool with_trace);
    void fail(const std::string& what);

    EntryStepperFactory factory_;
    std::unique_ptr<Queue> queue_;
    std::unique_ptr<Run> run_; // solver thread only
    EntryRunStatus status_;    // solver thread only
    EntryProgress published_;  // solver thread copy of the last publication
    TripleBuffer<EntryProgress> progress_;
    std::counting_semaphore<> wake_{0};
    std::atomic<std::uint64_t> slices_{0};
    // Control commands sent and not yet taken from the queue.
    std::atomic<std::uint32_t> controls_pending_{0};
    Clock::time_point last_advance_{};
    Clock::time_point last_publish_{};
    std::jthread thread_; // last: started after, joined before the others
};

} // namespace orrery::sim
