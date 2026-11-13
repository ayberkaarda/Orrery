#pragma once

#include <functional>
#include <memory>
#include <string>

#include "orrery/core/result.h"
#include "orrery/sim/entry_model.h"

// The Entry workspace's only door to the entry physics (orrery_atmo and the
// core integrator, docs/entry-workspace.md "Solver"): an incremental stepper
// built from an EntrySetup. The solver thread, the headless renderer and the
// tests talk to this interface; nothing else in the workspace knows the
// physics API.
namespace orrery::sim {

class EntryStepper {
public:
    EntryStepper() = default;
    EntryStepper(const EntryStepper&) = delete;
    EntryStepper& operator=(const EntryStepper&) = delete;
    EntryStepper(EntryStepper&&) = delete;
    EntryStepper& operator=(EntryStepper&&) = delete;
    virtual ~EntryStepper() = default;

    // Integrates at most `seconds` of simulated time (a short slice, so that
    // the caller can poll its commands between calls) and appends the samples
    // recorded in it to `out` (the first call also records the initial
    // state). Returns true once the entry has ended; outcome() says how.
    // Throws std::runtime_error when the integration fails.
    virtual bool advance(double seconds, EntrySeries& out) = 0;
    [[nodiscard]] virtual EntryOutcome outcome() const noexcept = 0;
    // Simulated time reached [s].
    [[nodiscard]] virtual double time_s() const noexcept = 0;
};

using EntryStepperFactory =
    std::function<core::Result<std::unique_ptr<EntryStepper>, std::string>(const EntrySetup&)>;

// The stepper of the physics library for `setup` (setup_errors() must be
// empty); the reason when it cannot be built.
[[nodiscard]] core::Result<std::unique_ptr<EntryStepper>, std::string>
make_entry_stepper(const EntrySetup& setup);

// Samples are recorded at this interval of simulated time, and at the end.
inline constexpr double kEntrySampleInterval = 0.5;

// Runs `setup` to its end on the calling thread (headless, tests); the
// series and the outcome, or the reason it failed.
struct EntryRunResult {
    EntrySeries series;
    EntrySummary summary;
};
[[nodiscard]] core::Result<EntryRunResult, std::string> run_entry(const EntrySetup& setup);

} // namespace orrery::sim
