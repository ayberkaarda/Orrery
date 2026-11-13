#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "orrery/atmo/entry.h"
#include "orrery/core/result.h"
#include "orrery/scene/entry_experiment.h"
#include "orrery/sim/entry_model.h"

// The Entry workspace's only door to the entry physics (orrery_atmo's
// EntryExperiment on the core integrator, docs/entry-workspace.md "Solver"):
// an incremental stepper built from an EntrySetup, the conversion of setups
// and runs to the "entry" experiment record, and Cd(Mach) sampling. The
// solver thread, the headless renderer and the tests talk to this interface.
namespace orrery::sim {

// A run as the workspace shows it: the series and summary in the units of
// the panels, plus the full result record of the physics library when the
// run ended (null while it is in progress), which is what Save writes.
struct EntryTrace {
    EntrySeries series;
    EntrySummary summary;
    std::shared_ptr<const scene::EntryResultRecord> record;
};

class EntryStepper {
public:
    EntryStepper() = default;
    EntryStepper(const EntryStepper&) = delete;
    EntryStepper& operator=(const EntryStepper&) = delete;
    EntryStepper(EntryStepper&&) = delete;
    EntryStepper& operator=(EntryStepper&&) = delete;
    virtual ~EntryStepper() = default;

    // Integrates at most `seconds` of simulated time (a short slice, so that
    // the caller can poll its commands between calls) and appends the
    // samples recorded in it to `out` (the first call also records the
    // initial state). Returns true once the entry has ended; outcome() says
    // how. Throws std::runtime_error when the integration fails.
    virtual bool advance(double seconds, EntrySeries& out) = 0;
    [[nodiscard]] virtual EntryOutcome outcome() const noexcept = 0;
    // Simulated time reached [s].
    [[nodiscard]] virtual double time_s() const noexcept = 0;
    // The result record of the run so far (null when the stepper has none).
    [[nodiscard]] virtual std::shared_ptr<const scene::EntryResultRecord> record() const {
        return nullptr;
    }
};

using EntryStepperFactory =
    std::function<core::Result<std::unique_ptr<EntryStepper>, std::string>(const EntrySetup&)>;

// The physics configuration of a setup (planet preset or exponential
// atmosphere, vehicle with its Cd(Mach) table, initial state, time limit);
// the reason when it cannot be built.
[[nodiscard]] core::Result<atmo::EntryConfig, std::string> entry_config(const EntrySetup& setup);

// The stepper of the physics library for `setup`; the reason when it cannot
// be built (setup_errors() first, then the library's own checks).
[[nodiscard]] core::Result<std::unique_ptr<EntryStepper>, std::string>
make_entry_stepper(const EntrySetup& setup);

// The setup of a physics configuration (the presets of entry_presets()).
[[nodiscard]] EntrySetup setup_from_config(const atmo::EntryConfig& config, std::string name);

// Runs `setup` to its end on the calling thread (headless, tests).
using EntryRunResult = EntryTrace;
[[nodiscard]] core::Result<EntryRunResult, std::string> run_entry(const EntrySetup& setup);

// The library's series in the workspace's units, with the downrange
// distance (great circle over the ground track from its first point).
[[nodiscard]] EntrySeries series_from(const atmo::EntrySeries& series, double planet_radius_m);
[[nodiscard]] EntryOutcome outcome_from(atmo::EntryOutcome outcome) noexcept;
[[nodiscard]] atmo::EntryIntegrator to_atmo(EntryIntegrator integrator) noexcept;
[[nodiscard]] EntryIntegrator from_atmo(atmo::EntryIntegrator integrator) noexcept;

// --- Cd(Mach) from the aero solver ------------------------------------------

// Spec 6: Cd(Mach) "sampled from the aero solver at about five Mach
// numbers" (atmo::sample_cd_table, Newtonian theory, Mach 5 to 25 where it is
// valid) on a primitive body whose frontal area is the entry's reference
// area, so the coefficients need no renormalisation. Outside the sampled
// range the table holds its end values.
enum class CdBody : std::uint8_t { apollo_capsule, orion_capsule, sphere };
struct CdSampling {
    CdBody body = CdBody::apollo_capsule;
    double reference_area_m2 = 12.02;
};
struct CdSamples {
    std::vector<MachCd> table;
    std::string label; // the body ("Apollo-type capsule, d 3.91 m")
    std::vector<std::string> warnings;
};
[[nodiscard]] std::string_view cd_body_name(CdBody body) noexcept;
// Runs on the calling thread (a background task in the workspace).
[[nodiscard]] core::Result<CdSamples, std::string> sample_cd_table(const CdSampling& sampling);

} // namespace orrery::sim
