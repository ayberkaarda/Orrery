#pragma once

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "orrery/core/conservation.h"
#include "orrery/core/force_model.h"
#include "orrery/core/integrator.h"
#include "orrery/core/sim_snapshot.h"
#include "orrery/render/trails.h"
#include "orrery/scene/scenario.h"

// Shared fixtures of the Phase 4b scene tests: the J2000 solar system loaded
// through orrery_scene, with trail history and an energy-drift series from a
// backward integration ending at the J2000 state.
namespace orrery::render::test {

struct SolarSystemHistory {
    std::shared_ptr<const core::SimSnapshot> snapshot; // the J2000 state
    TrailHistory trails;
    std::vector<double> drift_days;   // elapsed integration time [d]
    std::vector<double> drift_values; // relative energy drift
};

inline core::SimState load_j2000_state() {
    const std::filesystem::path path =
        std::filesystem::path(ORRERY_SCENARIO_DIR) / "solar_system_j2000.json";
    auto scene = scene::load_scene_file(path);
    REQUIRE(scene.has_value());
    return scene.value().state;
}

// Trail settings of the golden images: every trail spans about 85 percent of
// its orbit.
inline TrailSettings golden_trail_settings() {
    TrailSettings settings;
    settings.length = 600;
    settings.min_step_angle = 2.0 * 3.141592653589793 * 0.85 / 600.0;
    return settings;
}

// Integrates the J2000 state backwards for `years` (leapfrog; 0.1 d steps
// over the last 400 days, so that the inner planets and the Moon are sampled
// finely, 2 d steps before that), then records the snapshots into the trail
// history in chronological order, ending with the J2000 state itself.
inline SolarSystemHistory solar_system_history(double years,
                                               TrailSettings settings = golden_trail_settings()) {
    const core::SimState initial = load_j2000_state();
    core::SimState state = initial;
    const core::NewtonianGravity gravity;
    const auto integrator = core::make_integrator(core::IntegratorKind::leapfrog_kdk);
    const core::ConservationDiagnostics diagnostics(initial, gravity);

    SolarSystemHistory history;
    std::vector<std::shared_ptr<const core::SimSnapshot>> backwards;
    const double total = years * 365.25;
    const double fine_span = 400.0;
    double elapsed = 0.0;
    std::size_t step = 0;
    while (elapsed < total) {
        const double dt = elapsed < fine_span ? 0.1 : 2.0;
        REQUIRE(integrator->step(state, core::space::Time(-dt), gravity).has_value());
        elapsed += dt;
        ++step;
        backwards.push_back(core::SimSnapshot::capture(state, step));
        if (step % 50 == 0) {
            history.drift_days.push_back(elapsed);
            history.drift_values.push_back(diagnostics.drift(state, gravity).relative_energy);
        }
    }
    history.trails = TrailHistory(settings);
    for (auto it = backwards.rbegin(); it != backwards.rend(); ++it) {
        history.trails.record(**it);
    }
    history.snapshot = core::SimSnapshot::capture(initial, 0);
    history.trails.record(*history.snapshot);
    return history;
}

} // namespace orrery::render::test
