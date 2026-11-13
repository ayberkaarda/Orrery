#pragma once

#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "orrery/core/result.h"
#include "orrery/scene/experiment.h"
#include "orrery/scene/scenario.h"
#include "orrery/sim/simulation.h"

// Space experiments (docs/app.md, "Experiments"): an orrery/scene experiment
// record with domain "space" names a scenario file and the settings of the
// simulation that runs it.
//
//   {
//     "version": 1,
//     "domain": "space",
//     "inputs": { "scenario": "../scenarios/solar_system_j2000.json" },
//     "conditions": { "time_warp": 1000000 },
//     "solver": { "name": "leapfrog_kdk", "version": "1",
//                 "settings": { "dt_days": 0.05, "force": "auto" } },
//     "result": {}
//   }
//
// inputs.scenario      required; relative paths are resolved against the
//                      directory of the experiment file
// conditions.time_warp optional number in [1, 1e7], default 1e6
// conditions.paused    optional number, 0 or 1, default 0
// solver.name          integrator: leapfrog_kdk, yoshida4, rk4 or ias15
// solver.settings      optional: dt_days (default 0.05), force (auto,
//                      newtonian, barnes_hut, paczynski_wiita), softening_au,
//                      max_steps_per_frame (default 4096)
// Any other key in these maps is rejected, as are aero and entry domains
// (they open in their own workspaces). The result map is free-form.
namespace orrery::sim {

struct SpaceExperiment {
    std::filesystem::path experiment_path;
    std::filesystem::path scenario_path;
    scene::Scene scene;
    SimulationConfig config;
};

// Reads, validates and loads an experiment and its scenario. Errors name the
// offending file and value.
[[nodiscard]] core::Result<SpaceExperiment, std::string>
load_space_experiment(const std::filesystem::path& path);

// Locates an experiment given on the command line: an existing file path is
// used as is; otherwise `<dir>/<name>.json` (or `<dir>/<name>` when the name
// already ends in .json) is tried in each search directory in order. The
// error lists every location that was tried.
[[nodiscard]] core::Result<std::filesystem::path, std::string>
resolve_experiment(std::string_view name_or_path,
                   std::span<const std::filesystem::path> search_dirs);

struct ScenarioEntry {
    std::string name; // file stem, e.g. "solar_system_j2000"
    std::filesystem::path path;
};

// The *.json files of a directory, sorted by name (empty when the directory
// does not exist or cannot be read).
[[nodiscard]] std::vector<ScenarioEntry> list_scenarios(const std::filesystem::path& directory);

// Phase 10: saving and listing space experiments (docs/app.md, "Experiment
// panel").
//
// The record of the running simulation: `scenario` (the file the system came
// from) is written relative to `directory`, where the record will be saved,
// when both share a root, else absolute; conditions and solver from `status`
// (the requested warp, not the governed one); result: sim_time_days, steps,
// events and, when known, energy_drift. Loading the record again gives the
// same settings (tested).
[[nodiscard]] scene::Experiment space_record(const std::filesystem::path& scenario,
                                             const SimStatus& status, double sim_time_days,
                                             const std::filesystem::path& directory);

// Writes `record` to `path` through a temporary file that is renamed into
// place (creating the directory); the path or the reason.
[[nodiscard]] core::Result<std::filesystem::path, std::string>
save_space_experiment_file(const std::filesystem::path& path, const scene::Experiment& record);

struct StoredSpace {
    std::filesystem::path path;
    std::string name;    // file stem
    std::string summary; // "solar_system_j2000, leapfrog_kdk, 1e6 s/s"
};

struct SpaceListing {
    std::vector<StoredSpace> entries; // sorted by file name
    // Space experiments that cannot be loaded, as "<file>: <reason>"; files
    // of other domains are skipped silently.
    std::vector<std::string> problems;
};

// The space experiments of a directory (empty when it does not exist).
[[nodiscard]] SpaceListing list_space_experiments(const std::filesystem::path& directory);

} // namespace orrery::sim
