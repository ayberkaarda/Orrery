#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "orrery/core/result.h"
#include "orrery/scene/entry_experiment.h"
#include "orrery/scene/experiment.h"
#include "orrery/sim/entry_backend.h"
#include "orrery/sim/entry_model.h"

// Experiment records of the Entry workspace (docs/entry-workspace.md,
// "Experiments"): the "entry" domain record of orrery_scene
// (scene::EntryExperiment), converted to and from the workspace's setup and
// series, and the files on disk.
namespace orrery::sim {

struct StoredEntry {
    std::filesystem::path path; // empty when not from a file
    EntrySetup setup;
    // The recorded run (series, summary and the full result record); null
    // for inputs only.
    std::shared_ptr<const EntryTrace> result;
};

// The typed record of a setup and, when it has a result record, its run.
// The periapsis form of the start is stored resolved (as the flight-path
// angle the run used), and a sampled Cd table is marked in the vehicle name.
[[nodiscard]] scene::EntryExperiment entry_record(const EntrySetup& setup,
                                                  const EntryTrace* result);
// A setup and its run back from a typed record.
[[nodiscard]] StoredEntry entry_from_record(const scene::EntryExperiment& record);

// The generic record (domain "entry") and back; the message names the
// offending key.
[[nodiscard]] scene::Experiment entry_to_experiment(const EntrySetup& setup,
                                                    const EntryTrace* result);
[[nodiscard]] core::Result<StoredEntry, std::string>
entry_from_experiment(const scene::Experiment& experiment);

struct EntryListing {
    std::vector<StoredEntry> entries; // sorted by file name
    // Entry files that could not be read ("name.json: reason"); files of
    // other domains are skipped silently.
    std::vector<std::string> problems;
};

[[nodiscard]] core::Result<StoredEntry, std::string>
load_entry_experiment_file(const std::filesystem::path& path);

// Writes next to a temporary file first and renames it into place, so a
// failed write never leaves half a file. Creates the directory when needed.
[[nodiscard]] core::Result<std::filesystem::path, std::string>
save_entry_experiment_file(const std::filesystem::path& path, const EntrySetup& setup,
                           const EntryTrace* result);

[[nodiscard]] EntryListing list_entry_experiments(const std::filesystem::path& directory);

} // namespace orrery::sim
