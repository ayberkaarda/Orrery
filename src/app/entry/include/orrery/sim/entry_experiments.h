#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "orrery/core/result.h"
#include "orrery/scene/experiment.h"
#include "orrery/sim/entry_model.h"
#include "orrery/sim/entry_runner.h"

// Experiment records of the Entry workspace (docs/entry-workspace.md,
// "Experiments"): domain "entry" of the experiment schema, on disk.
namespace orrery::sim {

struct StoredEntry {
    std::filesystem::path path; // empty when not from a file
    EntrySetup setup;
    // The recorded run (series and summary); null for inputs only.
    std::shared_ptr<const EntryTrace> result;
};

struct EntryListing {
    std::vector<StoredEntry> entries; // sorted by file name
    // Entry files that could not be read ("name.json: reason"); files of
    // other domains are skipped silently.
    std::vector<std::string> problems;
};

// The record of a setup and, when given, its run.
[[nodiscard]] scene::Experiment entry_to_experiment(const EntrySetup& setup,
                                                    const EntryTrace* result);
// Reads an entry record; the message names the offending key.
[[nodiscard]] core::Result<StoredEntry, std::string>
entry_from_experiment(const scene::Experiment& experiment);

[[nodiscard]] core::Result<StoredEntry, std::string>
load_entry_experiment_file(const std::filesystem::path& path);

// Writes next to a temporary file first and renames it into place, so a
// failed write never leaves half a file. Creates the directory when needed.
[[nodiscard]] core::Result<std::filesystem::path, std::string>
save_entry_experiment_file(const std::filesystem::path& path, const EntrySetup& setup,
                           const EntryTrace* result);

[[nodiscard]] EntryListing list_entry_experiments(const std::filesystem::path& directory);

} // namespace orrery::sim
