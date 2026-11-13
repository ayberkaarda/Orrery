#pragma once

#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "orrery/aero/regime.h"
#include "orrery/core/result.h"
#include "orrery/scene/aero_experiment.h"

// Experiment records of the Aero workspace (docs/aero-workspace.md,
// "Experiments"): files on disk, and the comparison table of runs of the
// same geometry at different angles of attack.
namespace orrery::sim {

// --- Comparison --------------------------------------------------------------

struct ComparisonRow {
    std::string name;
    double angle_of_attack_deg = 0.0;
    double sideslip_deg = 0.0;
    double speed_m_s = 0.0;
    aero::SolverKind solver = aero::SolverKind::panel;
    double cd = 0.0;
    double cl = 0.0;
    double cm = 0.0;
    // Differences to the first row (the lowest angle of attack).
    double delta_cd = 0.0;
    double delta_cl = 0.0;
    double delta_cm = 0.0;
    double lift_to_drag = 0.0; // 0 when cd is 0
    bool converged = false;
};

struct ComparisonTable {
    // The shared geometry: its reference (and a label for the heading).
    scene::GeometryRef geometry;
    std::string label;
    // Rows sorted by angle of attack (then name).
    std::vector<ComparisonRow> rows;
    // Least-squares slope of cl over the angle of attack, per degree and
    // per radian, when at least two rows have different angles.
    std::optional<double> lift_slope_per_deg;
    std::optional<double> lift_slope_per_rad;
};

// Short description of a geometry reference ("NACA 0012 section, chord 1 m",
// "glider.glb").
[[nodiscard]] std::string geometry_label(const scene::GeometryRef& ref);

// One table per distinct geometry among the experiments that have a result,
// in the order the geometries first appear. Experiments without a result
// are left out.
[[nodiscard]] std::vector<ComparisonTable>
compare_by_geometry(std::span<const scene::AeroExperiment> experiments);

// The table as CSV (RFC 4180: comma separated, quoted where needed, one
// header line, '.' decimal point, full precision). Text cells that a
// spreadsheet would read as a formula (a leading '=', '+', '-', '@', tab or
// carriage return) are quoted with an apostrophe in front, so names from
// untrusted files are never evaluated.
[[nodiscard]] std::string comparison_csv(const ComparisonTable& table);

// --- Files -----------------------------------------------------------------

struct StoredExperiment {
    std::filesystem::path path;
    scene::AeroExperiment experiment;
};

struct ExperimentListing {
    std::vector<StoredExperiment> experiments; // sorted by file name
    // Aero files that could not be read ("name.json: reason"); files of
    // other domains are skipped silently.
    std::vector<std::string> problems;
};

[[nodiscard]] core::Result<scene::AeroExperiment, std::string>
load_aero_experiment_file(const std::filesystem::path& path);

// Writes the experiment as JSON next to a temporary file first and renames
// it into place, so a failed write never leaves half a file. Creates the
// directory when needed.
[[nodiscard]] core::Result<std::filesystem::path, std::string>
save_aero_experiment_file(const std::filesystem::path& path,
                          const scene::AeroExperiment& experiment);

[[nodiscard]] ExperimentListing list_aero_experiments(const std::filesystem::path& directory);

// A file name for a new experiment in `directory`: the name reduced to
// lower-case letters, digits and underscores ("naca_0012_4_deg.json"), with
// "_2", "_3", ... appended when the file exists.
[[nodiscard]] std::filesystem::path new_experiment_path(const std::filesystem::path& directory,
                                                        const std::string& name);

} // namespace orrery::sim
