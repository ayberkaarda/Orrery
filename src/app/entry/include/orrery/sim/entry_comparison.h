#pragma once

#include <optional>
#include <span>
#include <string>
#include <vector>

#include "orrery/sim/entry_model.h"

// Comparison table of saved entry experiments (docs/entry-workspace.md,
// "Experiments") and its CSV export.
namespace orrery::sim {

// What the table needs of one experiment.
struct EntryComparisonInput {
    std::string name;
    EntrySetup setup;
    EntrySummary summary;
};

struct EntryComparisonRow {
    std::string name;
    EntryPlanet planet = EntryPlanet::earth;
    double mass_kg = 0.0;
    double ballistic_coefficient_kg_m2 = 0.0;
    double speed_m_s = 0.0;
    double flight_path_angle_deg = 0.0; // the effective one (periapsis form resolved)
    EntryOutcome outcome = EntryOutcome::none;
    double peak_g = 0.0;
    double peak_heat_flux_w_m2 = 0.0;
    double peak_dynamic_pressure_pa = 0.0;
    double heat_load_j_m2 = 0.0;
    double duration_s = 0.0;
    double downrange_m = 0.0;
};

// One row per input, sorted by planet, then by flight-path angle from the
// steepest to the shallowest (an entry corridor reads top to bottom), then
// by name.
[[nodiscard]] std::vector<EntryComparisonRow>
compare_entries(std::span<const EntryComparisonInput> inputs);

// RFC 4180 CSV of the rows (one header line, '.' decimal point, full
// precision, SI units in the headers). Text cells that a spreadsheet would
// read as a formula (a leading '=', '+', '-', '@', tab or carriage return)
// are quoted with an apostrophe in front.
[[nodiscard]] std::string entry_comparison_csv(std::span<const EntryComparisonRow> rows);

// One CSV text cell with the rules above.
[[nodiscard]] std::string csv_text_cell(const std::string& text);

} // namespace orrery::sim
