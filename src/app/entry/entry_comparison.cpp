#include "orrery/sim/entry_comparison.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <string_view>
#include <system_error>

namespace orrery::sim {

namespace {

// Shortest round-trip decimal text of a double ("nan" and "inf" for the
// non-finite values, which a record should not hold).
std::string exact(double value) {
    std::array<char, 64> buffer{};
    const auto written = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
    return written.ec == std::errc{} ? std::string(buffer.data(), written.ptr) : std::string("0");
}

// True for text a spreadsheet would read as a formula (OWASP "CSV
// injection"): a leading '=', '+', '-', '@', tab or carriage return.
bool formula_like(const std::string& text) noexcept {
    return !text.empty() && std::string_view("=+-@\t\r").find(text.front()) != std::string::npos;
}

void append_number(std::string& out, double value) {
    out += ',';
    out += exact(value);
}

} // namespace

std::string csv_text_cell(const std::string& text) {
    const bool formula = formula_like(text);
    if (!formula && text.find_first_of(",\"\r\n") == std::string::npos) {
        return text;
    }
    std::string quoted = formula ? "\"'" : "\"";
    for (const char c : text) {
        quoted += c;
        if (c == '"') {
            quoted += '"';
        }
    }
    quoted += '"';
    return quoted;
}

std::vector<EntryComparisonRow> compare_entries(std::span<const EntryComparisonInput> inputs) {
    std::vector<EntryComparisonRow> rows;
    rows.reserve(inputs.size());
    for (const EntryComparisonInput& input : inputs) {
        EntryComparisonRow row;
        row.name = input.name;
        row.planet = input.setup.planet;
        row.mass_kg = input.setup.mass_kg;
        row.ballistic_coefficient_kg_m2 = ballistic_coefficient(input.setup);
        row.speed_m_s = input.setup.speed_m_s;
        row.flight_path_angle_deg = effective_flight_path_angle(input.setup).value_or(0.0);
        row.outcome = input.summary.outcome;
        row.peak_g = input.summary.peak_g;
        row.peak_heat_flux_w_m2 = input.summary.peak_heat_flux_w_m2;
        row.peak_dynamic_pressure_pa = input.summary.peak_dynamic_pressure_pa;
        row.heat_load_j_m2 = input.summary.heat_load_j_m2;
        row.duration_s = input.summary.duration_s;
        row.downrange_m = input.summary.downrange_m;
        rows.push_back(std::move(row));
    }
    std::stable_sort(rows.begin(), rows.end(),
                     [](const EntryComparisonRow& a, const EntryComparisonRow& b) {
                         if (a.planet != b.planet) {
                             return a.planet < b.planet;
                         }
                         if (a.flight_path_angle_deg != b.flight_path_angle_deg) {
                             return a.flight_path_angle_deg < b.flight_path_angle_deg;
                         }
                         return a.name < b.name;
                     });
    return rows;
}

std::string entry_comparison_csv(std::span<const EntryComparisonRow> rows) {
    std::string out = "name,planet,mass_kg,ballistic_coefficient_kg_m2,speed_m_s,"
                      "flight_path_angle_deg,outcome,peak_g,peak_heat_flux_w_m2,"
                      "peak_dynamic_pressure_pa,heat_load_j_m2,duration_s,downrange_m\r\n";
    for (const EntryComparisonRow& row : rows) {
        out += csv_text_cell(row.name);
        out += ',';
        out += to_string(row.planet);
        append_number(out, row.mass_kg);
        append_number(out, row.ballistic_coefficient_kg_m2);
        append_number(out, row.speed_m_s);
        append_number(out, row.flight_path_angle_deg);
        out += ',';
        out += to_string(row.outcome);
        append_number(out, row.peak_g);
        append_number(out, row.peak_heat_flux_w_m2);
        append_number(out, row.peak_dynamic_pressure_pa);
        append_number(out, row.heat_load_j_m2);
        append_number(out, row.duration_s);
        append_number(out, row.downrange_m);
        out += "\r\n";
    }
    return out;
}

} // namespace orrery::sim
