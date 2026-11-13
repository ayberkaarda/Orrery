#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <utility>

#include <imgui.h>

#include "../aero/aero_widgets.h"
#include "entry_workspace_impl.h"

// Entry Setup, Experiments and View panels of the Entry workspace.
namespace orrery::app {

namespace w = widgets;

namespace {

constexpr const char* kDegree = "\xC2\xB0";
constexpr const char* kSquareMetre = "m\xC2\xB2";
constexpr const char* kDensity = "kg/m\xC2\xB3";

// A number row in display units (SI x scale); the SI value changes when the
// edit is committed. Returns true then.
bool number_row(const char* label, const char* id, double& si_value, double scale, const char* unit,
                const char* format) {
    w::row_label_with_unit(label, unit);
    double shown = si_value * scale;
    (void)ImGui::InputDouble(id, &shown, 0.0, 0.0, format);
    const bool committed = ImGui::IsItemDeactivatedAfterEdit();
    w::unit(unit);
    if (committed && std::isfinite(shown)) {
        si_value = shown / scale;
    }
    return committed;
}

const char* atmosphere_label(sim::EntryPlanet planet, sim::EntryAtmosphere atmosphere) {
    if (atmosphere == sim::EntryAtmosphere::exponential) {
        return "Exponential";
    }
    return planet == sim::EntryPlanet::mars ? "Mars reference table" : "US Standard 1976";
}

const char* outcome_short(sim::EntryOutcome outcome) {
    const std::string_view label = sim::outcome_label(outcome);
    return label.empty() ? "-" : label.data();
}

} // namespace

// --- Entry Setup -----------------------------------------------------------------

void EntryWorkspace::Impl::draw_setup_panel() {
    if (!ImGui::Begin(std::string(kEntrySetupWindow).c_str())) {
        ImGui::End();
        return;
    }
    w::row_label("Preset");
    if (ImGui::BeginCombo("##preset", "Load a preset...")) {
        for (const sim::EntryPreset& preset : sim::entry_presets()) {
            if (ImGui::Selectable(preset.label)) {
                replace_setup(preset.setup);
                cd_label.clear();
                cd_warnings.clear();
            }
        }
        ImGui::EndCombo();
    }
    draw_planet_inputs();
    draw_probe_inputs();
    draw_cd_inputs();
    draw_initial_state();
    draw_run_controls();
    ImGui::End();
}

void EntryWorkspace::Impl::draw_planet_inputs() {
    w::section("PLANET");
    w::row_label("Planet");
    const char* planet = setup.planet == sim::EntryPlanet::mars ? "Mars" : "Earth";
    if (ImGui::BeginCombo("##planet", planet)) {
        for (const sim::EntryPlanet option : {sim::EntryPlanet::earth, sim::EntryPlanet::mars}) {
            const char* label = option == sim::EntryPlanet::mars ? "Mars" : "Earth";
            if (ImGui::Selectable(label, option == setup.planet) && option != setup.planet) {
                setup.planet = option;
                setup.altitude_m = sim::planet_constants(option).atmosphere_top_m;
            }
        }
        ImGui::EndCombo();
    }
    w::row_label("Atmosphere");
    if (ImGui::BeginCombo("##atmosphere", atmosphere_label(setup.planet, setup.atmosphere))) {
        for (const sim::EntryAtmosphere option :
             {sim::EntryAtmosphere::standard, sim::EntryAtmosphere::exponential}) {
            if (ImGui::Selectable(atmosphere_label(setup.planet, option),
                                  option == setup.atmosphere)) {
                setup.atmosphere = option;
            }
        }
        ImGui::EndCombo();
    }
    if (setup.atmosphere == sim::EntryAtmosphere::exponential) {
        (void)number_row("Surface density", "##rho0", setup.exponential_density_kg_m3, 1.0,
                         kDensity, "%.4g");
        (void)number_row("Scale height", "##scale_height", setup.exponential_scale_height_m, 1.0e-3,
                         "km", "%.2f");
        (void)number_row("Temperature", "##temperature", setup.exponential_temperature_k, 1.0, "K",
                         "%.1f");
    }
    (void)ImGui::Checkbox("Air turns with the planet", &setup.co_rotating_atmosphere);
    w::tooltip("Off: the atmosphere rests in the inertial frame. On: it co-rotates, and the "
               "drag acts on the speed relative to the turning air.");
}

void EntryWorkspace::Impl::draw_probe_inputs() {
    w::section("PROBE");
    (void)number_row("Mass", "##mass", setup.mass_kg, 1.0, "kg", "%.1f");
    (void)number_row("Ref. area", "##area", setup.reference_area_m2, 1.0, kSquareMetre, "%.3f");
    (void)number_row("Nose radius", "##nose", setup.nose_radius_m, 1.0, "m", "%.3f");
    const double beta = sim::ballistic_coefficient(setup);
    const ImVec4 muted = ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled);
    if (beta > 0.0) {
        w::wrapped("Ballistic coefficient " + w::fixed(beta, 0) + " kg/m\xC2\xB2 (largest Cd)",
                   muted);
    }
    w::wrapped("Constant mass, no lift, heat shield forward.", muted);
}

void EntryWorkspace::Impl::draw_cd_inputs() {
    w::section("DRAG  Cd(MACH)");
    if (setup.cd_source == sim::CdSource::aero_solver) {
        w::chip("AERO SOLVER", w::theme().accent_secondary,
                "Sampled with modified Newtonian theory on the body below");
        if (!cd_label.empty()) {
            ImGui::SameLine();
            w::muted(cd_label);
        }
    } else {
        w::chip("TABLE", w::theme().text_muted, "Typed-in Cd(Mach) points");
    }
    draw_cd_table();
    w::row_label("Body");
    if (ImGui::BeginCombo("##cd_body", std::string(sim::cd_body_name(cd_body)).c_str())) {
        for (const sim::CdBody body :
             {sim::CdBody::apollo_capsule, sim::CdBody::orion_capsule, sim::CdBody::sphere}) {
            if (ImGui::Selectable(std::string(sim::cd_body_name(body)).c_str(), body == cd_body)) {
                cd_body = body;
            }
        }
        ImGui::EndCombo();
    }
    ImGui::BeginDisabled(cd_task.started() || !(setup.reference_area_m2 > 0.0));
    if (ImGui::Button("Sample from aero solver", ImVec2(-1.0F, 0.0F))) {
        start_cd_sampling();
    }
    ImGui::EndDisabled();
    w::tooltip("Runs the Newtonian solver at Mach 2, 5, 10, 15 and 25 on this body, sized so "
               "that its frontal area is the reference area.");
    for (const std::string& warning : cd_warnings) {
        w::warning(warning);
    }
}

void EntryWorkspace::Impl::draw_cd_table() {
    constexpr ImGuiTableFlags kFlags =
        ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp;
    std::optional<std::size_t> remove;
    bool edited = false;
    if (ImGui::BeginTable("##cd_table", 3, kFlags)) {
        ImGui::TableSetupColumn("Mach", ImGuiTableColumnFlags_WidthStretch, 1.0F);
        ImGui::TableSetupColumn("Cd", ImGuiTableColumnFlags_WidthStretch, 1.0F);
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
        ImGui::TableHeadersRow();
        for (std::size_t i = 0; i < setup.cd_table.size(); ++i) {
            ImGui::PushID(static_cast<int>(i));
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::SetNextItemWidth(-1.0F);
            (void)ImGui::InputDouble("##mach", &setup.cd_table[i].mach, 0.0, 0.0, "%.2f");
            edited = ImGui::IsItemDeactivatedAfterEdit() || edited;
            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(-1.0F);
            (void)ImGui::InputDouble("##cd", &setup.cd_table[i].cd, 0.0, 0.0, "%.3f");
            edited = ImGui::IsItemDeactivatedAfterEdit() || edited;
            ImGui::TableSetColumnIndex(2);
            if (ImGui::SmallButton("x")) {
                remove = i;
            }
            w::tooltip("Remove this point");
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    if (remove && *remove < setup.cd_table.size()) {
        setup.cd_table.erase(setup.cd_table.begin() + static_cast<std::ptrdiff_t>(*remove));
        edited = true;
    }
    if (ImGui::SmallButton("+ Point")) {
        const double mach = setup.cd_table.empty() ? 1.0 : setup.cd_table.back().mach + 5.0;
        const double cd = setup.cd_table.empty() ? 1.0 : setup.cd_table.back().cd;
        setup.cd_table.push_back({mach, cd});
        edited = true;
    }
    w::tooltip("Points are interpolated linearly in Mach and held constant beyond the ends.");
    if (edited) {
        setup.cd_source = sim::CdSource::table;
        cd_label.clear();
        cd_warnings.clear();
    }
}

void EntryWorkspace::Impl::draw_initial_state() {
    w::section("INITIAL STATE");
    int mode = setup.start == sim::EntryStart::periapsis ? 1 : 0;
    if (ImGui::RadioButton("Path angle", &mode, 0)) {
        setup.start = sim::EntryStart::state;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Periapsis", &mode, 1)) {
        setup.start = sim::EntryStart::periapsis;
    }
    (void)number_row("Altitude", "##altitude", setup.altitude_m, 1.0e-3, "km", "%.2f");
    (void)number_row("Speed", "##speed", setup.speed_m_s, 1.0e-3, "km/s", "%.3f");
    if (setup.start == sim::EntryStart::state) {
        (void)number_row("Path angle", "##gamma", setup.flight_path_angle_deg, 1.0, kDegree,
                         "%.2f");
        if (const auto periapsis = sim::periapsis_altitude(
                setup.planet, setup.altitude_m, setup.speed_m_s, setup.flight_path_angle_deg)) {
            w::muted("Vacuum periapsis " + w::fixed(*periapsis * 1.0e-3, 1) + " km");
        }
    } else {
        (void)number_row("Periapsis", "##periapsis", setup.periapsis_altitude_m, 1.0e-3, "km",
                         "%.1f");
        if (const auto gamma = sim::effective_flight_path_angle(setup)) {
            w::muted("Flight-path angle " + w::fixed(*gamma, 2) + kDegree + " at the start");
        }
    }
    (void)number_row("Latitude", "##lat", setup.latitude_deg, 1.0, kDegree, "%.2f");
    (void)number_row("Longitude", "##lon", setup.longitude_deg, 1.0, kDegree, "%.2f");
    (void)number_row("Heading", "##heading", setup.heading_deg, 1.0, kDegree, "%.1f");
    (void)number_row("Time limit", "##limit", setup.max_time_s, 1.0, "s", "%.0f");
    draw_integration_inputs();
}

void EntryWorkspace::Impl::draw_integration_inputs() {
    w::section("INTEGRATION");
    constexpr std::array<std::pair<sim::EntryIntegrator, const char*>, 3> kIntegrators{{
        {sim::EntryIntegrator::ias15, "IAS15 (adaptive)"},
        {sim::EntryIntegrator::yoshida4, "Yoshida 4th order"},
        {sim::EntryIntegrator::leapfrog_kdk, "Leapfrog KDK"},
    }};
    sim::EntryIntegration& i = setup.integration;
    const char* current = kIntegrators.front().second;
    for (const auto& [integrator, label] : kIntegrators) {
        if (integrator == i.integrator) {
            current = label;
        }
    }
    w::row_label("Integrator");
    if (ImGui::BeginCombo("##integrator", current)) {
        for (const auto& [integrator, label] : kIntegrators) {
            if (ImGui::Selectable(label, integrator == i.integrator)) {
                i.integrator = integrator;
            }
        }
        ImGui::EndCombo();
    }
    w::tooltip("Gravity part of the split step; the drag acts as half kicks around it.");
    (void)number_row("Step", "##step", i.time_step_s, 1.0, "s", "%.3f");
    if (i.integrator == sim::EntryIntegrator::ias15) {
        (void)number_row("Tolerance", "##tolerance", i.tolerance, 1.0, "", "%.1e");
    }
    (void)ImGui::Checkbox("End at skip-out", &i.stop_on_skip_out);
}

void EntryWorkspace::Impl::draw_run_controls() {
    const std::vector<std::string> errors = sim::setup_errors(setup);
    ImGui::Dummy(ImVec2(0.0F, 4.0F));
    for (const std::string& error : errors) {
        w::error(error);
    }
    ImGui::BeginDisabled(!errors.empty() || active());
    ImGui::PushStyleColor(ImGuiCol_Button, w::color(w::theme().accent, 0.85F));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, w::color(w::theme().accent_hover));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, w::color(w::theme().accent_active));
    ImGui::PushStyleColor(ImGuiCol_Text, w::color(w::theme().title));
    if (ImGui::Button("Run entry", ImVec2(-1.0F, ImGui::GetFrameHeight() * 1.2F))) {
        (void)start_run();
    }
    ImGui::PopStyleColor(4);
    ImGui::EndDisabled();
    if (run_setup && !same_entry_inputs(*run_setup, setup) && active()) {
        w::muted("The inputs changed since this run started; it continues with the old ones.");
    }
}

// --- Experiments -----------------------------------------------------------------

void EntryWorkspace::Impl::draw_experiments_panel() {
    if (!ImGui::Begin(std::string(kEntryExperimentsWindow).c_str())) {
        ImGui::End();
        return;
    }
    if (listing_dirty) {
        refresh_listing();
    }
    w::row_label("Name");
    (void)ImGui::InputTextWithHint("##name", "experiment name", name_buffer.data(),
                                   name_buffer.size());
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        setup.name = w::from_buffer(name_buffer);
    }
    if (ImGui::Button("Save")) {
        (void)save();
    }
    w::tooltip("Writes the inputs and, when the shown run used exactly these inputs, its "
               "series and summary to the experiments folder.");
    ImGui::SameLine();
    if (ImGui::Button("Refresh")) {
        listing_dirty = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Export CSV")) {
        (void)export_comparison();
    }
    w::tooltip("Writes the comparison table (the ticked runs, or all) as CSV next to the "
               "experiments.");
    ImGui::SameLine();
    w::muted(options.experiment_dir.string());
    if (!notice.empty()) {
        if (notice_is_error) {
            w::error(notice);
        } else {
            w::wrapped(notice, w::color(w::theme().accent_secondary));
        }
    }
    for (const std::string& problem : listing.problems) {
        w::warning(problem);
    }
    draw_saved_table();
    draw_comparison_table();
    ImGui::End();
}

void EntryWorkspace::Impl::draw_saved_table() {
    w::section("SAVED");
    if (listing.entries.empty()) {
        w::muted("No entry experiments saved yet.");
        return;
    }
    constexpr ImGuiTableFlags kFlags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH |
                                       ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_ScrollY;
    const float height = std::min(ImGui::GetTextLineHeightWithSpacing() *
                                      static_cast<float>(listing.entries.size() + 2),
                                  ImGui::GetFontSize() * 10.0F);
    std::optional<std::size_t> load;
    if (ImGui::BeginTable("##saved", 8, kFlags, ImVec2(0.0F, height))) {
        ImGui::TableSetupScrollFreeze(0, 1);
        for (const char* header :
             {"", "Name", "Planet", "Speed", "Angle", "Outcome", "Peak g", ""}) {
            ImGui::TableSetupColumn(header);
        }
        ImGui::TableHeadersRow();
        for (std::size_t i = 0; i < listing.entries.size(); ++i) {
            const sim::StoredEntry& e = listing.entries[i];
            ImGui::PushID(static_cast<int>(i));
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            bool selected = i < compared.size() && compared[i] != 0;
            if (ImGui::Checkbox("##compare", &selected) && i < compared.size()) {
                compared[i] = selected ? 1 : 0;
            }
            w::tooltip("Compare, and overlay in the Entry Profile plots");
            ImGui::TableSetColumnIndex(1);
            w::text(e.setup.name.empty() ? e.path.stem().string() : e.setup.name);
            ImGui::TableSetColumnIndex(2);
            w::muted(std::string(sim::planet_constants(e.setup.planet).name));
            ImGui::TableSetColumnIndex(3);
            w::text(w::fixed(e.setup.speed_m_s * 1.0e-3, 2) + " km/s");
            ImGui::TableSetColumnIndex(4);
            const auto gamma = sim::effective_flight_path_angle(e.setup);
            w::text(gamma ? w::fixed(*gamma, 2) + kDegree : std::string("-"));
            ImGui::TableSetColumnIndex(5);
            w::text(e.result ? outcome_short(e.result->summary.outcome) : "-");
            ImGui::TableSetColumnIndex(6);
            w::text(e.result ? w::fixed(e.result->summary.peak_g, 2) : "-");
            ImGui::TableSetColumnIndex(7);
            if (ImGui::SmallButton("Load")) {
                load = i;
            }
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    if (load && *load < listing.entries.size()) {
        const sim::StoredEntry chosen = listing.entries[*load];
        open(chosen);
        set_notice("Loaded " + chosen.path.filename().string(), false);
    }
}

void EntryWorkspace::Impl::draw_comparison_table() {
    w::section("COMPARISON");
    const std::vector<sim::EntryComparisonRow> rows = sim::compare_entries(comparison_inputs());
    if (rows.empty()) {
        w::muted("Save runs (for example one vehicle at several flight-path angles) to compare "
                 "them.");
        return;
    }
    constexpr ImGuiTableFlags kFlags =
        ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchSame;
    if (!ImGui::BeginTable("##comparison", 8, kFlags)) {
        return;
    }
    for (const char* header : {"Name", "Angle", "Outcome", "Peak g", "Peak heat flux", "Peak q",
                               "Heat load", "Downrange"}) {
        ImGui::TableSetupColumn(header);
    }
    ImGui::TableHeadersRow();
    for (const sim::EntryComparisonRow& row : rows) {
        ImGui::TableNextRow();
        const std::array<std::string, 8> cells{
            row.name,
            w::fixed(row.flight_path_angle_deg, 2) + kDegree,
            outcome_short(row.outcome),
            w::fixed(row.peak_g, 2) + " g",
            w::fixed(row.peak_heat_flux_w_m2 * 1.0e-3, 0) + " kW/m\xC2\xB2",
            w::fixed(row.peak_dynamic_pressure_pa * 1.0e-3, 1) + " kPa",
            w::fixed(row.heat_load_j_m2 * 1.0e-6, 1) + " MJ/m\xC2\xB2",
            w::fixed(row.downrange_m * 1.0e-3, 0) + " km"};
        for (std::size_t c = 0; c < cells.size(); ++c) {
            ImGui::TableSetColumnIndex(static_cast<int>(c));
            w::text(cells[c]);
        }
    }
    ImGui::EndTable();
}

// --- View ------------------------------------------------------------------------

void EntryWorkspace::Impl::draw_view_panel() {
    if (!ImGui::Begin(std::string(kEntryViewWindow).c_str())) {
        ImGui::End();
        return;
    }
    w::section("RIBBON");
    w::row_label("Colour by");
    const char* coloring =
        layers.coloring == EntryColoring::heat_flux ? "Stagnation heat flux" : "g-load";
    if (ImGui::BeginCombo("##coloring", coloring)) {
        if (ImGui::Selectable("g-load", layers.coloring == EntryColoring::g_load)) {
            layers.coloring = EntryColoring::g_load;
        }
        if (ImGui::Selectable("Stagnation heat flux",
                              layers.coloring == EntryColoring::heat_flux)) {
            layers.coloring = EntryColoring::heat_flux;
        }
        ImGui::EndCombo();
    }
    w::row_label("Altitude scale");
    const double lo = 1.0;
    const double hi = 20.0;
    (void)ImGui::SliderScalar("##scale", ImGuiDataType_Double, &layers.altitude_scale, &lo, &hi,
                              "x%.0f");
    w::tooltip("Altitudes are drawn exaggerated by this factor so the trajectory separates "
               "from the ground; the planet keeps its true size.");
    w::section("LAYERS");
    (void)ImGui::Checkbox("Atmosphere shell", &layers.atmosphere);
    ImGui::SameLine();
    (void)ImGui::Checkbox("Graticule", &layers.graticule);
    (void)ImGui::Checkbox("Ground track", &layers.ground_track);
    ImGui::SameLine();
    (void)ImGui::Checkbox("Altitude curtain", &layers.curtain);
    if (!view_problem.empty()) {
        w::error("The scene could not show everything: " + view_problem);
    }
    w::section("CAMERA");
    if (ImGui::Button("Frame the trajectory (F)")) {
        reframe_pending = true;
    }
    w::muted("Drag to orbit, wheel to zoom.");
    ImGui::End();
}

} // namespace orrery::app
