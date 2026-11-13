#include "orrery/render/ui_panels.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>
#include <imgui.h>
#include <imgui_internal.h>
#include <implot.h>

#include "orrery/core/kepler.h"
#include "orrery/core/units.h"
#include "orrery/render/body_visuals.h"
#include "orrery/render/ui_app_bar.h"
#include "orrery/render/ui_theme.h"

namespace orrery::render {

namespace {

constexpr double kRadToDeg = 57.29577951308232;

// Number formatting without printf format strings.
std::string fixed(double value, int precision) {
    std::array<char, 64> buffer{};
    const auto result = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value,
                                      std::chars_format::fixed, precision);
    return result.ec == std::errc{} ? std::string(buffer.data(), result.ptr) : std::string("-");
}

std::string scientific(double value, int precision) {
    std::array<char, 64> buffer{};
    const auto result = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value,
                                      std::chars_format::scientific, precision);
    return result.ec == std::errc{} ? std::string(buffer.data(), result.ptr) : std::string("-");
}

// Fixed notation for ordinary magnitudes, scientific outside [1e-3, 1e7).
std::string readable(double value, int precision) {
    const double magnitude = std::abs(value);
    if (value != 0.0 && (magnitude < 1.0e-3 || magnitude >= 1.0e7)) {
        return scientific(value, precision);
    }
    return fixed(value, precision);
}

std::string kind_label(core::BodyKindTag kind) {
    switch (kind) {
        case core::BodyKindTag::star:
            return "Star";
        case core::BodyKindTag::planet:
            return "Planet";
        case core::BodyKindTag::moon:
            return "Moon";
        case core::BodyKindTag::asteroid:
            return "Asteroid";
        case core::BodyKindTag::black_hole:
            return "Black hole";
        case core::BodyKindTag::probe:
            return "Probe";
    }
    return "Body";
}

ImVec4 kind_color(core::BodyKindTag kind) {
    switch (kind) {
        case core::BodyKindTag::star:
            return {0.95F, 0.74F, 0.42F, 1.0F};
        case core::BodyKindTag::planet:
            return {0.55F, 0.72F, 0.86F, 1.0F};
        case core::BodyKindTag::moon:
            return {0.70F, 0.72F, 0.76F, 1.0F};
        case core::BodyKindTag::asteroid:
            return {0.66F, 0.58F, 0.50F, 1.0F};
        case core::BodyKindTag::black_hole:
            return {0.68F, 0.56F, 0.90F, 1.0F};
        case core::BodyKindTag::probe:
            return {0.45F, 0.86F, 0.90F, 1.0F};
    }
    return {1.0F, 1.0F, 1.0F, 1.0F};
}

void text(const std::string& value) {
    ImGui::TextUnformatted(value.c_str());
}

void muted(const std::string& value) {
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextUnformatted(value.c_str());
    ImGui::PopStyleColor();
}

// Small upper-case section caption with a hairline under it.
void section(const char* caption) {
    ImGui::Dummy(ImVec2(0.0F, 2.0F));
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::SeparatorText(caption);
    ImGui::PopStyleColor();
}

// Two-column property table rows: muted label in a column just wide enough
// for the longest label, value in the rest (long values keep their room in a
// narrow panel).
bool begin_properties(const char* id) {
    if (!ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingFixedFit)) {
        return false;
    }
    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::CalcTextSize("Period").x + ImGui::GetFontSize() * 1.5F);
    ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);
    return true;
}

void property(const char* label, const std::string& value) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    muted(label);
    ImGui::TableSetColumnIndex(1);
    text(value);
}

// Offset of the control column of the settings rows: room for the longest
// label, capped so that a narrow panel keeps most of its width for controls.
float control_column() {
    const float longest = ImGui::CalcTextSize("Brightness").x + ImGui::GetStyle().ItemSpacing.x;
    const float cap = ImGui::GetStyle().WindowPadding.x + ImGui::GetContentRegionAvail().x * 0.4F;
    return std::max(std::min(ImGui::GetFontSize() * 7.0F, cap),
                    ImGui::GetStyle().WindowPadding.x + longest);
}

// Label on the left, control filling the rest of the row.
void row_label(const char* label) {
    ImGui::AlignTextToFramePadding();
    muted(label);
    ImGui::SameLine(control_column());
    ImGui::SetNextItemWidth(-1.0F);
}

// Unit and tick decimals of a time axis, chosen from the plotted span in
// days; the plotted values are days / days_per_unit, so that the ticks fall
// on round values of the unit.
struct TimeAxis {
    double days_per_unit = 1.0;
    int decimals = 0;
    const char* unit = " d";
};

TimeAxis time_axis_for(double span_days) {
    constexpr double kDaysPerYear = 365.25;
    if (span_days >= 5.0 * kDaysPerYear) {
        return {kDaysPerYear, span_days >= 50.0 * kDaysPerYear ? 0 : 1, " yr"};
    }
    if (span_days >= 20.0) {
        return {1.0, 0, " d"};
    }
    return {1.0, span_days >= 2.0 ? 1 : 2, " d"};
}

// ImPlotFormatter: `user_data` is the TimeAxis of the plot.
int format_time_tick(double value, char* buffer, int size, void* user_data) {
    const auto* axis = static_cast<const TimeAxis*>(user_data);
    const std::string text = fixed(value, axis->decimals) + axis->unit;
    if (size <= 0) {
        return 0;
    }
    const std::size_t count = std::min(text.size(), static_cast<std::size_t>(size - 1));
    std::copy_n(text.data(), count, buffer);
    buffer[count] = '\0';
    return static_cast<int>(count);
}

core::BodyKindTag tag_at(const core::SimSnapshot& snapshot, std::size_t i) {
    return snapshot.kind_tag(i);
}

glm::dvec3 position_au(const core::SimSnapshot& s, std::size_t i) {
    const auto& p = s.positions()[i];
    return {p.x.value(), p.y.value(), p.z.value()};
}

// Primary of body i: the least massive heavier body whose sphere of
// influence contains it, else the heaviest body.
std::optional<std::size_t> primary_of(const core::SimSnapshot& s, std::size_t i) {
    std::size_t heaviest = 0;
    for (std::size_t j = 1; j < s.size(); ++j) {
        if (s.masses()[j].value() > s.masses()[heaviest].value()) {
            heaviest = j;
        }
    }
    if (heaviest == i || s.size() < 2) {
        return std::nullopt;
    }
    const double m_heaviest = s.masses()[heaviest].value();
    const glm::dvec3 centre = position_au(s, heaviest);
    std::optional<std::size_t> best;
    for (std::size_t j = 0; j < s.size(); ++j) {
        const double mj = s.masses()[j].value();
        if (j == i || j == heaviest || !(mj > s.masses()[i].value()) ||
            !core::exerts_gravity(s.kinds()[j])) {
            continue;
        }
        const double r_soi =
            glm::length(position_au(s, j) - centre) * std::pow(mj / m_heaviest, 0.4);
        if (glm::length(position_au(s, i) - position_au(s, j)) < r_soi &&
            (!best || mj < s.masses()[*best].value())) {
            best = j;
        }
    }
    return best ? best : std::optional<std::size_t>(heaviest);
}

// A stat tile of the System Monitor: caption above, value below.
struct MonitorTile {
    const char* caption = nullptr;
    std::string value;
    bool warning = false; // value in the warning colour
};

std::vector<MonitorTile> monitor_tiles(const SystemMonitorData& data) {
    std::vector<MonitorTile> tiles;
    tiles.reserve(6);
    tiles.push_back({"SIM TIME", std::abs(data.sim_time_days) < 1000.0
                                     ? fixed(data.sim_time_days, 2) + " d"
                                     : fixed(data.sim_time_days / 365.25, 2) + " yr"});
    if (data.warp) {
        const std::string requested = format_time_warp(static_cast<double>(data.warp->requested));
        tiles.push_back({"TIME WARP",
                         data.warp->limited
                             ? format_time_warp(data.warp->achieved) + " / " + requested
                             : requested,
                         data.warp->limited});
    }
    tiles.push_back({"BODIES", std::to_string(data.body_count)});
    tiles.push_back({"FRAME", fixed(data.frame_time_ms, 2) + " ms"});
    tiles.push_back(
        {"PHYSICS STEP", data.physics_step_ms ? fixed(*data.physics_step_ms, 3) + " ms" : "-"});
    tiles.push_back({"INTEGRATOR", data.integrator.empty() ? std::string("-") : data.integrator});
    return tiles;
}

ImVec4 warning_colour() {
    const UiColor& c = default_theme().warning;
    return {c.r, c.g, c.b, 1.0F};
}

void draw_monitor_tiles(const std::vector<MonitorTile>& tiles) {
    if (!ImGui::BeginTable("##tiles", static_cast<int>(tiles.size()),
                           ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_BordersInnerV)) {
        return;
    }
    const ImGuiStyle& style = ImGui::GetStyle();
    ImGui::TableNextRow();
    for (std::size_t i = 0; i < tiles.size(); ++i) {
        ImGui::TableSetColumnIndex(static_cast<int>(i));
        ImGui::PushFont(nullptr, style.FontSizeBase * 0.8F);
        muted(tiles[i].caption);
        ImGui::PopFont();
        ImGui::PushFont(nullptr, style.FontSizeBase * 1.2F);
        ImGui::PushStyleColor(ImGuiCol_Text, tiles[i].warning
                                                 ? warning_colour()
                                                 : ImGui::GetStyleColorVec4(ImGuiCol_Text));
        text(tiles[i].value);
        ImGui::PopStyleColor();
        ImGui::PopFont();
    }
    ImGui::EndTable();
}

// Why the warp is lower than asked, in one line (nothing while it is not).
void draw_warp_reason(const WarpReadout& warp) {
    if (!warp.limited) {
        return;
    }
    std::string reason = "Warp lowered to fit the physics budget";
    if (warp.load && warp.load_limit > 0.0) {
        reason += " (load ";
        reason += fixed(*warp.load * 100.0, 0);
        reason += " % of ";
        reason += fixed(warp.load_limit * 100.0, 0);
        reason += " %)";
    }
    reason += "; ";
    reason += format_time_warp(static_cast<double>(warp.requested));
    reason += " returns when the load allows.";
    ImGui::PushStyleColor(ImGuiCol_Text, warning_colour());
    ImGui::PushTextWrapPos(0.0F);
    ImGui::TextUnformatted(reason.c_str());
    ImGui::PopTextWrapPos();
    ImGui::PopStyleColor();
}

} // namespace

BodyInspectorData make_body_inspector_data(const core::SimSnapshot& snapshot,
                                           std::optional<core::BodyId> selected) {
    BodyInspectorData data;
    data.bodies.reserve(snapshot.size());
    for (std::size_t i = 0; i < snapshot.size(); ++i) {
        data.bodies.push_back({snapshot.ids()[i], snapshot.names()[i], tag_at(snapshot, i)});
    }
    if (!selected) {
        return data;
    }
    const std::optional<std::size_t> found = snapshot.index_of(*selected);
    if (!found) {
        return data;
    }
    const std::size_t i = *found;
    const double au_m = core::constants::astronomical_unit.value();
    const double au_per_day_to_km_s = au_m / core::constants::day.value() / 1000.0;

    BodyDetails d;
    d.id = snapshot.ids()[i];
    d.name = snapshot.names()[i];
    d.kind = tag_at(snapshot, i);
    d.mass_solar = snapshot.masses()[i].value();
    d.mass_kg = core::to_si(snapshot.masses()[i]).value();
    const BodyAppearance appearance =
        appearance_for(snapshot.names()[i], snapshot.kinds()[i], snapshot.masses()[i]);
    d.radius_is_nominal = !(snapshot.radii()[i].value() > 0.0);
    d.radius_km = physical_radius(snapshot.radii()[i], appearance) / 1000.0;
    d.position_au = position_au(snapshot, i);
    const auto& v = snapshot.velocities()[i];
    d.velocity_km_s = glm::dvec3(v.x.value(), v.y.value(), v.z.value()) * au_per_day_to_km_s;
    d.distance_au = glm::length(d.position_au);
    d.speed_km_s = glm::length(d.velocity_km_s);

    if (const std::optional<std::size_t> primary = primary_of(snapshot, i)) {
        const std::size_t p = *primary;
        core::CartesianState relative;
        relative.position = snapshot.positions()[i] - snapshot.positions()[p];
        relative.velocity = snapshot.velocities()[i] - snapshot.velocities()[p];
        const core::space::GravitationalParameter mu =
            core::constants::gravitational_constant_space *
            (snapshot.masses()[i] + snapshot.masses()[p]);
        if (const auto elements = core::to_elements(relative, mu)) {
            OsculatingOrbit orbit;
            orbit.primary = snapshot.names()[p];
            orbit.semi_major_axis_au = elements.value().semi_major_axis.value();
            orbit.eccentricity = elements.value().eccentricity;
            orbit.inclination_deg = elements.value().inclination.radians() * kRadToDeg;
            if (orbit.eccentricity < 1.0) {
                if (const auto period =
                        core::orbital_period(elements.value().semi_major_axis, mu)) {
                    orbit.period_days = period.value().value();
                }
            }
            d.orbit = orbit;
        }
    }
    data.selected = std::move(d);
    return data;
}

BodyInspectorIntent draw_body_inspector(const BodyInspectorData& data) {
    BodyInspectorIntent intent;
    if (!ImGui::Begin(std::string(kBodyInspectorWindow).c_str())) {
        ImGui::End();
        return intent;
    }
    const ImGuiStyle& style = ImGui::GetStyle();

    section("BODIES");
    if (data.bodies.empty()) {
        // Before the first snapshot, or after every body was removed.
        muted("No bodies to show.");
        ImGui::End();
        return intent;
    }
    const float row = ImGui::GetTextLineHeightWithSpacing();
    const float list_height =
        std::min(static_cast<float>(data.bodies.size()), 6.0F) * row + style.WindowPadding.y;
    if (ImGui::BeginChild("##bodies", ImVec2(0.0F, list_height), ImGuiChildFlags_Borders)) {
        ImDrawList* draw = ImGui::GetWindowDrawList();
        for (const BodyListEntry& entry : data.bodies) {
            const bool is_selected = data.selected && data.selected->id == entry.id;
            ImGui::PushID(static_cast<int>(entry.id.value));
            const ImVec2 start = ImGui::GetCursorScreenPos();
            const std::string label = "      " + entry.name;
            if (ImGui::Selectable(label.c_str(), is_selected)) {
                intent.select = entry.id;
            }
            const float dot_y = start.y + ImGui::GetTextLineHeight() * 0.5F;
            draw->AddCircleFilled(ImVec2(start.x + 7.0F, dot_y), 3.5F,
                                  ImGui::GetColorU32(kind_color(entry.kind)));
            // The kind, right-aligned, only where it does not run into the
            // name (a narrow panel shows the names alone).
            const std::string kind = kind_label(entry.kind);
            const float kind_x =
                ImGui::GetWindowContentRegionMax().x - ImGui::CalcTextSize(kind.c_str()).x - 2.0F;
            const float name_end = ImGui::GetCursorStartPos().x +
                                   ImGui::CalcTextSize(label.c_str()).x + style.ItemSpacing.x;
            if (kind_x > name_end) {
                ImGui::SameLine(kind_x);
                muted(kind);
            }
            ImGui::PopID();
        }
    }
    ImGui::EndChild();

    if (!data.selected) {
        ImGui::Dummy(ImVec2(0.0F, 6.0F));
        muted("Select a body to inspect it.");
        ImGui::End();
        return intent;
    }
    const BodyDetails& d = *data.selected;

    ImGui::Dummy(ImVec2(0.0F, 4.0F));
    ImGui::PushFont(nullptr, style.FontSizeBase * 1.35F);
    text(d.name);
    ImGui::PopFont();
    const float focus_width = ImGui::CalcTextSize("Focus").x + style.FramePadding.x * 2.0F;
    ImGui::AlignTextToFramePadding();
    ImGui::PushStyleColor(ImGuiCol_Text, kind_color(d.kind));
    ImGui::TextUnformatted(kind_label(d.kind).c_str());
    ImGui::PopStyleColor();
    ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - focus_width);
    if (ImGui::Button("Focus")) {
        intent.focus_camera = true;
    }

    section("PHYSICAL");
    if (begin_properties("##physical")) {
        property("Mass", scientific(d.mass_solar, 4) + " Msun");
        property("", scientific(d.mass_kg, 4) + " kg");
        property("Radius", readable(d.radius_km, 1) + " km");
        if (d.radius_is_nominal) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(1);
            muted("nominal value");
        }
        ImGui::EndTable();
    }

    section("STATE  (ECLIPTIC J2000)");
    // Position and velocity side by side: axis, AU, km/s.
    if (ImGui::BeginTable("##vectors", 3, ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("axis", ImGuiTableColumnFlags_WidthFixed,
                                ImGui::CalcTextSize("x").x + 4.0F);
        ImGui::TableSetupColumn("position", ImGuiTableColumnFlags_WidthStretch, 0.55F);
        ImGui::TableSetupColumn("velocity", ImGuiTableColumnFlags_WidthStretch, 0.45F);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(1);
        muted("r [AU]");
        ImGui::TableSetColumnIndex(2);
        muted("v [km/s]");
        const std::array<const char*, 3> axes{"x", "y", "z"};
        for (int k = 0; k < 3; ++k) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            muted(axes[static_cast<std::size_t>(k)]);
            ImGui::TableSetColumnIndex(1);
            text(fixed(d.position_au[k], 6));
            ImGui::TableSetColumnIndex(2);
            text(fixed(d.velocity_km_s[k], 3));
        }
        // Magnitudes: distance from the origin and speed.
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        muted("|.|");
        ImGui::TableSetColumnIndex(1);
        text(fixed(d.distance_au, 6));
        ImGui::TableSetColumnIndex(2);
        text(fixed(d.speed_km_s, 3));
        ImGui::EndTable();
    }

    if (d.orbit) {
        const OsculatingOrbit& o = *d.orbit;
        section("OSCULATING ORBIT");
        if (begin_properties("##orbit")) {
            property("Around", o.primary);
            property("a", readable(o.semi_major_axis_au, 6) + " AU");
            property("e", fixed(o.eccentricity, 5));
            property("i", fixed(o.inclination_deg, 3) + " deg");
            if (o.period_days) {
                const double days = *o.period_days;
                property("Period",
                         days < 1000.0 ? fixed(days, 2) + " d" : fixed(days / 365.25, 2) + " yr");
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
    return intent;
}

SystemMonitorIntent draw_system_monitor(const SystemMonitorData& data) {
    SystemMonitorIntent intent;
    if (!ImGui::Begin(std::string(kSystemMonitorWindow).c_str())) {
        ImGui::End();
        return intent;
    }
    const ImGuiStyle& style = ImGui::GetStyle();

    draw_monitor_tiles(monitor_tiles(data));
    if (data.warp) {
        draw_warp_reason(*data.warp);
    }

    ImGui::Dummy(ImVec2(0.0F, 2.0F));
    ImGui::AlignTextToFramePadding();
    muted(std::string(data.series_label));
    const float clear_width = ImGui::CalcTextSize("Clear").x + style.FramePadding.x * 2.0F;
    ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - clear_width);
    if (ImGui::SmallButton("Clear")) {
        intent.clear_series = true;
    }

    const std::size_t n = std::min(data.series_time_days.size(), data.series_values.size());
    const ImPlotFlags plot_flags =
        ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect | ImPlotFlags_NoLegend | ImPlotFlags_NoTitle;
    const float plot_height = std::max(ImGui::GetContentRegionAvail().y - 2.0F, 60.0F);
    // Time ticks in days or, for long series, years, with as many decimals as
    // the span needs (no repeated "0 d" labels on short runs). Read by the
    // formatter until EndPlot.
    TimeAxis time_axis =
        n > 0 ? time_axis_for(std::abs(data.series_time_days[n - 1] - data.series_time_days[0]))
              : TimeAxis{};
    std::vector<double> times(data.series_time_days.begin(),
                              data.series_time_days.begin() + static_cast<std::ptrdiff_t>(n));
    for (double& t : times) {
        t /= time_axis.days_per_unit;
    }
    if (ImPlot::BeginPlot("##series", ImVec2(-1.0F, plot_height), plot_flags)) {
        if (n == 0) {
            // Nothing recorded yet: a quiet empty frame instead of the
            // placeholder axis range [0, 1] and its tick labels.
            ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoTickLabels,
                              ImPlotAxisFlags_NoTickLabels);
            ImPlot::SetupAxesLimits(0.0, 1.0, 0.0, 1.0, ImPlotCond_Always);
            ImPlot::PlotText("No data recorded yet", 0.5, 0.5);
        } else {
            ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            ImPlot::SetupAxisFormat(ImAxis_X1, format_time_tick, &time_axis);
            ImPlot::SetupAxisFormat(ImAxis_Y1, "%.1e");
            const ImVec4 accent = ImGui::GetStyleColorVec4(ImGuiCol_PlotLines);
            ImPlotSpec spec;
            spec.LineColor = accent;
            spec.LineWeight = 1.5F;
            spec.FillColor = ImVec4(accent.x, accent.y, accent.z, 0.12F);
            ImPlot::PlotShaded("##fill", times.data(), data.series_values.data(),
                               static_cast<int>(n), 0.0, spec);
            ImPlot::PlotLine("##line", times.data(), data.series_values.data(), static_cast<int>(n),
                             spec);
        }
        ImPlot::EndPlot();
    }
    ImGui::End();
    return intent;
}

RenderSettingsIntent draw_render_settings(const RenderSettings& settings,
                                          const RenderSettingsContext& context) {
    RenderSettingsIntent intent;
    if (!ImGui::Begin(std::string(kRenderSettingsWindow).c_str())) {
        ImGui::End();
        return intent;
    }
    RenderSettings s = settings;
    bool changed = false;

    section("BODIES");
    int mode = s.scale_mode == ScaleMode::clamped ? 0 : 1;
    row_label("Scale");
    if (ImGui::RadioButton("Clamped", &mode, 0)) {
        changed = true;
    }
    // The second option wraps under the first in a narrow panel.
    const ImGuiStyle& style = ImGui::GetStyle();
    const float true_width =
        ImGui::GetFrameHeight() + style.ItemInnerSpacing.x + ImGui::CalcTextSize("True").x;
    const float right_edge = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
    if (ImGui::GetItemRectMax().x + style.ItemSpacing.x + true_width <= right_edge) {
        ImGui::SameLine();
    } else {
        ImGui::SetCursorPosX(control_column());
    }
    if (ImGui::RadioButton("True", &mode, 1)) {
        changed = true;
    }
    s.scale_mode = mode == 0 ? ScaleMode::clamped : ScaleMode::true_scale;

    auto min_scale = static_cast<float>(s.min_radius_scale);
    row_label("Min. size");
    ImGui::BeginDisabled(s.scale_mode == ScaleMode::true_scale);
    if (ImGui::SliderFloat("##min_size", &min_scale, 0.25F, 8.0F, "%.2fx",
                           ImGuiSliderFlags_Logarithmic)) {
        s.min_radius_scale = static_cast<double>(min_scale);
        changed = true;
    }
    ImGui::EndDisabled();

    row_label("LOD bias");
    if (ImGui::SliderInt("##lod_bias", &s.lod_bias, -3, 3, "%+d")) {
        changed = true;
    }
    row_label("Star glare");
    changed |= ImGui::Checkbox("##glow", &s.star_glow);

    section("TRAILS");
    row_label("Show");
    changed |= ImGui::Checkbox("##trails", &s.show_trails);
    ImGui::BeginDisabled(!s.show_trails);
    auto length = static_cast<int>(s.trail_length);
    row_label("Length");
    if (ImGui::SliderInt("##length", &length, 16, 4096, "%d samples",
                         ImGuiSliderFlags_Logarithmic)) {
        s.trail_length = static_cast<std::size_t>(length);
        changed = true;
    }
    auto width = static_cast<float>(s.trail_width_px);
    row_label("Width");
    if (ImGui::SliderFloat("##width", &width, 0.5F, 6.0F, "%.1f px")) {
        s.trail_width_px = static_cast<double>(width);
        changed = true;
    }
    auto trail_brightness = static_cast<float>(s.trail_brightness);
    row_label("Brightness");
    if (ImGui::SliderFloat("##trail_brightness", &trail_brightness, 0.0F, 4.0F, "%.2f")) {
        s.trail_brightness = static_cast<double>(trail_brightness);
        changed = true;
    }
    ImGui::EndDisabled();

    section("SKY");
    row_label("Starfield");
    changed |= ImGui::Checkbox("##starfield", &s.show_starfield);
    ImGui::BeginDisabled(!s.show_starfield);
    auto stars = static_cast<float>(s.star_brightness);
    row_label("Stars");
    if (ImGui::SliderFloat("##stars", &stars, 0.0F, 4.0F, "%.2f")) {
        s.star_brightness = static_cast<double>(stars);
        changed = true;
    }
    auto milky_way = static_cast<float>(s.milky_way_brightness);
    row_label("Milky Way");
    if (ImGui::SliderFloat("##milky_way", &milky_way, 0.0F, 4.0F, "%.2f")) {
        s.milky_way_brightness = static_cast<double>(milky_way);
        changed = true;
    }
    ImGui::EndDisabled();

    if (context.black_hole) {
        section("BLACK HOLE");
        row_label("Lensing");
        changed |= ImGui::Checkbox("##lensing", &s.black_hole_lensing);
        ImGui::SetItemTooltip("Bend the sky and draw the shadow and the accretion disk by\n"
                              "tracing light rays through the Schwarzschild geometry");
        ImGui::BeginDisabled(!s.black_hole_lensing);
        row_label("Disk");
        changed |= ImGui::Checkbox("##disk", &s.accretion_disk);
        ImGui::BeginDisabled(!s.accretion_disk);
        auto disk_brightness = static_cast<float>(s.disk_brightness);
        row_label("Brightness");
        if (ImGui::SliderFloat("##disk_brightness", &disk_brightness, 0.0F, 4.0F, "%.2f")) {
            s.disk_brightness = static_cast<double>(disk_brightness);
            changed = true;
        }
        auto temperature = static_cast<float>(s.disk_temperature);
        row_label("Temp.");
        if (ImGui::SliderFloat("##disk_temperature", &temperature, 2000.0F, 20000.0F, "%.0f K",
                               ImGuiSliderFlags_Logarithmic)) {
            s.disk_temperature = static_cast<double>(temperature);
            changed = true;
        }
        ImGui::SetItemTooltip("Peak temperature of the disk in its rest frame; the observed\n"
                              "colour also carries the Doppler and gravitational shifts");
        ImGui::EndDisabled();
        row_label("Steps");
        if (ImGui::SliderInt("##geodesic_steps", &s.geodesic_steps, 64, 512, "%d per ray")) {
            changed = true;
        }
        ImGui::SetItemTooltip("Integration steps per light ray: fewer are faster and coarser\n"
                              "close to the photon ring");
        ImGui::EndDisabled();
    }

    section("EXPOSURE");
    auto ev = static_cast<float>(std::log2(std::max(s.exposure, 1.0e-6)));
    row_label("Exposure");
    if (ImGui::SliderFloat("##exposure", &ev, -6.0F, 6.0F, "%+.1f EV")) {
        s.exposure = std::exp2(static_cast<double>(ev));
        changed = true;
    }
    auto falloff = static_cast<float>(s.light_falloff_exponent);
    row_label("Falloff");
    if (ImGui::SliderFloat("##falloff", &falloff, 0.0F, 2.0F, "1/r^%.2f")) {
        s.light_falloff_exponent = static_cast<double>(falloff);
        changed = true;
    }
    ImGui::Dummy(ImVec2(0.0F, 4.0F));
    if (ImGui::Button("Reset to defaults", ImVec2(-1.0F, 0.0F))) {
        s = RenderSettings{};
        changed = true;
    }
    ImGui::End();
    if (changed) {
        intent.changed = sanitized(s);
    }
    return intent;
}

std::vector<SceneLabel> make_scene_labels(const core::SimSnapshot& snapshot,
                                          std::span<const BodyScreenInfo> screen,
                                          std::optional<core::BodyId> selected,
                                          double min_spacing_px) {
    std::vector<const BodyScreenInfo*> order;
    for (const BodyScreenInfo& info : screen) {
        if (info.visible && info.index < snapshot.size()) {
            order.push_back(&info);
        }
    }
    const auto priority = [&](const BodyScreenInfo* info) {
        const bool is_selected = selected && info->id == *selected;
        return std::pair<int, double>(is_selected ? 0 : 1, -snapshot.masses()[info->index].value());
    };
    std::stable_sort(order.begin(), order.end(),
                     [&](const BodyScreenInfo* a, const BodyScreenInfo* b) {
                         return priority(a) < priority(b);
                     });
    std::vector<SceneLabel> labels;
    for (const BodyScreenInfo* info : order) {
        bool crowded = false;
        for (const SceneLabel& placed : labels) {
            crowded = crowded || glm::length(placed.pixel - info->pixel) < min_spacing_px;
        }
        if (crowded) {
            continue;
        }
        labels.push_back({info->pixel, info->radius_px, snapshot.names()[info->index],
                          snapshot.kind_tag(info->index), selected && info->id == *selected});
    }
    return labels;
}

void draw_scene_labels(std::span<const SceneLabel> labels) {
    ImDrawList* draw = ImGui::GetBackgroundDrawList();
    const ImU32 text_color = ImGui::GetColorU32(ImGuiCol_Text, 0.82F);
    const ImU32 accent = ImGui::GetColorU32(ImGuiCol_CheckMark);
    const ImU32 leader = ImGui::GetColorU32(ImGuiCol_TextDisabled, 0.7F);
    // A soft dark shadow keeps names readable over the Milky Way and trails.
    const ImU32 shadow = ImGui::GetColorU32(ImVec4(0.0F, 0.0F, 0.0F, 0.7F));
    const float font = ImGui::GetFontSize();
    // Labels are placed in order (most important first); one whose text box
    // would overlap a placed one is dropped.
    std::vector<ImRect> placed;
    for (const SceneLabel& label : labels) {
        const auto x = static_cast<float>(label.pixel.x);
        const auto y = static_cast<float>(label.pixel.y);
        const float r = static_cast<float>(std::min(label.radius_px, 2000.0)) + 3.0F;
        const ImVec2 a(x + r * 0.7071F, y - r * 0.7071F);
        const ImVec2 b(a.x + 7.0F, a.y - 7.0F);
        const ImVec2 text_pos(b.x + 3.0F, b.y - font * 0.6F);
        const ImVec2 size = ImGui::CalcTextSize(label.name.c_str());
        const ImRect box(ImVec2(text_pos.x - 2.0F, text_pos.y - 1.0F),
                         ImVec2(text_pos.x + size.x + 2.0F, text_pos.y + size.y + 1.0F));
        bool overlaps = false;
        for (const ImRect& other : placed) {
            overlaps = overlaps || box.Overlaps(other);
        }
        if (overlaps && !label.selected) {
            continue;
        }
        placed.push_back(box);
        if (label.selected) {
            draw->AddCircle(ImVec2(x, y), r + 3.0F, accent, 0, 1.5F);
        }
        draw->AddLine(a, b, label.selected ? accent : leader, 1.0F);
        draw->AddText(ImVec2(text_pos.x + 1.0F, text_pos.y + 1.0F), shadow, label.name.c_str());
        draw->AddText(text_pos, label.selected ? accent : text_color, label.name.c_str());
    }
}

} // namespace orrery::render
