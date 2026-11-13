#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include <imgui.h>
#include <implot.h>

#include "../aero/aero_widgets.h"
#include "entry_workspace_impl.h"
#include "orrery/render/entry_geometry.h"

// Entry Profile, Ground Track and the scene overlay (legend and outcome
// banner) of the Entry workspace.
namespace orrery::app {

namespace w = widgets;

namespace {

constexpr const char* kDot = " \xC2\xB7 ";

// Overlaid runs of the comparison: Okabe-Ito colours without the orange of
// the current run (distinguishable with the common colour-vision
// deficiencies, and drawn thinner than the current run).
constexpr std::array<ImU32, 5> kOverlayColors{
    IM_COL32(86, 180, 233, 255), IM_COL32(0, 158, 115, 255), IM_COL32(204, 121, 167, 255),
    IM_COL32(240, 228, 66, 255), IM_COL32(0, 114, 178, 255)};

struct Plot {
    const char* id;
    const char* title; // drawn in the plot's top-left corner
    const char* unit;  // y axis
    const std::vector<double> sim::EntrySeries::* values;
    double scale;
};

constexpr std::array<Plot, 5> kPlots{{
    {"##altitude", "Altitude", "km", &sim::EntrySeries::altitude_m, 1.0e-3},
    {"##speed", "Speed", "km/s", &sim::EntrySeries::speed_m_s, 1.0e-3},
    {"##g", "g-load", "g", &sim::EntrySeries::g_load, 1.0},
    {"##q", "Dynamic pressure", "kPa", &sim::EntrySeries::dynamic_pressure_pa, 1.0e-3},
    {"##heat", "Stagnation heat flux", "kW/m\xC2\xB2", &sim::EntrySeries::heat_flux_w_m2, 1.0e-3},
}};

// The quantity's name inside the plot, top left, on a faint backing so the
// curves stay readable under it.
void plot_title(const char* title) {
    ImDrawList* draw = ImPlot::GetPlotDrawList();
    const ImVec2 pos = ImPlot::GetPlotPos();
    const ImVec2 size = ImGui::CalcTextSize(title);
    const ImVec2 at(pos.x + 6.0F, pos.y + 4.0F);
    draw->AddRectFilled(ImVec2(at.x - 3.0F, at.y - 1.0F),
                        ImVec2(at.x + size.x + 3.0F, at.y + size.y + 1.0F),
                        ImGui::GetColorU32(ImGuiCol_WindowBg, 0.9F), 3.0F);
    draw->AddText(at, ImGui::GetColorU32(ImGuiCol_TextDisabled), title);
}

struct ScaledSeries {
    const double* x;
    const double* y;
    double scale;
};

ImPlotPoint scaled_point(int index, void* data) {
    const auto* s = static_cast<const ScaledSeries*>(data);
    const auto i = static_cast<std::size_t>(index);
    return {s->x[i], s->y[i] * s->scale};
}

void plot_series(const char* label, const sim::EntrySeries& series, const Plot& plot, ImU32 color,
                 float weight, bool legend = true) {
    const std::vector<double>& y = series.*(plot.values);
    if (series.time_s.size() != y.size() || y.empty()) {
        return;
    }
    ScaledSeries data{series.time_s.data(), y.data(), plot.scale};
    ImPlotSpec spec;
    spec.LineColor = ImGui::ColorConvertU32ToFloat4(color);
    spec.LineWeight = weight;
    spec.Flags = legend ? ImPlotItemFlags_None : ImPlotItemFlags_NoLegend;
    ImPlot::PlotLineG(label, scaled_point, &data, static_cast<int>(y.size()), spec);
}

std::string heat_text(double w_m2) {
    if (w_m2 >= 1.0e6) {
        return w::fixed(w_m2 * 1.0e-6, 2) + " MW/m\xC2\xB2";
    }
    return w::fixed(w_m2 * 1.0e-3, 0) + " kW/m\xC2\xB2";
}

std::string duration_text(double seconds) {
    if (seconds >= 120.0) {
        const auto minutes = static_cast<int>(seconds / 60.0);
        return std::to_string(minutes) + " min " + w::fixed(seconds - 60.0 * minutes, 0) + " s";
    }
    return w::fixed(seconds, 1) + " s";
}

// Theme colour of an outcome (always shown together with its label).
render::UiColor outcome_tint(sim::EntryOutcome outcome) {
    switch (outcome) {
        case sim::EntryOutcome::landed:
            return w::theme().accent_secondary;
        case sim::EntryOutcome::skip_out:
            return w::theme().warning;
        case sim::EntryOutcome::orbiting:
        case sim::EntryOutcome::none:
            return w::theme().text_muted;
    }
    return w::theme().text_muted;
}

const char* outcome_note(sim::EntryOutcome outcome) {
    switch (outcome) {
        case sim::EntryOutcome::landed:
            return "reached the ground";
        case sim::EntryOutcome::skip_out:
            return "left the atmosphere again";
        case sim::EntryOutcome::orbiting:
            return "still flying at the time limit";
        case sim::EntryOutcome::none:
            return "";
    }
    return "";
}

ImU32 map_color(const render::ScalarRange& range, double value, float alpha = 1.0F) {
    const glm::dvec3 c = render::colormap_srgb(render::Colormap::sequential,
                                               render::entry_map_position(range, value));
    return ImGui::GetColorU32(
        ImVec4(static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(c.z), alpha));
}

// Horizontal colour bar with ticks below it, inside [x, x + width].
void colour_bar(ImDrawList* draw, ImVec2 pos, float width, float height,
                const render::ScalarRange& range) {
    constexpr int kSteps = 64;
    for (int i = 0; i < kSteps; ++i) {
        const float x0 = pos.x + width * static_cast<float>(i) / kSteps;
        const float x1 = pos.x + width * static_cast<float>(i + 1) / kSteps;
        const double value = range.min + (range.max - range.min) * (i + 0.5) / kSteps;
        draw->AddRectFilled(ImVec2(x0, pos.y), ImVec2(x1 + 0.5F, pos.y + height),
                            map_color(range, value));
    }
    draw->AddRect(pos, ImVec2(pos.x + width, pos.y + height), ImGui::GetColorU32(ImGuiCol_Border),
                  0.0F, 0, 1.0F);
    const ImU32 ink = ImGui::GetColorU32(ImGuiCol_Text);
    for (const render::LegendTick& tick : render::legend_ticks(range, 5)) {
        const float x = pos.x + width * static_cast<float>(tick.position);
        draw->AddLine(ImVec2(x, pos.y + height), ImVec2(x, pos.y + height + 4.0F), ink, 1.0F);
        const ImVec2 size = ImGui::CalcTextSize(tick.label.c_str());
        const float tx = std::clamp(x - size.x * 0.5F, pos.x - 4.0F, pos.x + width - size.x + 4.0F);
        draw->AddText(ImVec2(tx, pos.y + height + 5.0F), ink, tick.label.c_str());
    }
}

} // namespace

// --- Entry Profile ---------------------------------------------------------------

void EntryWorkspace::Impl::draw_profile_panel() {
    if (!ImGui::Begin(std::string(kEntryProfileWindow).c_str())) {
        ImGui::End();
        return;
    }
    const std::shared_ptr<const sim::EntryTrace> trace = shown();
    const sim::EntryProgress& p = runner.latest();
    if (p.status.state == sim::EntryRunState::failed && run_setup && p.status.run == run_number) {
        w::error("The entry failed: " + p.status.message);
    }
    draw_readouts(trace.get());
    if (trace && !trace->series.empty()) {
        draw_cursor_line(*trace);
    }
    draw_plots(trace.get());
    ImGui::End();
}

void EntryWorkspace::Impl::draw_readouts(const sim::EntryTrace* trace) {
    if (trace == nullptr || trace->series.empty()) {
        w::muted(active() ? "Integrating..." : "No run of these inputs yet: press Run (Space).");
        return;
    }
    const sim::EntrySummary& s = trace->summary;
    if (s.outcome != sim::EntryOutcome::none) {
        std::string label(sim::outcome_label(s.outcome));
        std::transform(label.begin(), label.end(), label.begin(), [](char c) {
            return static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        });
        w::chip(label, outcome_tint(s.outcome), outcome_note(s.outcome));
    } else {
        w::chip(active() ? "IN FLIGHT" : "INCOMPLETE", w::theme().text_muted,
                "The run has not reached the ground, left the atmosphere or hit the time limit");
    }
    if (shows_record()) {
        ImGui::SameLine();
        w::muted("recorded run");
    }
    if (!w::begin_properties("##summary", "Peak dyn. pressure")) {
        return;
    }
    w::property("Peak g-load",
                w::fixed(s.peak_g, 2) + " g at " + w::fixed(s.peak_g_time_s, 0) + " s");
    w::property("Peak heat flux", heat_text(s.peak_heat_flux_w_m2) + " at " +
                                      w::fixed(s.peak_heat_flux_time_s, 0) + " s");
    w::property("Peak dyn. pressure", w::fixed(s.peak_dynamic_pressure_pa * 1.0e-3, 1) + " kPa");
    w::property("Heat load", w::fixed(s.heat_load_j_m2 * 1.0e-6, 1) + " MJ/m\xC2\xB2");
    w::property("Duration", duration_text(s.duration_s));
    w::property("Downrange", w::fixed(s.downrange_m * 1.0e-3, 0) + " km");
    w::end_properties();
}

void EntryWorkspace::Impl::draw_cursor_line(const sim::EntryTrace& trace) {
    const std::optional<double> t = cursor();
    if (!t) {
        return;
    }
    const sim::EntrySeries& s = trace.series;
    const auto at = [&](const std::vector<double>& v) {
        return sim::value_at(s.time_s, v, *t).value_or(0.0);
    };
    const char* caption = "CURSOR AT PEAK g";
    if (hover_time) {
        caption = "CURSOR";
    } else if (pinned_time) {
        caption = "CURSOR (PINNED, DOUBLE-CLICK TO RELEASE)";
    } else if (active()) {
        caption = "CURSOR AT THE LATEST SAMPLE";
    }
    w::section(caption);
    const std::array<std::pair<const char*, std::string>, 6> cells{{
        {"t", w::fixed(*t, 1) + " s"},
        {"h", w::fixed(at(s.altitude_m) * 1.0e-3, 1) + " km"},
        {"v", w::fixed(at(s.speed_m_s) * 1.0e-3, 2) + " km/s"},
        {"n", w::fixed(at(s.g_load), 2) + " g"},
        {"q", w::fixed(at(s.dynamic_pressure_pa) * 1.0e-3, 1) + " kPa"},
        {"Q", heat_text(at(s.heat_flux_w_m2))},
    }};
    if (!ImGui::BeginTable("##cursor", 3, ImGuiTableFlags_SizingStretchSame)) {
        return;
    }
    for (const auto& [symbol, value] : cells) {
        ImGui::TableNextColumn();
        w::muted(symbol);
        ImGui::SameLine();
        w::text(value);
    }
    ImGui::EndTable();
    w::tooltip("t time, h altitude, v speed, n g-load, q dynamic pressure, Q stagnation heat "
               "flux. Hover a plot or the map to move the cursor, click to pin it.");
}

void EntryWorkspace::Impl::draw_plots(const sim::EntryTrace* trace) {
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    if (avail.y < ImGui::GetFontSize() * 6.0F) {
        return;
    }
    constexpr ImPlotSubplotFlags kSubplotFlags =
        ImPlotSubplotFlags_LinkAllX | ImPlotSubplotFlags_NoTitle | ImPlotSubplotFlags_NoResize;
    if (!ImPlot::BeginSubplots("##profile", static_cast<int>(kPlots.size()), 1, avail,
                               kSubplotFlags)) {
        return;
    }
    const std::optional<double> t = cursor();
    const ImU32 current = ImGui::GetColorU32(w::color(w::theme().accent));
    constexpr ImPlotFlags kFlags = ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect |
                                   ImPlotFlags_NoTitle | ImPlotFlags_NoMouseText;
    for (std::size_t row = 0; row < kPlots.size(); ++row) {
        const Plot& plot = kPlots[row];
        const bool last = row + 1 == kPlots.size();
        if (!ImPlot::BeginPlot(plot.id, ImVec2(-1.0F, 0.0F),
                               kFlags | (row == 0 ? 0 : ImPlotFlags_NoLegend))) {
            continue;
        }
        ImPlot::SetupAxes(last ? "time [s]" : nullptr, plot.unit,
                          ImPlotAxisFlags_AutoFit | (last ? 0 : ImPlotAxisFlags_NoTickLabels),
                          ImPlotAxisFlags_AutoFit);
        if (row == 0) {
            ImPlot::SetupLegend(ImPlotLocation_NorthEast);
        }
        bool overlays = false;
        for (std::size_t i = 0; i < listing.entries.size() && i < compared.size(); ++i) {
            const sim::StoredEntry& e = listing.entries[i];
            if (compared[i] == 0 || !e.result || !e.result->series.consistent()) {
                continue;
            }
            overlays = true;
            const std::string name =
                (e.setup.name.empty() ? e.path.stem().string() : e.setup.name) + "##o" +
                std::to_string(i);
            plot_series(name.c_str(), e.result->series, plot,
                        kOverlayColors[i % kOverlayColors.size()], 1.2F);
        }
        if (trace != nullptr && trace->series.consistent()) {
            plot_series(shows_record() ? "recorded" : "this run", trace->series, plot, current,
                        2.0F, overlays);
        }
        if (t) {
            ImPlotSpec spec;
            spec.LineColor = ImGui::GetStyleColorVec4(ImGuiCol_Text);
            spec.LineWeight = 1.0F;
            spec.Flags = ImPlotItemFlags_NoLegend;
            ImPlot::PlotInfLines("##cursor", &*t, 1, spec);
        }
        ImPlot::PushPlotClipRect();
        plot_title(plot.title);
        ImPlot::PopPlotClipRect();
        if (ImPlot::IsPlotHovered()) {
            const double x = ImPlot::GetPlotMousePos().x;
            hover_time = x;
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                pinned_time = x;
            }
            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                pinned_time.reset();
            }
        }
        ImPlot::EndPlot();
    }
    ImPlot::EndSubplots();
}

// --- Ground Track ----------------------------------------------------------------

namespace {

// Longitudes made continuous across the date line (each step taken the short
// way round), so a track that crosses it is drawn and framed in one piece.
std::vector<double> unwrapped_longitudes(const std::vector<double>& lon) {
    std::vector<double> out;
    out.reserve(lon.size());
    for (const double l : lon) {
        if (out.empty()) {
            out.push_back(l);
            continue;
        }
        double step = std::fmod(l - out.back(), 360.0);
        if (step > 180.0) {
            step -= 360.0;
        } else if (step < -180.0) {
            step += 360.0;
        }
        out.push_back(out.back() + step);
    }
    return out;
}

// Axis labels of unwrapped longitudes, back in (-180, 180].
int longitude_label(double value, char* buffer, int size, void* /*data*/) {
    double wrapped = std::fmod(value + 180.0, 360.0);
    wrapped = (wrapped <= 0.0 ? wrapped + 360.0 : wrapped) - 180.0;
    return std::snprintf(buffer, static_cast<std::size_t>(size), "%g\xC2\xB0", wrapped);
}

// Frames the track with a margin (at least 20 degrees across), else the
// whole planet.
void setup_map_axes(const std::vector<double>& lon, const std::vector<double>& lat) {
    ImPlot::SetupAxes("longitude", "latitude");
    ImPlot::SetupAxisFormat(ImAxis_X1, longitude_label);
    ImPlot::SetupAxisFormat(ImAxis_Y1, "%g\xC2\xB0");
    if (lon.size() < 2 || lon.size() != lat.size()) {
        ImPlot::SetupAxesLimits(-180.0, 180.0, -90.0, 90.0, ImPlotCond_Always);
        return;
    }
    const auto [lon_lo, lon_hi] = std::minmax_element(lon.begin(), lon.end());
    const auto [lat_lo, lat_hi] = std::minmax_element(lat.begin(), lat.end());
    const double cx = 0.5 * (*lon_lo + *lon_hi);
    const double cy = 0.5 * (*lat_lo + *lat_hi);
    const double half = std::max({0.5 * (*lon_hi - *lon_lo), *lat_hi - *lat_lo, 10.0}) * 1.25;
    ImPlot::SetupAxesLimits(cx - half, cx + half, cy - half * 0.5, cy + half * 0.5,
                            ImPlotCond_Always);
}

constexpr ImU32 kMapOutline = IM_COL32(4, 5, 8, 220);

// The track as an outlined line coloured by `values` (in the legend's
// units) with the shared sequential map.
void draw_map_track(const std::vector<double>& lon, const std::vector<double>& lat,
                    const std::vector<double>& values, double scale,
                    const render::ScalarRange& range) {
    ImDrawList* draw = ImPlot::GetPlotDrawList();
    for (int pass = 0; pass < 2; ++pass) {
        for (std::size_t i = 1; i < lon.size(); ++i) {
            const ImVec2 a = ImPlot::PlotToPixels(lon[i - 1], lat[i - 1]);
            const ImVec2 b = ImPlot::PlotToPixels(lon[i], lat[i]);
            if (pass == 0) {
                draw->AddLine(a, b, kMapOutline, 5.0F);
            } else {
                draw->AddLine(a, b, map_color(range, values[i] * scale), 3.0F);
            }
        }
    }
}

void draw_map_marker(ImVec2 centre, ImU32 fill, float radius) {
    ImDrawList* draw = ImPlot::GetPlotDrawList();
    draw->AddCircleFilled(centre, radius + 1.5F, kMapOutline);
    draw->AddCircleFilled(centre, radius, fill);
}

// Time of the track point nearest to the pointer within 24 px.
std::optional<double> map_hover(const std::vector<double>& lon, const std::vector<double>& lat,
                                const std::vector<double>& time) {
    const ImVec2 mouse = ImGui::GetMousePos();
    float best = 24.0F * 24.0F;
    std::optional<double> hit;
    for (std::size_t i = 0; i < lon.size(); ++i) {
        const ImVec2 q = ImPlot::PlotToPixels(lon[i], lat[i]);
        const float d = (q.x - mouse.x) * (q.x - mouse.x) + (q.y - mouse.y) * (q.y - mouse.y);
        if (d < best) {
            best = d;
            hit = time[i];
        }
    }
    return hit;
}

} // namespace

void EntryWorkspace::Impl::draw_ground_track_panel() {
    if (!ImGui::Begin(std::string(kGroundTrackWindow).c_str())) {
        ImGui::End();
        return;
    }
    const std::shared_ptr<const sim::EntryTrace> trace = shown();
    const bool have = trace && trace->series.consistent() && trace->series.size() >= 2;
    const std::vector<double> lon =
        have ? unwrapped_longitudes(trace->series.longitude_deg) : std::vector<double>{};
    const std::vector<double> none;
    const std::vector<double>& lat = have ? trace->series.latitude_deg : none;
    constexpr ImPlotFlags kFlags = ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect |
                                   ImPlotFlags_NoTitle | ImPlotFlags_NoLegend | ImPlotFlags_Equal;
    if (!ImPlot::BeginPlot("##ground_track", ImGui::GetContentRegionAvail(), kFlags)) {
        ImGui::End();
        return;
    }
    setup_map_axes(lon, lat);
    if (!have) {
        ImPlot::PlotText("No ground track yet", 0.0, 0.0);
        ImPlot::EndPlot();
        ImGui::End();
        return;
    }
    const sim::EntrySeries& s = trace->series;
    ImPlot::PushPlotClipRect();
    draw_map_track(lon, lat, coloring_values(s, layers.coloring),
                   entry_quantity(layers.coloring).scale, coloring_range(s, layers.coloring));
    const double peak = layers.coloring == EntryColoring::heat_flux
                            ? trace->summary.peak_heat_flux_time_s
                            : trace->summary.peak_g_time_s;
    // The peak is a larger disc, so the cursor dot stays visible on it.
    if (const auto i = sim::nearest_index(s.time_s, peak)) {
        draw_map_marker(ImPlot::PlotToPixels(lon[*i], lat[*i]),
                        ImGui::GetColorU32(w::color(w::theme().accent)), 7.0F);
    }
    if (const std::optional<double> t = cursor()) {
        if (const auto i = sim::nearest_index(s.time_s, *t)) {
            draw_map_marker(ImPlot::PlotToPixels(lon[*i], lat[*i]), IM_COL32(242, 244, 248, 255),
                            4.5F);
        }
    }
    ImPlot::PopPlotClipRect();
    if (ImPlot::IsPlotHovered()) {
        if (const std::optional<double> t = map_hover(lon, lat, s.time_s)) {
            hover_time = t;
        }
    }
    ImPlot::EndPlot();
    ImGui::End();
}
// --- Scene overlay ----------------------------------------------------------------

void EntryWorkspace::Impl::draw_scene_overlay(const render::UiRect& scene) {
    if (scene.width < 40.0 || scene.height < 40.0) {
        return;
    }
    constexpr ImGuiWindowFlags kFlags =
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoNav |
        ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoBringToFrontOnFocus |
        ImGuiWindowFlags_NoDocking;
    ImGui::SetNextWindowPos(ImVec2(static_cast<float>(scene.x), static_cast<float>(scene.y)));
    ImGui::SetNextWindowSize(
        ImVec2(static_cast<float>(scene.width), static_cast<float>(scene.height)));
    if (!ImGui::Begin("##entry_scene_overlay", nullptr, kFlags)) {
        ImGui::End();
        return;
    }
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const auto x0 = static_cast<float>(scene.x);
    const auto y0 = static_cast<float>(scene.y);
    const auto sw = static_cast<float>(scene.width);
    const auto sh = static_cast<float>(scene.height);
    const float pad = ImGui::GetFontSize() * 0.8F;
    const ImU32 panel = ImGui::GetColorU32(w::color(w::theme().window, 0.86F));
    const ImU32 border = ImGui::GetColorU32(ImGuiCol_Border);
    const std::shared_ptr<const sim::EntryTrace> trace = shown();
    if (!trace || trace->series.size() < 2) {
        const char* hint = active() ? "Integrating..." : "Set up an entry and press Run (Space)";
        const ImVec2 size = ImGui::CalcTextSize(hint);
        draw->AddText(ImVec2(x0 + (sw - size.x) * 0.5F, y0 + (sh - size.y) * 0.5F),
                      ImGui::GetColorU32(ImGuiCol_TextDisabled), hint);
        ImGui::End();
        return;
    }
    // Outcome banner, top centre.
    const sim::EntrySummary& s = trace->summary;
    std::string banner;
    render::UiColor tint = w::theme().text_muted;
    if (s.outcome != sim::EntryOutcome::none) {
        banner = std::string(sim::outcome_label(s.outcome)) + kDot + "peak " +
                 w::fixed(s.peak_g, 2) + " g" + kDot + heat_text(s.peak_heat_flux_w_m2) + kDot +
                 duration_text(s.duration_s);
        tint = outcome_tint(s.outcome);
    } else {
        banner = std::string(active() ? "In flight" : "Incomplete") + kDot + "t " +
                 w::fixed(trace->series.time_s.back(), 0) + " s" + kDot + "peak so far " +
                 w::fixed(s.peak_g, 2) + " g";
    }
    const ImVec2 banner_size = ImGui::CalcTextSize(banner.c_str());
    const ImVec2 b0(x0 + (sw - banner_size.x) * 0.5F - pad, y0 + pad * 0.8F);
    const ImVec2 b1(b0.x + banner_size.x + 2.0F * pad, b0.y + banner_size.y + pad);
    draw->AddRectFilled(b0, b1, panel, 6.0F);
    draw->AddRect(b0, b1, ImGui::GetColorU32(w::color(tint, 0.8F)), 6.0F, 0, 1.2F);
    draw->AddRectFilled(ImVec2(b0.x, b0.y + 3.0F), ImVec2(b0.x + 3.0F, b1.y - 3.0F),
                        ImGui::GetColorU32(w::color(tint)));
    draw->AddText(ImVec2(b0.x + pad, b0.y + pad * 0.5F), ImGui::GetColorU32(ImGuiCol_Text),
                  banner.c_str());

    // Legend, bottom right.
    const EntryQuantity quantity = entry_quantity(layers.coloring);
    const render::ScalarRange range = coloring_range(trace->series, layers.coloring);
    const std::string title = std::string(quantity.title) + " [" + quantity.unit + "]";
    const float font = ImGui::GetFontSize();
    const std::string scale_note =
        "Altitude x" + w::fixed(std::max(1.0, layers.altitude_scale), 0) + "   ";
    const float key_w = ImGui::CalcTextSize((scale_note + "peak   cursor").c_str()).x + font * 1.8F;
    const float bar_w = std::max({font * 14.0F, ImGui::CalcTextSize(title.c_str()).x, key_w});
    const float bar_h = font * 0.7F;
    const float panel_w = bar_w + 2.0F * pad;
    const float panel_h = pad + font + 6.0F + bar_h + font * 1.4F + font + pad;
    const ImVec2 l0(x0 + sw - panel_w - pad, y0 + sh - panel_h - pad);
    draw->AddRectFilled(l0, ImVec2(l0.x + panel_w, l0.y + panel_h), panel, 6.0F);
    draw->AddRect(l0, ImVec2(l0.x + panel_w, l0.y + panel_h), border, 6.0F);
    draw->AddText(ImVec2(l0.x + pad, l0.y + pad), ImGui::GetColorU32(ImGuiCol_Text), title.c_str());
    colour_bar(draw, ImVec2(l0.x + pad, l0.y + pad + font + 6.0F), bar_w, bar_h, range);
    // Key of the markers: amber disc = peak, white disc = cursor.
    const float ky = l0.y + panel_h - pad - font * 0.5F;
    float kx = l0.x + pad;
    const ImU32 muted = ImGui::GetColorU32(ImGuiCol_TextDisabled);
    draw->AddText(ImVec2(kx, ky - font * 0.5F), muted, scale_note.c_str());
    kx += ImGui::CalcTextSize(scale_note.c_str()).x;
    for (const auto& [fill, label] :
         {std::pair<ImU32, const char*>{ImGui::GetColorU32(w::color(w::theme().accent)), "peak   "},
          std::pair<ImU32, const char*>{IM_COL32(242, 244, 248, 255), "cursor"}}) {
        draw->AddCircleFilled(ImVec2(kx + font * 0.35F, ky), font * 0.32F, IM_COL32(4, 5, 8, 255));
        draw->AddCircleFilled(ImVec2(kx + font * 0.35F, ky), font * 0.25F, fill);
        kx += font * 0.9F;
        draw->AddText(ImVec2(kx, ky - font * 0.5F), muted, label);
        kx += ImGui::CalcTextSize(label).x;
    }
    ImGui::End();
}

} // namespace orrery::app
