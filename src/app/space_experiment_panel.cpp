#include "orrery/app/space_experiment_panel.h"

#include <array>
#include <utility>

#include <imgui.h>

#include "aero/aero_widgets.h"
#include "orrery/render/ui_app_bar.h"
#include "orrery/sim/aero_comparison.h"

namespace orrery::app {

namespace w = widgets;

namespace {

constexpr std::size_t kNameLength = 64;

// One listed experiment: a selectable row with the summary in muted text;
// a double click opens it.
bool list_row(const sim::StoredSpace& entry, bool selected, bool& opened) {
    ImGui::PushID(entry.path.string().c_str());
    const bool clicked =
        ImGui::Selectable(entry.name.c_str(), selected, ImGuiSelectableFlags_AllowDoubleClick);
    if (clicked && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        opened = true;
    }
    ImGui::SameLine();
    w::muted(entry.summary);
    ImGui::PopID();
    return clicked;
}

} // namespace

SpaceExperimentPanel::SpaceExperimentPanel(std::filesystem::path experiment_dir,
                                           std::filesystem::path bundled_dir)
    : experiment_dir_(std::move(experiment_dir)), bundled_dir_(std::move(bundled_dir)) {
    refresh();
}

void SpaceExperimentPanel::refresh() {
    saved_ = sim::list_space_experiments(experiment_dir_);
    bundled_ =
        bundled_dir_.empty() ? sim::SpaceListing{} : sim::list_space_experiments(bundled_dir_);
}

std::optional<std::filesystem::path>
SpaceExperimentPanel::save(const std::filesystem::path& scenario, const sim::SimStatus& status,
                           double sim_time_days) {
    // A new file each time, like the other workspaces: "<name>_2.json" when
    // the name is taken.
    const std::filesystem::path file = sim::new_experiment_path(experiment_dir_, name_);
    const scene::Experiment record =
        sim::space_record(scenario, status, sim_time_days, experiment_dir_);
    auto written = sim::save_space_experiment_file(file, record);
    if (!written) {
        notice_ = "Cannot save: " + written.error();
        notice_is_error_ = true;
        return std::nullopt;
    }
    notice_ = "Saved " + file.filename().string();
    notice_is_error_ = false;
    refresh();
    return written.value();
}

void SpaceExperimentPanel::draw_run(const SpaceRunView& run, SpaceExperimentIntent& intent) {
    w::section("RUNNING");
    if (w::begin_properties("##run", "Experiment")) {
        w::property("Scenario", run.scenario.empty() ? std::string("-") : run.scenario);
        w::property("Experiment", run.experiment.empty() ? std::string("-") : run.experiment);
        w::property("Integrator", run.integrator + ", dt " + w::significant(run.dt_days) + " d");
        w::property("Forces", run.force_model.empty() ? std::string("-") : run.force_model);
        w::property("Warp", render::format_time_warp(static_cast<double>(run.time_warp)));
        w::end_properties();
    }

    w::section("SAVE");
    std::array<char, kNameLength> buffer{};
    w::set_buffer(buffer, name_);
    w::row_label("Name");
    if (ImGui::InputTextWithHint("##space_name", "experiment name", buffer.data(), buffer.size())) {
        name_ = w::from_buffer(buffer);
    }
    if (ImGui::Button("Save")) {
        intent.save = true;
    }
    std::string tooltip = "Writes the scenario, the integrator settings, the warp and the run so "
                          "far (time, steps, energy drift) as a new file (Ctrl+S).\nFolder: ";
    tooltip += experiment_dir_.string();
    w::tooltip(tooltip.c_str());
    if (!notice_.empty()) {
        ImGui::SameLine();
        if (notice_is_error_) {
            w::error(notice_);
        } else {
            w::wrapped(notice_, w::color(w::theme().accent_secondary));
        }
    }
}

void SpaceExperimentPanel::draw_open(SpaceExperimentIntent& intent) {
    w::section("OPEN");
    bool opened = false;
    const auto list = [&](const sim::SpaceListing& listing, const char* caption,
                          const char* empty) {
        w::muted(caption);
        if (listing.entries.empty()) {
            ImGui::SameLine();
            w::muted(empty);
        }
        for (const sim::StoredSpace& entry : listing.entries) {
            if (list_row(entry, selected_ == entry.path, opened)) {
                selected_ = entry.path;
            }
        }
        for (const std::string& problem : listing.problems) {
            w::warning(problem);
        }
    };
    // The lists scroll; the buttons stay below them.
    const float buttons = ImGui::GetFrameHeightWithSpacing();
    if (ImGui::BeginChild("##experiment_lists", ImVec2(0.0F, -buttons))) {
        list(saved_, "Saved", "- none yet");
        ImGui::Dummy(ImVec2(0.0F, 2.0F));
        list(bundled_, "Bundled", "- none found");
    }
    ImGui::EndChild();
    ImGui::BeginDisabled(!selected_);
    if (ImGui::Button("Open") || (opened && selected_)) {
        intent.open = selected_;
    }
    ImGui::EndDisabled();
    w::tooltip("Replaces the running system and its settings by the selected experiment "
               "(a double click opens a row at once).");
    ImGui::SameLine();
    if (ImGui::Button("Refresh")) {
        refresh();
    }
    w::tooltip("Reads the experiments folder again.");
}

SpaceExperimentIntent SpaceExperimentPanel::draw(const SpaceRunView& run) {
    SpaceExperimentIntent intent;
    if (!ImGui::Begin(std::string(kSpaceExperimentWindow).c_str())) {
        ImGui::End();
        return intent;
    }
    // Side by side in the wide bottom dock, one above the other when narrow.
    if (ImGui::GetContentRegionAvail().x >= ImGui::GetFontSize() * 44.0F &&
        ImGui::BeginTable("##experiment_layout", 2,
                          ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_BordersInnerV |
                              ImGuiTableFlags_PadOuterX,
                          ImVec2(0.0F, ImGui::GetContentRegionAvail().y))) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        draw_run(run, intent);
        ImGui::TableSetColumnIndex(1);
        draw_open(intent);
        ImGui::EndTable();
    } else {
        draw_run(run, intent);
        draw_open(intent);
    }
    ImGui::End();
    return intent;
}

} // namespace orrery::app
