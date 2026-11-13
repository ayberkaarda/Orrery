#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <fstream>
#include <limits>
#include <numbers>
#include <system_error>
#include <utility>

#include <imgui.h>
#include <imgui_internal.h>

#include "../aero/aero_widgets.h"
#include "entry_workspace_impl.h"
#include "orrery/render/entry_geometry.h"
#include "orrery/sim/aero_comparison.h"

namespace orrery::app {

namespace w = widgets;

namespace {

constexpr double kDragRadiansPerPixel = 0.005;
constexpr double kZoomPerNotch = 0.15;
// Largest number of track points sent to the view.
constexpr std::size_t kMaxViewPoints = 4000;

render::BodyFrameLookup no_bodies() {
    return [](core::BodyId) -> std::optional<render::BodyFrame> { return std::nullopt; };
}

const char* state_label(sim::EntryRunState state) {
    switch (state) {
        case sim::EntryRunState::idle:
            return "Ready";
        case sim::EntryRunState::running:
            return "Running";
        case sim::EntryRunState::paused:
            return "Paused";
        case sim::EntryRunState::finished:
            return "Finished";
        case sim::EntryRunState::stopped:
            return "Stopped";
        case sim::EntryRunState::failed:
            return "Failed";
    }
    return "";
}

// Linear-light value of an sRGB theme colour (the view's colours are linear).
glm::dvec3 linear_of(const render::UiColor& c) {
    const auto channel = [](float v) {
        const auto s = static_cast<double>(v);
        return s <= 0.04045 ? s / 12.92 : std::pow((s + 0.055) / 1.055, 2.4);
    };
    return {channel(c.r), channel(c.g), channel(c.b)};
}

std::uint64_t mix(std::uint64_t h, std::uint64_t v) noexcept {
    return (h ^ v) * 0x100000001b3ULL;
}

} // namespace

bool same_entry_inputs(const sim::EntrySetup& a, const sim::EntrySetup& b) {
    sim::EntrySetup named = b;
    named.name = a.name;
    return a == named;
}

EntryQuantity entry_quantity(EntryColoring coloring) noexcept {
    if (coloring == EntryColoring::heat_flux) {
        return {"Stagnation heat flux", "kW/m\xC2\xB2", 1.0e-3};
    }
    return {"g-load", "g", 1.0};
}

const std::vector<double>& coloring_values(const sim::EntrySeries& series,
                                           EntryColoring coloring) noexcept {
    return coloring == EntryColoring::heat_flux ? series.heat_flux_w_m2 : series.g_load;
}

render::ScalarRange coloring_range(const sim::EntrySeries& series, EntryColoring coloring) {
    const double scale = entry_quantity(coloring).scale;
    std::vector<double> values{0.0};
    for (const double v : coloring_values(series, coloring)) {
        values.push_back(v * scale);
    }
    render::ColorScale automatic;
    automatic.mode = render::RangeMode::automatic;
    render::ScalarRange range = render::resolve_range(automatic, values);
    range.min = 0.0;
    return range;
}

EntryWorkspace::Impl::Impl(EntryWorkspaceOptions o) : options(std::move(o)) {
    pace = std::max(options.pace, 0.0);
    runner.send(sim::SetEntryPace{pace});
    const std::vector<sim::EntryPreset> presets = sim::entry_presets();
    if (!presets.empty()) {
        setup = presets.front().setup;
    }
    w::set_buffer(name_buffer, setup.name);
    render::CameraLens lens;
    lens.vertical_fov = 32.0 * std::numbers::pi / 180.0;
    lens.near_plane = 100.0;
    lens.log_depth_far = 1.0e10;
    (void)camera.set_lens(lens);
}

// --- State -------------------------------------------------------------------

bool EntryWorkspace::Impl::active() const {
    const sim::EntryRunState state = runner.latest().status.state;
    return state == sim::EntryRunState::running || state == sim::EntryRunState::paused;
}

std::shared_ptr<const sim::EntryTrace> EntryWorkspace::Impl::shown() const {
    const sim::EntryProgress& p = runner.latest();
    // The workspace's own run: always while it is active, afterwards only
    // while the inputs are still the ones it ran with.
    if (p.trace && run_setup && p.status.run == run_number &&
        (active() || same_entry_inputs(*run_setup, setup))) {
        return p.trace;
    }
    if (loaded_trace && loaded_setup && same_entry_inputs(*loaded_setup, setup)) {
        return loaded_trace;
    }
    return nullptr;
}

const sim::EntrySetup& EntryWorkspace::Impl::shown_setup() const {
    const std::shared_ptr<const sim::EntryTrace> trace = shown();
    if (trace != nullptr && trace == runner.latest().trace && run_setup) {
        return *run_setup;
    }
    if (trace != nullptr && trace == loaded_trace && loaded_setup) {
        return *loaded_setup;
    }
    return setup;
}

bool EntryWorkspace::Impl::shows_record() const {
    const std::shared_ptr<const sim::EntryTrace> trace = shown();
    return trace != nullptr && trace == loaded_trace && loaded_is_record;
}

std::optional<double> EntryWorkspace::Impl::cursor() const {
    if (hover_time) {
        return hover_time;
    }
    if (pinned_time) {
        return pinned_time;
    }
    const std::shared_ptr<const sim::EntryTrace> trace = shown();
    if (trace && !trace->series.empty()) {
        if (active()) {
            return trace->series.time_s.back();
        }
        return trace->summary.peak_g_time_s;
    }
    return std::nullopt;
}

std::uint64_t EntryWorkspace::Impl::view_key() const {
    const std::shared_ptr<const sim::EntryTrace> trace = shown();
    std::uint64_t key = 0xcbf29ce484222325ULL;
    key = mix(key, reinterpret_cast<std::uintptr_t>(trace.get()));
    key = mix(key, static_cast<std::uint64_t>(shown_setup().planet));
    // The exaggeration is clamped to the slider's range before the cast.
    const double scale =
        std::isfinite(layers.altitude_scale) ? std::clamp(layers.altitude_scale, 1.0, 1000.0) : 1.0;
    key = mix(key, static_cast<std::uint64_t>(std::llround(scale * 100.0)));
    return key;
}

bool EntryWorkspace::Impl::start_run() {
    setup.name = w::from_buffer(name_buffer);
    const std::vector<std::string> errors = sim::setup_errors(setup);
    if (!errors.empty()) {
        set_notice("Cannot run: " + errors.front(), true);
        return false;
    }
    run_setup = std::make_shared<const sim::EntrySetup>(setup);
    // The runner numbers its runs from 1 in the order they were sent.
    run_number = ++starts_sent;
    runner.send(sim::StartEntryRun{setup});
    pinned_time.reset();
    reframe_pending = true;
    return true;
}

void EntryWorkspace::Impl::replace_setup(const sim::EntrySetup& inputs) {
    setup = inputs;
    w::set_buffer(name_buffer, setup.name);
    ++inputs_generation;
}

void EntryWorkspace::Impl::open(sim::StoredEntry entry) {
    runner.send(sim::StopEntryRun{});
    run_setup.reset();
    setup = std::move(entry.setup);
    ++inputs_generation;
    w::set_buffer(name_buffer, setup.name);
    loaded_trace = std::move(entry.result);
    loaded_setup.reset();
    if (loaded_trace) {
        loaded_setup = std::make_shared<const sim::EntrySetup>(setup);
    }
    pinned_time.reset();
    cd_label.clear();
    cd_warnings.clear();
    reframe_pending = true;
}

std::optional<std::filesystem::path> EntryWorkspace::Impl::save() {
    setup.name = w::from_buffer(name_buffer);
    const std::shared_ptr<const sim::EntryTrace> trace = shown();
    // A run still in progress is not a result yet: it has no result record.
    const bool with_result = trace != nullptr && trace->record != nullptr;
    const std::string name = setup.name.empty() ? std::string("entry") : setup.name;
    const std::filesystem::path path = sim::new_experiment_path(options.experiment_dir, name);
    auto saved = sim::save_entry_experiment_file(path, setup, with_result ? trace.get() : nullptr);
    if (!saved) {
        set_notice("Save failed: " + saved.error(), true);
        return std::nullopt;
    }
    set_notice("Saved " + saved.value().filename().string() +
                   (with_result ? "" : " (inputs only: no finished run of these inputs)"),
               false);
    listing_dirty = true;
    return saved.value();
}

void EntryWorkspace::Impl::refresh_listing() {
    listing = sim::list_entry_experiments(options.experiment_dir);
    compared.assign(listing.entries.size(), 0);
    listing_dirty = false;
}

std::vector<sim::EntryComparisonInput> EntryWorkspace::Impl::comparison_inputs() const {
    std::vector<sim::EntryComparisonInput> inputs;
    const bool any = std::any_of(compared.begin(), compared.end(), [](char c) { return c != 0; });
    for (std::size_t i = 0; i < listing.entries.size(); ++i) {
        const sim::StoredEntry& e = listing.entries[i];
        if (!e.result || (any && (i >= compared.size() || compared[i] == 0))) {
            continue;
        }
        const std::string name = e.setup.name.empty() ? e.path.stem().string() : e.setup.name;
        inputs.push_back({name, e.setup, e.result->summary});
    }
    return inputs;
}

std::optional<std::filesystem::path> EntryWorkspace::Impl::export_comparison() {
    const std::vector<sim::EntryComparisonInput> inputs = comparison_inputs();
    if (inputs.empty()) {
        set_notice("Nothing to export: save runs first", true);
        return std::nullopt;
    }
    const std::vector<sim::EntryComparisonRow> rows = sim::compare_entries(inputs);
    std::error_code error;
    std::filesystem::create_directories(options.experiment_dir, error);
    std::filesystem::path csv_path =
        sim::new_experiment_path(options.experiment_dir, "entry comparison")
            .replace_extension(".csv");
    std::ofstream out(csv_path, std::ios::binary | std::ios::trunc);
    const std::string csv = sim::entry_comparison_csv(rows);
    out.write(csv.data(), static_cast<std::streamsize>(csv.size()));
    out.close();
    if (!out) {
        set_notice("Export failed: cannot write " + csv_path.string(), true);
        return std::nullopt;
    }
    set_notice("Exported " + csv_path.string(), false);
    return csv_path;
}

void EntryWorkspace::Impl::set_notice(std::string message, bool is_error) {
    notice = std::move(message);
    notice_is_error = is_error;
}

void EntryWorkspace::Impl::show_problem(std::string title, std::string detail) {
    modal = std::make_pair(std::move(title), std::move(detail));
    modal_requested = true;
}

void EntryWorkspace::Impl::start_cd_sampling() {
    sim::CdSampling sampling;
    sampling.body = cd_body;
    sampling.reference_area_m2 = setup.reference_area_m2;
    cd_generation = inputs_generation;
    cd_area_m2 = setup.reference_area_m2;
    cd_task = sim::BackgroundTask<CdTaskResult>(
        [sampling, sampler = options.cd_sampler](sim::TaskProgress& progress) -> CdTaskResult {
            progress.set_stage("Sampling Cd(Mach)");
            return sampler(sampling);
        },
        [](std::string message) -> CdTaskResult { return core::failure(std::move(message)); });
}

void EntryWorkspace::Impl::poll_cd_task() {
    if (!cd_task.done()) {
        return;
    }
    CdTaskResult result = cd_task.take();
    if (cd_generation != inputs_generation || cd_area_m2 != setup.reference_area_m2) {
        // Other inputs were loaded, or the area the coefficients are
        // normalised by changed, while it ran: the table belongs to none of
        // the current inputs.
        set_notice("Cd(Mach) sampling discarded: the inputs changed while it ran", true);
        return;
    }
    if (!result) {
        show_problem("Cd(Mach) sampling failed", result.error());
        return;
    }
    sim::CdSamples samples = std::move(result).value();
    setup.cd_table = std::move(samples.table);
    setup.cd_source = sim::CdSource::aero_solver;
    cd_label = std::move(samples.label);
    cd_warnings = std::move(samples.warnings);
}

void EntryWorkspace::Impl::reframe() {
    if (camera_width <= 0 || camera_height <= 0) {
        return;
    }
    const EntryViewState state = view_state();
    const render::EntryFrame frame = render::make_entry_frame(state.track, state.globe);
    // Frame the track in the part of the scene the overlay leaves free: the
    // outcome banner on top, the legend at the bottom.
    const double top = std::min(40.0, 0.08 * camera_scene.height);
    const double bottom = std::min(80.0, 0.15 * camera_scene.height);
    const render::UiRect area{camera_scene.x, camera_scene.y + top, camera_scene.width,
                              camera_scene.height - top - bottom};
    const bool usable = area.width > 1.0 && area.height > 1.0;
    const double aspect =
        usable ? area.width / area.height : static_cast<double>(camera_width) / camera_height;
    render::TurntableMode mode = render::frame_entry(frame, aspect, camera.lens());
    if (usable) {
        mode.distance *= std::max(1.0, static_cast<double>(camera_height) / area.height);
        const double per_pixel =
            2.0 * mode.distance * std::tan(0.5 * camera.lens().vertical_fov) / camera_height;
        const double shift_x = (area.x + 0.5 * area.width - 0.5 * camera_width) * per_pixel;
        const double shift_up = (0.5 * camera_height - (area.y + 0.5 * area.height)) * per_pixel;
        const glm::dvec3 right{-std::sin(mode.yaw), std::cos(mode.yaw), 0.0};
        const glm::dvec3 up{-std::cos(mode.yaw) * std::sin(mode.pitch),
                            -std::sin(mode.yaw) * std::sin(mode.pitch), std::cos(mode.pitch)};
        mode.focus -= right * shift_x + up * shift_up;
    }
    camera.set_mode(mode);
    (void)camera.update(0.0, render::CameraInput{}, no_bodies());
    framed_key = view_key();
}

// --- Layout ------------------------------------------------------------------

render::UiRect EntryWorkspace::Impl::dockspace() {
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    const ImGuiID dock_id = ImGui::GetID("OrreryEntryDockspace");
    if (!layout_built) {
        layout_built = true;
        ImGui::DockBuilderRemoveNode(dock_id);
        ImGui::DockBuilderAddNode(dock_id,
                                  static_cast<ImGuiDockNodeFlags>(ImGuiDockNodeFlags_DockSpace) |
                                      ImGuiDockNodeFlags_PassthruCentralNode);
        ImGui::DockBuilderSetNodePos(dock_id, viewport->WorkPos);
        ImGui::DockBuilderSetNodeSize(dock_id, viewport->WorkSize);
        ImGuiID rest = 0;
        const ImGuiID left =
            ImGui::DockBuilderSplitNode(dock_id, ImGuiDir_Left, 0.22F, nullptr, &rest);
        const ImGuiID right =
            ImGui::DockBuilderSplitNode(rest, ImGuiDir_Right, 0.31F, nullptr, &rest);
        ImGuiID centre = 0;
        const ImGuiID bottom =
            ImGui::DockBuilderSplitNode(rest, ImGuiDir_Down, 0.36F, nullptr, &centre);
        ImGui::DockBuilderDockWindow(std::string(kEntrySetupWindow).c_str(), left);
        ImGui::DockBuilderDockWindow(std::string(kEntryProfileWindow).c_str(), right);
        for (const std::string_view name :
             {kGroundTrackWindow, kEntryExperimentsWindow, kEntryViewWindow}) {
            ImGui::DockBuilderDockWindow(std::string(name).c_str(), bottom);
        }
        for (const ImGuiID id : {left, right, bottom}) {
            if (ImGuiDockNode* node = ImGui::DockBuilderGetNode(id)) {
                node->SetLocalFlags(node->LocalFlags | ImGuiDockNodeFlags_NoWindowMenuButton);
            }
        }
        ImGui::DockBuilderFinish(dock_id);
    }
    ImGui::DockSpaceOverViewport(dock_id, viewport, ImGuiDockNodeFlags_PassthruCentralNode);
    render::UiRect area{
        static_cast<double>(viewport->WorkPos.x), static_cast<double>(viewport->WorkPos.y),
        static_cast<double>(viewport->WorkSize.x), static_cast<double>(viewport->WorkSize.y)};
    if (const ImGuiDockNode* central = ImGui::DockBuilderGetCentralNode(dock_id)) {
        area = {static_cast<double>(central->Pos.x), static_cast<double>(central->Pos.y),
                static_cast<double>(central->Size.x), static_cast<double>(central->Size.y)};
    }
    return area;
}

void EntryWorkspace::Impl::draw_modal() {
    const std::string id(kEntryErrorModal);
    if (modal_requested) {
        ImGui::OpenPopup(id.c_str());
        modal_requested = false;
    }
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5F, 0.5F));
    ImGui::SetNextWindowSizeConstraints(ImVec2(ImGui::GetFontSize() * 24.0F, 0.0F),
                                        ImVec2(ImGui::GetFontSize() * 42.0F, FLT_MAX));
    if (ImGui::BeginPopupModal(id.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (modal) {
            ImGui::PushStyleColor(ImGuiCol_Text, w::color(w::theme().warning));
            ImGui::TextUnformatted(modal->first.c_str());
            ImGui::PopStyleColor();
            ImGui::Separator();
            w::wrapped(modal->second, ImGui::GetStyleColorVec4(ImGuiCol_Text));
            ImGui::Dummy(ImVec2(0.0F, 4.0F));
        }
        if (ImGui::Button("Copy details")) {
            ImGui::SetClipboardText(modal ? (modal->first + "\n" + modal->second).c_str() : "");
        }
        ImGui::SameLine();
        if (ImGui::Button("Close") || ImGui::IsKeyPressed(ImGuiKey_Escape)) {
            modal.reset();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

// --- Main bar ------------------------------------------------------------------

namespace {

constexpr std::array<double, 6> kPaces{10.0, 30.0, 100.0, 300.0, 1000.0, 0.0};

std::string pace_label(double pace) {
    if (pace <= 0.0) {
        return "as fast as possible";
    }
    return w::fixed(pace, 0) + "x real time";
}

} // namespace

void EntryWorkspace::Impl::draw_bar(double fps) {
    const ImGuiStyle& base = ImGui::GetStyle();
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,
                        ImVec2(base.FramePadding.x, base.FramePadding.y + 3.0F));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(6.0F, base.ItemSpacing.y));
    if (!ImGui::BeginMainMenuBar()) {
        ImGui::PopStyleVar(2);
        return;
    }
    const float gap = ImGui::GetStyle().ItemSpacing.x * 2.0F;
    const bool compact = ImGui::GetWindowWidth() < ImGui::GetFontSize() * 80.0F;
    w::word_mark();
    ImGui::Dummy(ImVec2(gap, 0.0F));
    w::workspace_label(std::string(kEntryWorkspace).c_str(),
                       "Entry workspace: atmospheric entry of a probe");
    ImGui::Dummy(ImVec2(gap * 0.5F, 0.0F));
    ImGui::Separator();

    const sim::EntryProgress& p = runner.latest();
    const sim::EntryRunState state = p.status.state;
    if (state == sim::EntryRunState::running) {
        if (w::icon_button("##pause", w::Glyph::pause, "Pause the entry (Space)", false)) {
            runner.send(sim::SetEntryPaused{true});
        }
    } else if (state == sim::EntryRunState::paused) {
        if (w::icon_button("##resume", w::Glyph::play, "Resume the entry (Space)", true)) {
            runner.send(sim::SetEntryPaused{false});
        }
    } else if (w::icon_button("##run", w::Glyph::play, "Run the entry with these inputs (Space)",
                              true, sim::setup_errors(setup).empty())) {
        (void)start_run();
    }
    if (w::icon_button("##stop", w::Glyph::stop, "Stop the entry (what was flown stays shown)",
                       false, active())) {
        runner.send(sim::StopEntryRun{});
    }
    ImGui::Separator();
    if (!compact) {
        w::muted("PACE");
    }
    if (ImGui::Button((pace_label(pace) + "##pace").c_str())) {
        ImGui::OpenPopup("##pace_menu");
    }
    w::tooltip("Simulated seconds per second of wall-clock time");
    if (ImGui::BeginPopup("##pace_menu")) {
        for (const double option : kPaces) {
            if (ImGui::Selectable(pace_label(option).c_str(), option == pace)) {
                pace = option;
                runner.send(sim::SetEntryPace{pace});
            }
        }
        ImGui::EndPopup();
    }
    ImGui::Separator();
    if (!compact) {
        w::muted("PLANET");
    }
    w::text(std::string(sim::planet_constants(setup.planet).name));

    // Right-aligned status and frame rate.
    std::string status = state_label(state);
    const std::shared_ptr<const sim::EntryTrace> trace = shown();
    if (active() && trace && !trace->series.empty()) {
        status += std::string(" \xC2\xB7 t ") + w::fixed(trace->series.time_s.back(), 0) +
                  " s \xC2\xB7 " + w::fixed(trace->series.g_load.back(), 1) + " g";
    } else if (trace && trace->summary.outcome != sim::EntryOutcome::none) {
        status = std::string(sim::outcome_label(trace->summary.outcome)) + " \xC2\xB7 peak " +
                 w::fixed(trace->summary.peak_g, 2) + " g";
    }
    if (cd_task.started()) {
        status = std::string(cd_task.stage()) + "...";
    }
    const std::string fps_text = w::fixed(fps, 0) + " fps";
    const float status_width = ImGui::CalcTextSize(status.c_str()).x;
    const float fps_width = ImGui::CalcTextSize(fps_text.c_str()).x + gap;
    const float start = ImGui::GetWindowContentRegionMax().x - status_width - fps_width;
    if (start > ImGui::GetCursorPosX()) {
        ImGui::SetCursorPosX(start);
    }
    if (state == sim::EntryRunState::failed) {
        ImGui::PushStyleColor(ImGuiCol_Text, w::color(w::theme().warning));
        ImGui::TextUnformatted(status.c_str());
        ImGui::PopStyleColor();
        w::tooltip(p.status.message.c_str());
    } else {
        w::text(status);
    }
    ImGui::Dummy(ImVec2(gap * 0.5F, 0.0F));
    w::muted(fps_text);
    ImGui::EndMainMenuBar();
    ImGui::PopStyleVar(2);
}

// --- Public interface ----------------------------------------------------------

EntryWorkspace::EntryWorkspace(EntryWorkspaceOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}

EntryWorkspace::~EntryWorkspace() = default;

void EntryWorkspace::open(sim::StoredEntry entry) {
    impl_->open(std::move(entry));
    impl_->loaded_is_record = true;
}

void EntryWorkspace::present(sim::StoredEntry entry) {
    impl_->open(std::move(entry));
    impl_->loaded_is_record = false;
}

void EntryWorkspace::update(double dt) {
    Impl& s = *impl_;
    s.poll_cd_task();
    (void)s.runner.fetch();
    s.runner.wake();
    (void)s.camera.update(dt, s.camera_input, no_bodies());
    s.camera_input = render::CameraInput{};
}

void EntryWorkspace::draw_bar(double fps) {
    impl_->draw_bar(fps);
}

render::UiRect EntryWorkspace::draw_panels() {
    Impl& s = *impl_;
    s.hover_time.reset();
    const render::UiRect scene = s.dockspace();
    s.draw_setup_panel();
    s.draw_profile_panel();
    // The ground track is the tab shown first in the bottom panel.
    if (s.layout_frames < 2) {
        ImGui::SetNextWindowFocus();
        ++s.layout_frames;
    }
    s.draw_ground_track_panel();
    s.draw_experiments_panel();
    s.draw_view_panel();
    s.draw_scene_overlay(scene);
    s.draw_modal();
    return scene;
}

void EntryWorkspace::handle_scene_input(const render::SceneInput& input, double dt) {
    Impl& s = *impl_;
    if (input.toggle_pause) {
        const sim::EntryRunState state = s.runner.latest().status.state;
        if (state == sim::EntryRunState::running) {
            s.runner.send(sim::SetEntryPaused{true});
        } else if (state == sim::EntryRunState::paused) {
            s.runner.send(sim::SetEntryPaused{false});
        } else {
            (void)s.start_run();
        }
    }
    if (input.reset_view) {
        s.reframe_pending = true;
    }
    if (dt > 0.0) {
        s.camera_input.yaw = -input.drag_x * kDragRadiansPerPixel / dt;
        s.camera_input.pitch = input.drag_y * kDragRadiansPerPixel / dt;
        s.camera_input.zoom = input.wheel * kZoomPerNotch / dt;
    }
}

const render::Camera& EntryWorkspace::camera(const render::UiRect& scene, int width, int height) {
    Impl& s = *impl_;
    const bool moved = std::abs(scene.x - s.camera_scene.x) > 0.5 ||
                       std::abs(scene.y - s.camera_scene.y) > 0.5 ||
                       std::abs(scene.width - s.camera_scene.width) > 0.5 ||
                       std::abs(scene.height - s.camera_scene.height) > 0.5 ||
                       width != s.camera_width || height != s.camera_height;
    // A running entry grows its track: frame it again when it finishes, not
    // on every published slice.
    const bool new_view = s.framed_key != s.view_key() && !s.active();
    if (s.reframe_pending || moved || new_view) {
        s.reframe_pending = false;
        s.camera_scene = scene;
        s.camera_width = width;
        s.camera_height = height;
        s.reframe();
    }
    return s.camera;
}

EntryViewState EntryWorkspace::view_state() const {
    return impl_->view_state();
}

EntryViewState EntryWorkspace::Impl::view_state() const {
    const Impl& s = *this;
    EntryViewState state;
    // The planet the shown run flew around, not the one the inputs name now
    // (a paused run stays shown while the inputs change).
    const sim::EntrySetup& inputs = s.shown_setup();
    state.planet = inputs.planet;
    const sim::PlanetConstants planet = sim::planet_constants(inputs.planet);
    state.globe.radius_m = planet.radius_m;
    state.globe.atmosphere_top_m =
        inputs.integration.interface_altitude_m.value_or(planet.atmosphere_top_m);
    state.globe.altitude_scale = std::max(1.0, s.layers.altitude_scale);
    state.trace = s.shown();
    // The uploaded ribbon carries the coloured quantity too; the framing key
    // leaves it out so that switching quantities does not move the camera.
    state.revision = mix(s.view_key(), static_cast<std::uint64_t>(s.layers.coloring) + 1U);
    render::EntryViewOptions& o = state.options;
    o.style = inputs.planet == sim::EntryPlanet::mars ? render::EntryPlanetStyle::mars
                                                      : render::EntryPlanetStyle::earth;
    o.atmosphere = s.layers.atmosphere;
    o.graticule = s.layers.graticule;
    o.ground_track = s.layers.ground_track;
    o.curtain = s.layers.curtain;
    if (!state.trace || state.trace->series.empty() || !state.trace->series.consistent()) {
        return state;
    }
    const sim::EntrySeries& series = state.trace->series;
    const EntryQuantity quantity = entry_quantity(s.layers.coloring);
    const std::vector<double>& values = coloring_values(series, s.layers.coloring);
    std::vector<render::EntryTrackPoint> track;
    track.reserve(series.size());
    for (std::size_t i = 0; i < series.size(); ++i) {
        track.push_back({series.latitude_deg[i], series.longitude_deg[i], series.altitude_m[i],
                         values[i] * quantity.scale});
    }
    state.track = render::decimate_track(track, kMaxViewPoints);
    o.range = coloring_range(series, s.layers.coloring);
    // Markers: the peak of the shown quantity, then the cursor on top.
    const double peak_time = s.layers.coloring == EntryColoring::heat_flux
                                 ? state.trace->summary.peak_heat_flux_time_s
                                 : state.trace->summary.peak_g_time_s;
    const auto add_marker = [&](double t, const glm::dvec3& color, double radius) {
        const auto i = sim::nearest_index(series.time_s, t);
        if (!i) {
            return;
        }
        o.markers.push_back({series.latitude_deg[*i], series.longitude_deg[*i],
                             series.altitude_m[*i], color, radius});
    };
    // The peak is a larger disc, so the cursor dot stays visible on it.
    add_marker(peak_time, linear_of(w::theme().accent), 8.5);
    if (const std::optional<double> t = s.cursor()) {
        add_marker(*t, glm::dvec3(0.95, 0.96, 0.98), 4.5);
    }
    return state;
}

const sim::EntryProgress& EntryWorkspace::progress() const noexcept {
    return impl_->runner.latest();
}

const sim::EntrySetup& EntryWorkspace::setup() const noexcept {
    return impl_->setup;
}

void EntryWorkspace::set_setup(const sim::EntrySetup& setup) {
    impl_->replace_setup(setup);
}

std::shared_ptr<const sim::EntryTrace> EntryWorkspace::shown_trace() const {
    return impl_->shown();
}

bool EntryWorkspace::shows_record() const {
    return impl_->shows_record();
}

std::optional<double> EntryWorkspace::cursor_time() const {
    return impl_->cursor();
}

void EntryWorkspace::set_cursor_time(std::optional<double> t) {
    impl_->pinned_time = t;
}

bool EntryWorkspace::start_run() {
    return impl_->start_run();
}

void EntryWorkspace::pause(bool paused) {
    impl_->runner.send(sim::SetEntryPaused{paused});
}

void EntryWorkspace::stop() {
    impl_->runner.send(sim::StopEntryRun{});
}

void EntryWorkspace::set_pace(double pace) {
    impl_->pace = std::max(pace, 0.0);
    impl_->runner.send(sim::SetEntryPace{impl_->pace});
}

std::optional<std::filesystem::path> EntryWorkspace::save() {
    return impl_->save();
}

void EntryWorkspace::refresh_experiments() {
    impl_->refresh_listing();
}

const sim::EntryListing& EntryWorkspace::experiments() const noexcept {
    return impl_->listing;
}

void EntryWorkspace::set_compared(std::size_t index, bool compared) {
    if (index < impl_->compared.size()) {
        impl_->compared[index] = compared ? 1 : 0;
    }
}

std::optional<std::filesystem::path> EntryWorkspace::export_comparison() {
    return impl_->export_comparison();
}

void EntryWorkspace::sample_cd(sim::CdBody body) {
    impl_->cd_body = body;
    impl_->start_cd_sampling();
}

bool EntryWorkspace::sampling_cd() const noexcept {
    return impl_->cd_task.started();
}

EntryLayers& EntryWorkspace::layers() noexcept {
    return impl_->layers;
}

void EntryWorkspace::set_view_problem(std::string problem) {
    impl_->view_problem = std::move(problem);
}

const std::string& EntryWorkspace::notice() const noexcept {
    return impl_->notice;
}

} // namespace orrery::app
