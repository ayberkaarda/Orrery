#include "orrery/app/aero_workspace.h"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <numbers>
#include <span>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

#include <imgui.h>
#include <imgui_internal.h>
#include <implot.h>

#include "aero_widgets.h"
#include "orrery/app/aero_scene.h"
#include "orrery/app/config.h"
#include "orrery/geom/naca.h"
#include "orrery/geom/primitives.h"
#include "orrery/sim/aero_comparison.h"
#include "orrery/sim/aero_flow.h"
#include "orrery/sim/aero_import.h"
#include "orrery/sim/aero_solve.h"
#include "orrery/sim/background_task.h"

namespace orrery::app {

namespace w = widgets;

namespace {

constexpr double kDragRadiansPerPixel = 0.005;
constexpr double kZoomPerNotch = 0.15;
constexpr std::array<std::int64_t, 10> kStepsPerFrame{10,  20,   50,   100,  200,
                                                      500, 1000, 2000, 5000, 10000};
// UTF-8 of the Latin-1 symbols the default font covers.
constexpr const char* kSquared = "m\xC2\xB2";
constexpr const char* kCubed = "m\xC2\xB3";
constexpr const char* kDensity = "kg/m\xC2\xB3";
constexpr const char* kDegree = "\xC2\xB0";
constexpr const char* kDot = " \xC2\xB7 ";

struct DimensionLabel {
    std::string_view key;
    const char* label;
    const char* unit;
};

const char* dimension_label(scene::AeroShape shape, std::string_view key) {
    if (key == "length_m") {
        return shape == scene::AeroShape::cylinder ? "Span" : "Length";
    }
    constexpr std::array<DimensionLabel, 8> kLabels{{
        {"diameter_m", "Diameter", "m"},
        {"width_m", "Width", "m"},
        {"height_m", "Height", "m"},
        {"side_m", "Side", "m"},
        {"chord_m", "Chord", "m"},
        {"span_m", "Span", "m"},
        {"thickness_m", "Thickness", "m"},
        {"half_angle_deg", "Half angle", "\xC2\xB0"},
    }};
    for (const DimensionLabel& entry : kLabels) {
        if (entry.key == key) {
            return entry.label;
        }
    }
    return "Size";
}

double* dimension_field(scene::ShapeDimensions& d, std::string_view key) {
    if (key == "diameter_m") {
        return &d.diameter_m;
    }
    if (key == "length_m") {
        return &d.length_m;
    }
    if (key == "width_m") {
        return &d.width_m;
    }
    if (key == "height_m") {
        return &d.height_m;
    }
    if (key == "side_m") {
        return &d.side_m;
    }
    if (key == "chord_m") {
        return &d.chord_m;
    }
    if (key == "span_m") {
        return &d.span_m;
    }
    if (key == "thickness_m") {
        return &d.thickness_m;
    }
    return &d.half_angle_deg;
}

const char* area_label(geom::ReferenceAreaKind kind) {
    switch (kind) {
        case geom::ReferenceAreaKind::frontal:
            return "Frontal";
        case geom::ReferenceAreaKind::planform:
            return "Planform";
        case geom::ReferenceAreaKind::wetted:
            return "Wetted";
    }
    return "Frontal";
}

const char* state_label(sim::AeroRunState state) {
    switch (state) {
        case sim::AeroRunState::idle:
            return "Ready";
        case sim::AeroRunState::setting_up:
            return "Setting up";
        case sim::AeroRunState::running:
            return "Running";
        case sim::AeroRunState::paused:
            return "Paused";
        case sim::AeroRunState::finished:
            return "Finished";
        case sim::AeroRunState::failed:
            return "Failed";
    }
    return "";
}

bool same_inputs(const scene::AeroExperiment& a, const scene::AeroExperiment& b) {
    scene::AeroSolverSettings sa = a.solver;
    scene::AeroSolverSettings sb = b.solver;
    sa.steps_per_frame = 0;
    sb.steps_per_frame = 0;
    // The reference area only rescales the coefficients.
    sa.reference_area = sb.reference_area;
    return a.geometry == b.geometry && a.conditions == b.conditions && sa == sb;
}

// Animated ellipsis for running background work.
std::string dots() {
    const auto phase = static_cast<int>(ImGui::GetTime() * 3.0) % 4;
    std::string ellipsis(static_cast<std::size_t>(phase), '.');
    return ellipsis;
}

constexpr std::array<aero::SolverKind, 3> kSolverKinds{
    aero::SolverKind::panel, aero::SolverKind::newtonian, aero::SolverKind::lattice_boltzmann};

int solver_index(aero::SolverKind kind) noexcept {
    for (std::size_t i = 0; i < kSolverKinds.size(); ++i) {
        if (kSolverKinds[i] == kind) {
            return static_cast<int>(i);
        }
    }
    return 0;
}

// The solver combo: selection 0 is the recommendation, 1 + i an override
// with kSolverKinds[i]. Returns the entry picked this frame, or -1.
int solver_combo(int selection, aero::SolverKind recommended_kind, bool two_d) {
    const std::string recommended =
        "Recommended (" + std::string(sim::solver_display_name(recommended_kind)) + ")";
    const std::string preview = selection <= 0
                                    ? recommended
                                    : std::string(sim::solver_display_name(
                                          kSolverKinds[static_cast<std::size_t>(selection - 1)]));
    int picked = -1;
    if (!ImGui::BeginCombo("##solver", preview.c_str())) {
        return picked;
    }
    if (ImGui::Selectable(recommended.c_str(), selection == 0)) {
        picked = 0;
    }
    for (std::size_t i = 0; i < kSolverKinds.size(); ++i) {
        const bool available = sim::solver_available(kSolverKinds[i], two_d);
        ImGui::BeginDisabled(!available);
        if (ImGui::Selectable(std::string(sim::solver_display_name(kSolverKinds[i])).c_str(),
                              selection == static_cast<int>(i) + 1)) {
            picked = static_cast<int>(i) + 1;
        }
        ImGui::EndDisabled();
        if (!available) {
            w::tooltip("Needs a 3D surface mesh");
        }
    }
    ImGui::EndCombo();
    return picked;
}

// Rationale of the regime choice and the envelope warnings (never blocking).
void regime_notes(const sim::FlowSummary& flow) {
    w::wrapped(flow.regime.rationale, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    if (flow.warnings.empty()) {
        w::chip("WITHIN ENVELOPE", w::theme().accent_secondary,
                "These conditions are inside the documented validity envelope of the solver.");
        return;
    }
    for (const aero::ValidityWarning& warning : flow.warnings) {
        w::warning(warning.message);
    }
    w::muted("Warnings never block a run.");
}

// The aero scene has no bodies for the camera to follow.
render::BodyFrameLookup no_bodies() {
    return [](core::BodyId) -> std::optional<render::BodyFrame> { return std::nullopt; };
}

bool input_number(const char* id, double* value, const char* format) {
    (void)ImGui::InputDouble(id, value, 0.0, 0.0, format);
    return ImGui::IsItemDeactivatedAfterEdit();
}

} // namespace

struct AeroWorkspace::Impl {
    explicit Impl(AeroWorkspaceOptions o) : options(std::move(o)) {
        steps_per_frame = std::max<std::int64_t>(1, options.steps_per_frame);
        experiment.name = "NACA 0012 section";
        experiment.geometry.shape = scene::AeroShape::naca_section;
        experiment.conditions.speed_m_s = 30.0;
        experiment.conditions.angle_of_attack_deg = 4.0;
        experiment.solver.steps_per_frame = steps_per_frame;
        sync_buffers();
        rebuild_geometry();
    }

    // Members are grouped by size (large objects first, scalars and text
    // buffers last) so that the struct carries no padding to speak of.
    AeroWorkspaceOptions options;

    // Inputs.
    scene::AeroExperiment experiment;
    std::filesystem::path base_dir;

    // Geometry and background work.
    std::shared_ptr<const sim::PreparedGeometry> geometry;
    sim::BackgroundTask<sim::PrepareResult> geometry_task;
    sim::BackgroundTask<sim::ImportResult> import_task;
    std::optional<sim::ImportedModel> pending_import;
    // The URL model of an opened experiment that waits for the download
    // prompt's answer.
    std::optional<scene::GeometryRef> download_prompt;
    // Errors shown in the modal.
    std::optional<sim::GeometryError> modal;

    // Solver.
    sim::AeroRunner runner;
    // Inputs of the last run started and the record it left when finished.
    std::optional<scene::AeroExperiment> run_inputs;
    std::optional<scene::AeroExperiment> run_record;
    // A record loaded from a file, shown until a run replaces it, and the
    // inputs it was recorded with (their geometry becomes the prepared one
    // once the build requested by the load finishes).
    std::optional<scene::AeroResultRecord> loaded_result;
    std::optional<scene::AeroExperiment> loaded_inputs;

    // View.
    AeroLayers layers;
    render::Camera camera;
    render::UiRect camera_scene;
    render::CameraInput camera_input;

    // Experiments.
    sim::ExperimentListing listing;
    std::vector<char> compare; // per listed experiment
    std::string notice;
    std::string view_problem;

    // Scalars.
    std::uint64_t geometry_revision = 0;
    // Geometry requests (every change of the geometry inputs), the one the
    // running build serves and the one a loaded record asked for.
    std::uint64_t geometry_request = 0;
    std::uint64_t task_request = 0;
    std::uint64_t loaded_request = 0;
    std::uint64_t recorded_run = 0;
    std::int64_t steps_per_frame = 100;
    double pending_scale = 1.0;
    int source = 0; // 0 primitive, 1 file, 2 URL
    int pending_unit = 0;
    int pending_up = 0; // 0 Z up, 1 Y up
    int camera_width = 0;
    int camera_height = 0;
    bool source_tab_pending = true;
    bool rebuild_queued = false;
    bool modal_requested = false;
    bool download_prompt_requested = false;
    bool to_convergence = false;
    bool reframe_pending = true;
    bool spin = false;
    bool listing_dirty = true;
    bool notice_is_error = false;
    bool layout_built = false;

    // Text fields.
    std::array<char, 1024> path_buffer{};
    std::array<char, 2048> url_buffer{};
    std::array<char, 512> license_buffer{};
    std::array<char, 160> name_buffer{};
    std::array<char, 8> naca_buffer{};

    // --- Inputs and background work ---------------------------------------

    void sync_buffers() {
        source_tab_pending = true;
        w::set_buffer(name_buffer, experiment.name);
        w::set_buffer(naca_buffer, experiment.geometry.naca);
        w::set_buffer(license_buffer, experiment.geometry.license_note);
        if (experiment.geometry.source == scene::GeometrySource::file) {
            source = 1;
            w::set_buffer(path_buffer, experiment.geometry.path);
        } else if (experiment.geometry.source == scene::GeometrySource::url) {
            source = 2;
            w::set_buffer(url_buffer, experiment.geometry.url);
        } else {
            source = 0;
        }
    }

    void show_error(sim::GeometryError error) {
        modal = std::move(error);
        modal_requested = true;
    }

    // A new request for the geometry of the current inputs.
    void rebuild_geometry() {
        ++geometry_request;
        if (geometry_task.started()) {
            // Never wait for the running build (joining it would stall the
            // frame): its result is dropped and the build starts again.
            rebuild_queued = true;
            return;
        }
        start_geometry_build();
    }

    // Builds the geometry of the current inputs; a URL model that would have
    // to be downloaded waits for the user's permission first.
    void start_geometry_build(bool download_allowed = false) {
        task_request = geometry_request;
        const scene::GeometryRef ref = experiment.geometry;
        const std::filesystem::path dir = base_dir;
        if (!download_allowed && sim::needs_download(ref, dir)) {
            download_prompt = ref;
            download_prompt_requested = true;
            drop_geometry();
            return;
        }
        download_prompt.reset();
        sim::ExperimentDownload download;
        download.allowed = download_allowed;
        download.fetch.cache_dir = options.model_cache_dir;
        geometry_task = sim::BackgroundTask<sim::PrepareResult>(
            [ref, dir, download](sim::TaskProgress& progress) {
                return sim::load_experiment_geometry(ref, dir, &progress, download);
            },
            [](const std::string& message) -> sim::PrepareResult {
                return core::failure(sim::GeometryError{"Geometry failed", message});
            });
    }

    void answer_download(bool download) {
        if (!download_prompt) {
            return;
        }
        const bool current = *download_prompt == experiment.geometry;
        download_prompt.reset();
        if (download && current && !geometry_task.started()) {
            start_geometry_build(true);
        } else if (!download) {
            set_notice("Model not downloaded: the experiment has no geometry", true);
        }
    }

    void cancel_work() {
        rebuild_queued = false;
        geometry_task.request_stop();
        import_task.request_stop();
    }

    // Errors go to the modal; a cancellation the user asked for only to the
    // status line.
    void report_failure(const sim::GeometryError& cause) {
        if (cause.title == sim::cancelled_error().title) {
            set_notice("Cancelled", false);
            return;
        }
        show_error({cause.title, cause.detail});
    }

    void import_file() {
        const std::filesystem::path path(w::from_buffer(path_buffer));
        if (path.empty()) {
            show_error({"No file given", "enter the path of an OBJ, STL, glTF, GLB, PLY or FBX "
                                         "model file"});
            return;
        }
        const std::string license = w::from_buffer(license_buffer);
        pending_import.reset();
        import_task = sim::BackgroundTask<sim::ImportResult>(
            [path, license](sim::TaskProgress& progress) {
                return sim::load_model_file(path, license, &progress);
            },
            [](const std::string& message) -> sim::ImportResult {
                return core::failure(sim::GeometryError{"Import failed", message});
            });
    }

    void import_url() {
        const std::string url = w::from_buffer(url_buffer);
        if (url.empty()) {
            show_error({"No URL given", "enter an https:// address of a model file or zip"});
            return;
        }
        sim::UrlImportOptions fetch;
        fetch.cache_dir = options.model_cache_dir;
        fetch.license_note = w::from_buffer(license_buffer);
        pending_import.reset();
        import_task = sim::BackgroundTask<sim::ImportResult>(
            [url, fetch](sim::TaskProgress& progress) {
                return sim::fetch_model_url(url, fetch, &progress);
            },
            [](const std::string& message) -> sim::ImportResult {
                return core::failure(sim::GeometryError{"Download failed", message});
            });
    }

    void confirm_import() {
        if (!pending_import) {
            return;
        }
        const std::vector<sim::UnitChoice> units = sim::unit_choices();
        const double scale = pending_unit < static_cast<int>(units.size())
                                 ? units[static_cast<std::size_t>(pending_unit)].scale
                                 : pending_scale;
        const geom::UpAxis up = pending_up == 1 ? geom::UpAxis::y_up : geom::UpAxis::z_up;
        sim::ImportedModel model = std::move(*pending_import);
        pending_import.reset();
        model.ref.license_note = w::from_buffer(license_buffer);
        task_request = ++geometry_request;
        geometry_task = sim::BackgroundTask<sim::PrepareResult>(
            [model = std::move(model), scale, up](sim::TaskProgress& progress) {
                if (progress.stop_requested()) {
                    return sim::PrepareResult(core::failure(sim::cancelled_error()));
                }
                progress.set_stage("repairing");
                return sim::prepare_user_model(model, scale, up);
            },
            [](const std::string& message) -> sim::PrepareResult {
                return core::failure(sim::GeometryError{"Repair failed", message});
            });
    }

    void poll_tasks() {
        if (import_task.done()) {
            sim::ImportResult imported = import_task.take();
            if (imported) {
                sim::ImportedModel model = std::move(imported).value();
                // Preselect the unit the importer suggests.
                const std::vector<sim::UnitChoice> units = sim::unit_choices();
                pending_unit = static_cast<int>(units.size()); // custom
                pending_scale = model.imported.unit.scale_to_metres;
                for (std::size_t i = 0; i < units.size(); ++i) {
                    if (units[i].scale == model.imported.unit.scale_to_metres) {
                        pending_unit = static_cast<int>(i);
                    }
                }
                pending_up = model.imported.up.axis == geom::UpAxis::y_up ? 1 : 0;
                pending_import = std::move(model);
            } else {
                report_failure(imported.error());
            }
        }
        if (geometry_task.done()) {
            sim::PrepareResult prepared = geometry_task.take();
            if (rebuild_queued) {
                rebuild_queued = false;
                start_geometry_build();
            } else if (prepared) {
                adopt_geometry(std::move(prepared).value());
            } else {
                // The inputs no longer describe the body prepared before:
                // nothing may run or be saved against it.
                drop_geometry();
                report_failure(prepared.error());
            }
        }
    }

    void adopt_geometry(sim::PreparedGeometry prepared) {
        auto ready = std::make_shared<const sim::PreparedGeometry>(std::move(prepared));
        experiment.geometry = ready->ref;
        if (loaded_inputs && task_request == loaded_request) {
            loaded_inputs->geometry = ready->ref;
        }
        geometry = std::move(ready);
        ++geometry_revision;
        reframe_pending = true;
    }

    void drop_geometry() {
        if (geometry) {
            geometry.reset();
            ++geometry_revision;
            reframe_pending = true;
        }
    }

    // --- Solver ---------------------------------------------------------------

    [[nodiscard]] sim::FlowSummary input_flow() const {
        sim::AeroRunSpec spec;
        spec.geometry = geometry;
        spec.experiment = experiment;
        return spec.flow();
    }

    bool start_run() {
        if (!geometry || geometry_task.started()) {
            set_notice("The geometry is not ready yet", true);
            return false;
        }
        const sim::FlowSummary flow = input_flow();
        if (!flow.runnable()) {
            set_notice(flow.input_errors.front(), true);
            return false;
        }
        sim::AeroRunSpec spec;
        spec.geometry = geometry;
        spec.experiment = experiment;
        spec.experiment.result.reset();
        spec.experiment.solver.steps_per_frame = steps_per_frame;
        run_inputs = spec.experiment;
        run_record.reset();
        loaded_result.reset();
        loaded_inputs.reset();
        runner.send(sim::StartAeroRun{std::move(spec), to_convergence});
        notice.clear();
        return true;
    }

    void record_finished_run() {
        const sim::AeroProgress& p = runner.latest();
        if (p.status.state != sim::AeroRunState::finished || p.status.run == recorded_run ||
            !p.result || !p.diagnostics || !p.spec) {
            return;
        }
        recorded_run = p.status.run;
        scene::AeroExperiment record = p.spec->experiment;
        record.result =
            sim::record_result(*p.result, *p.diagnostics, p.status.solver, p.flow, p.reference);
        run_record = std::move(record);
    }

    // Whether the panels show the solver's published run: only a run of the
    // current experiment (run_inputs is cleared when another one is opened,
    // before the solver thread has even seen the stop).
    [[nodiscard]] bool shows_run(const sim::AeroProgress& p) const {
        return p.result != nullptr && p.spec != nullptr && run_inputs.has_value();
    }

    void set_notice(std::string message, bool is_error) {
        notice = std::move(message);
        notice_is_error = is_error;
    }

    // --- Camera ---------------------------------------------------------------

    [[nodiscard]] aero::FlowConditions view_conditions() const {
        const sim::AeroProgress& p = runner.latest();
        if (p.spec && p.spec->geometry == geometry && run_inputs) {
            return p.flow.flow;
        }
        return experiment.conditions.flow();
    }

    // Whether the view frames the wake as well: for lattice Boltzmann runs,
    // which have a flow field to show.
    [[nodiscard]] bool wake_view() const {
        const sim::AeroProgress& p = runner.latest();
        if (p.spec && p.spec->geometry == geometry && run_inputs) {
            return p.status.solver == aero::SolverKind::lattice_boltzmann;
        }
        return input_flow().solver == aero::SolverKind::lattice_boltzmann;
    }

    // Frames the body in the scene area: the framing is computed for the
    // area's aspect, moved back so that the area's height (not the
    // window's) spans the field of view, and the focus is shifted so that
    // the body sits at the area's centre instead of the window's.
    void reframe() {
        if (!geometry || camera_width <= 0 || camera_height <= 0) {
            return;
        }
        const sim::ViewFrame frame = sim::view_frame(view_conditions(), geometry->two_dimensional);
        const auto surface = view_surface(*geometry, {}, frame);
        if (!surface) {
            return;
        }
        const geom::Bounds bounds = view_bounds(surface.value().mesh);
        const bool usable = camera_scene.width > 1.0 && camera_scene.height > 1.0;
        const double aspect = usable ? camera_scene.width / camera_scene.height
                                     : static_cast<double>(camera_width) / camera_height;
        render::TurntableMode mode =
            view_camera(bounds, geometry->two_dimensional, wake_view(), aspect, camera.lens());
        if (usable) {
            mode.distance *=
                std::max(1.0, static_cast<double>(camera_height) / camera_scene.height);
            const double per_pixel =
                2.0 * mode.distance * std::tan(0.5 * camera.lens().vertical_fov) / camera_height;
            const double shift_x =
                (camera_scene.x + 0.5 * camera_scene.width - 0.5 * camera_width) * per_pixel;
            const double shift_up =
                (0.5 * camera_height - (camera_scene.y + 0.5 * camera_scene.height)) * per_pixel;
            const glm::dvec3 right{-std::sin(mode.yaw), std::cos(mode.yaw), 0.0};
            const glm::dvec3 up{-std::cos(mode.yaw) * std::sin(mode.pitch),
                                -std::sin(mode.yaw) * std::sin(mode.pitch), std::cos(mode.pitch)};
            mode.focus -= right * shift_x + up * shift_up;
        }
        mode.spin_rate = spin ? 0.25 : 0.0;
        camera.set_mode(mode);
        (void)camera.update(0.0, render::CameraInput{}, no_bodies());
    }

    // --- Experiments ------------------------------------------------------------

    void open_record(scene::AeroExperiment record, std::filesystem::path dir) {
        loaded_result = record.result;
        record.result.reset();
        experiment = std::move(record);
        base_dir = std::move(dir);
        steps_per_frame = std::max<std::int64_t>(1, experiment.solver.steps_per_frame);
        sync_buffers();
        pending_import.reset();
        rebuild_geometry();
        loaded_request = geometry_request;
        loaded_inputs.reset();
        if (loaded_result) {
            loaded_inputs = experiment;
        }
        runner.send(sim::StopAeroRun{});
        run_record.reset();
        run_inputs.reset();
    }

    void refresh_listing() {
        listing = sim::list_aero_experiments(options.experiment_dir);
        compare.assign(listing.experiments.size(), 0);
        listing_dirty = false;
    }

    std::optional<std::filesystem::path> save() {
        experiment.name = w::from_buffer(name_buffer);
        scene::AeroExperiment record = experiment;
        record.result.reset();
        bool with_result = false;
        if (run_record && same_inputs(*run_record, experiment)) {
            record.result = run_record->result;
            with_result = true;
        } else if (loaded_result && loaded_inputs && same_inputs(*loaded_inputs, experiment)) {
            record.result = loaded_result;
            with_result = true;
        }
        const std::string name = record.name.empty() ? std::string("experiment") : record.name;
        const std::filesystem::path path = sim::new_experiment_path(options.experiment_dir, name);
        auto saved = sim::save_aero_experiment_file(path, record);
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

    void export_comparison(const sim::ComparisonTable& table) {
        std::error_code error;
        std::filesystem::create_directories(options.experiment_dir, error);
        const std::filesystem::path csv_path =
            sim::new_experiment_path(options.experiment_dir, table.label + " comparison")
                .replace_extension(".csv");
        std::ofstream out(csv_path, std::ios::binary | std::ios::trunc);
        const std::string csv = sim::comparison_csv(table);
        out.write(csv.data(), static_cast<std::streamsize>(csv.size()));
        out.close();
        if (!out) {
            set_notice("Export failed: cannot write " + csv_path.string(), true);
            return;
        }
        set_notice("Exported " + csv_path.string(), false);
    }

    // --- Layout -------------------------------------------------------------------

    render::UiRect dockspace() {
        ImGuiViewport* viewport = ImGui::GetMainViewport();
        const ImGuiID dock_id = ImGui::GetID("OrreryAeroDockspace");
        if (!layout_built) {
            layout_built = true;
            ImGui::DockBuilderRemoveNode(dock_id);
            ImGui::DockBuilderAddNode(
                dock_id, static_cast<ImGuiDockNodeFlags>(ImGuiDockNodeFlags_DockSpace) |
                             ImGuiDockNodeFlags_PassthruCentralNode);
            ImGui::DockBuilderSetNodePos(dock_id, viewport->WorkPos);
            ImGui::DockBuilderSetNodeSize(dock_id, viewport->WorkSize);
            ImGuiID rest = 0;
            const ImGuiID left =
                ImGui::DockBuilderSplitNode(dock_id, ImGuiDir_Left, 0.24F, nullptr, &rest);
            const ImGuiID right =
                ImGui::DockBuilderSplitNode(rest, ImGuiDir_Right, 0.30F, nullptr, &rest);
            ImGuiID centre = 0;
            const ImGuiID bottom =
                ImGui::DockBuilderSplitNode(rest, ImGuiDir_Down, 0.32F, nullptr, &centre);
            ImGuiID left_top = 0;
            const ImGuiID left_bottom =
                ImGui::DockBuilderSplitNode(left, ImGuiDir_Down, 0.48F, nullptr, &left_top);
            ImGui::DockBuilderDockWindow(std::string(kGeometryWindow).c_str(), left_top);
            ImGui::DockBuilderDockWindow(std::string(kFlowWindow).c_str(), left_bottom);
            ImGui::DockBuilderDockWindow(std::string(kResultsWindow).c_str(), right);
            ImGui::DockBuilderDockWindow(std::string(kExperimentsWindow).c_str(), bottom);
            ImGui::DockBuilderDockWindow(std::string(kViewWindow).c_str(), bottom);
            for (const ImGuiID id : {left_top, left_bottom, right, bottom}) {
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

    // --- Panels -------------------------------------------------------------------

    void draw_geometry_panel();
    void draw_primitive_inputs();
    void draw_import_inputs();
    void draw_geometry_report();
    void draw_flow_panel();
    void draw_free_stream();
    void draw_derived(const sim::FlowSummary& flow);
    void draw_solver_choice(const sim::FlowSummary& flow, bool two_d);
    void draw_lattice_settings(bool two_d);
    void draw_run_button(bool runnable);
    // Coefficients of the shown run and their renormalisation to the
    // selected reference.
    struct ShownCoefficients {
        bool have = false;
        double cd = 0.0;
        double cl = 0.0;
        double cy = 0.0;
        double cm = 0.0;
        double run_area = 1.0;
        double run_length = 1.0;
        double area_factor = 1.0;
        double moment_factor = 1.0;
    };

    void draw_results_panel();
    void draw_run_status(const sim::AeroProgress& p, bool live,
                         const scene::AeroResultRecord* record);
    aero::ReferenceValues draw_reference(bool two_d);
    void draw_coefficients(const ShownCoefficients& c, bool two_d);
    void draw_breakdown(const aero::AeroResult& r, double area_factor, bool two_d);
    void draw_lattice_controls(const sim::RunDiagnostics* diagnostics, double steps_per_second);
    void draw_warnings(const aero::AeroResult* result, const scene::AeroResultRecord* record);
    void draw_convergence(const sim::RunDiagnostics* diagnostics, const aero::AeroResult* result,
                          const scene::AeroResultRecord* record);
    void draw_experiments_panel();
    void draw_view_panel();
    void draw_modal();
    void draw_download_prompt();
};

// --- Geometry panel ------------------------------------------------------------------

void AeroWorkspace::Impl::draw_geometry_panel() {
    if (!ImGui::Begin(std::string(kGeometryWindow).c_str())) {
        ImGui::End();
        return;
    }
    if (ImGui::BeginTabBar("##source")) {
        const std::array<const char*, 3> labels{"Primitive", "File", "URL"};
        // An opened experiment selects the tab of its geometry source once;
        // afterwards the tab bar decides.
        int shown = source;
        for (int i = 0; i < 3; ++i) {
            const ImGuiTabItemFlags flags =
                source_tab_pending && source == i ? ImGuiTabItemFlags_SetSelected : 0;
            if (ImGui::BeginTabItem(labels[static_cast<std::size_t>(i)], nullptr, flags)) {
                shown = i;
                ImGui::EndTabItem();
            }
        }
        ImGui::EndTabBar();
        if (!source_tab_pending) {
            source = shown;
        }
        source_tab_pending = false;
    }
    if (source == 0) {
        draw_primitive_inputs();
    } else {
        draw_import_inputs();
    }

    if (geometry_task.started() || import_task.started()) {
        ImGui::Dummy(ImVec2(0.0F, 2.0F));
        const char* stage = import_task.started() ? import_task.stage() : geometry_task.stage();
        w::muted(std::string("Working: ") + stage + dots());
        ImGui::SameLine();
        if (ImGui::SmallButton("Cancel##work")) {
            cancel_work();
        }
        w::tooltip("Stop the download, import or repair (a download stops within about a "
                   "second)");
    }
    draw_geometry_report();
    ImGui::End();
}

void AeroWorkspace::Impl::draw_primitive_inputs() {
    scene::GeometryRef& ref = experiment.geometry;
    bool rebuild = false;
    w::section("SHAPE");
    w::row_label("Shape");
    if (ImGui::BeginCombo("##shape", std::string(scene::display_name(ref.shape)).c_str())) {
        for (std::size_t i = 0; i < scene::kAeroShapeCount; ++i) {
            const auto shape = static_cast<scene::AeroShape>(i);
            if (ImGui::Selectable(std::string(scene::display_name(shape)).c_str(),
                                  shape == ref.shape)) {
                if (ref.source != scene::GeometrySource::primitive || shape != ref.shape) {
                    ref = scene::GeometryRef{};
                    ref.shape = shape;
                    if (shape == scene::AeroShape::capsule) {
                        ref.dimensions.diameter_m = geom::CapsuleShape::apollo().max_diameter;
                    }
                    w::set_buffer(naca_buffer, ref.naca);
                    rebuild = true;
                }
            }
        }
        ImGui::EndCombo();
    }
    for (const std::string_view key : scene::shape_dimensions(ref.shape)) {
        double* field = dimension_field(ref.dimensions, key);
        const bool angle = key == "half_angle_deg";
        w::row_label_with_unit(dimension_label(ref.shape, key), angle ? kDegree : "m");
        double value = *field;
        if (input_number(("##" + std::string(key)).c_str(), &value, "%.4g")) {
            if (value > 0.0 && std::isfinite(value) && (!angle || value < 90.0)) {
                *field = value;
                rebuild = true;
            } else {
                set_notice(std::string(dimension_label(ref.shape, key)) +
                               (angle ? " must be between 0 and 90 degrees" : " must be positive"),
                           true);
            }
        } else if (ImGui::IsItemActive()) {
            *field = value;
        }
        w::unit(angle ? kDegree : "m");
    }
    if (ref.shape == scene::AeroShape::naca_wing || ref.shape == scene::AeroShape::naca_section) {
        w::row_label("NACA");
        (void)ImGui::InputText("##naca", naca_buffer.data(), naca_buffer.size(),
                               ImGuiInputTextFlags_CharsDecimal);
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            const std::string digits = w::from_buffer(naca_buffer);
            if (geom::Naca4::from_designation(digits)) {
                ref.naca = digits;
                rebuild = true;
            } else {
                set_notice("'" + digits + "' is not a NACA 4-digit designation", true);
                w::set_buffer(naca_buffer, ref.naca);
            }
        }
        w::tooltip("Four digits: maximum camber (percent of chord), its position (tenths of "
                   "chord) and thickness (percent), e.g. 0012 or 2412");
    }
    if (ref.shape == scene::AeroShape::capsule) {
        w::row_label("Proportions");
        int preset = ref.capsule == "orion" ? 1 : 0;
        if (ImGui::Combo("##capsule", &preset, "Apollo\0Orion\0")) {
            ref.capsule = preset == 1 ? "orion" : "apollo";
            rebuild = true;
        }
    }
    if (rebuild) {
        ref.source = scene::GeometrySource::primitive;
        rebuild_geometry();
    }
}

void AeroWorkspace::Impl::draw_import_inputs() {
    const bool busy = import_task.started() || geometry_task.started();
    if (source == 1) {
        w::section("MODEL FILE");
        w::row_label("Path");
        (void)ImGui::InputTextWithHint("##path", "C:/models/body.stl", path_buffer.data(),
                                       path_buffer.size());
        w::muted("OBJ, STL, glTF, GLB, PLY or FBX");
    } else {
        w::section("MODEL URL");
        w::row_label("URL");
        (void)ImGui::InputTextWithHint("##url", "https://...", url_buffer.data(),
                                       url_buffer.size());
        w::muted("https only, at most 200 MB; zip archives are opened one level deep");
    }
    w::row_label("Licence");
    (void)ImGui::InputTextWithHint("##license", "source and licence of the model",
                                   license_buffer.data(), license_buffer.size());
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        experiment.geometry.license_note = w::from_buffer(license_buffer);
    }
    w::tooltip("Recorded with the experiment. The program does not check it: using the model "
               "is your responsibility.");
    ImGui::BeginDisabled(busy);
    if (ImGui::Button(source == 1 ? "Import" : "Download and import", ImVec2(-1.0F, 0.0F))) {
        if (source == 1) {
            import_file();
        } else {
            import_url();
        }
    }
    ImGui::EndDisabled();

    if (!pending_import) {
        return;
    }
    const sim::ImportedModel& model = *pending_import;
    const geom::UnitEstimate& estimate = model.imported.unit;
    w::section("CONFIRM UNITS AND ORIENTATION");
    w::text(model.file_name + kDot + w::grouped(model.imported.mesh.triangles.size()) +
            " triangles");
    w::wrapped("Largest extent " + w::significant(estimate.raw_extent, 4) + " file units; " +
                   (estimate.from_format ? "the format defines " : "the size suggests ") +
                   estimate.unit + ".",
               ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    const std::vector<sim::UnitChoice> units = sim::unit_choices();
    w::row_label("Unit");
    const std::string preview = pending_unit < static_cast<int>(units.size())
                                    ? units[static_cast<std::size_t>(pending_unit)].name
                                    : "custom scale";
    if (ImGui::BeginCombo("##unit", preview.c_str())) {
        for (std::size_t i = 0; i <= units.size(); ++i) {
            const char* label = i < units.size() ? units[i].name : "custom scale";
            if (ImGui::Selectable(label, pending_unit == static_cast<int>(i))) {
                pending_unit = static_cast<int>(i);
            }
        }
        ImGui::EndCombo();
    }
    if (pending_unit >= static_cast<int>(units.size())) {
        w::row_label_with_unit("Scale", "m/unit");
        (void)ImGui::InputDouble("##scale", &pending_scale, 0.0, 0.0, "%.6g");
        w::unit("m/unit");
    }
    const double scale = pending_unit < static_cast<int>(units.size())
                             ? units[static_cast<std::size_t>(pending_unit)].scale
                             : pending_scale;
    w::row_label("Up axis");
    (void)ImGui::RadioButton("Z up", &pending_up, 0);
    ImGui::SameLine();
    (void)ImGui::RadioButton("Y up", &pending_up, 1);
    w::muted(std::string(model.imported.up.from_format ? "The format defines " : "Convention: ") +
             (model.imported.up.axis == geom::UpAxis::y_up ? "Y up" : "Z up"));
    w::muted("Result: " + w::significant(estimate.raw_extent * scale, 4) + " m largest extent");
    ImGui::BeginDisabled(!(scale > 0.0) || !std::isfinite(scale) || geometry_task.started());
    if (ImGui::Button("Use this model", ImVec2(-1.0F, 0.0F))) {
        confirm_import();
    }
    ImGui::EndDisabled();
}

void AeroWorkspace::Impl::draw_geometry_report() {
    if (!geometry) {
        return;
    }
    const sim::PreparedGeometry& g = *geometry;
    w::section("BODY");
    w::text(g.label);
    if (g.unvalidated) {
        w::chip("UNVALIDATED GEOMETRY", w::theme().warning,
                "A user model has no validation case: its coefficients are not checked against a "
                "reference, and repair may have changed its shape.");
    } else if (g.two_dimensional) {
        w::muted("2D section, per metre of span");
    }
    if (w::begin_properties("##body", "Inconsistent edges")) {
        w::property("Triangles", w::grouped(g.mesh.triangles.size()));
        w::property("Watertight", g.report.watertight() ? "yes" : "no");
        w::property("Outward", g.report.outward() ? "yes" : "no");
        if (g.repair) {
            const sim::RepairSummary& r = *g.repair;
            const auto change = [](std::size_t before, std::size_t after) {
                return w::change(before, after);
            };
            w::property("Boundary edges", change(r.before.boundary_edges, r.after.boundary_edges));
            w::property("Non-manifold",
                        change(r.before.non_manifold_edges, r.after.non_manifold_edges));
            w::property("Inconsistent edges",
                        change(r.before.inconsistent_edges, r.after.inconsistent_edges));
            w::property("Components", change(r.before.components, r.after.components));
            w::property("Welded", std::to_string(r.welded_vertices) + " vertices");
            w::property("Removed", std::to_string(r.removed_degenerate + r.removed_duplicates +
                                                  r.removed_fins) +
                                       " triangles");
            w::property("Flipped", std::to_string(r.flipped_triangles) + " triangles");
            w::property("Holes filled", std::to_string(r.filled_holes));
        }
        w::end_properties();
    }
    if (g.repair && g.repair->orientation_conflict) {
        w::warning("A component could not be oriented consistently (non-manifold or "
                   "one-sided surface); the volume solvers may misclassify it.");
    }
    if (g.repair && !g.report.watertight()) {
        w::warning("The model is still open after repair: the lattice Boltzmann solver needs a "
                   "closed body; the panel and Newtonian results are approximate.");
    }
    if (!g.two_dimensional) {
        w::section("REFERENCE QUANTITIES");
        if (w::begin_properties("##reference", "Planform area")) {
            w::property("Frontal area", w::significant(g.quantities.frontal_area) + " " + kSquared);
            w::property("Planform area",
                        w::significant(g.quantities.planform_area) + " " + kSquared);
            w::property("Wetted area", w::significant(g.quantities.wetted_area) + " " + kSquared);
            w::property("Length", w::significant(g.quantities.reference_length) + " m");
            w::property("Span", w::significant(g.quantities.span) + " m");
            w::property("Volume", w::significant(g.quantities.volume) + " " + kCubed);
            w::end_properties();
        }
    }
    if (g.unvalidated && !g.ref.license_note.empty()) {
        w::muted("Licence: " + g.ref.license_note);
    }
}

// --- Flow panel ----------------------------------------------------------------------

void AeroWorkspace::Impl::draw_free_stream() {
    scene::AeroConditions& c = experiment.conditions;
    w::section("FREE STREAM");
    const auto row = [](const char* label, const char* id, double* value, const char* unit,
                        const char* format) {
        w::row_label_with_unit(label, unit);
        (void)ImGui::InputDouble(id, value, 0.0, 0.0, format);
        w::unit(unit);
    };
    row("Speed", "##speed", &c.speed_m_s, "m/s", "%.4g");
    row("Density", "##density", &c.density_kg_m3, kDensity, "%.4g");
    row("Viscosity", "##viscosity", &c.viscosity_pa_s, "Pa s", "%.4g");
    row("Temperature", "##temperature", &c.temperature_k, "K", "%.5g");
    w::row_label_with_unit("Angle (AoA)", kDegree);
    const double alpha_min = -25.0;
    const double alpha_max = 25.0;
    (void)ImGui::SliderScalar("##alpha", ImGuiDataType_Double, &c.angle_of_attack_deg, &alpha_min,
                              &alpha_max, "%.2f");
    w::unit(kDegree);
    w::tooltip("Ctrl+click to type a value outside the slider range");
    w::row_label_with_unit("Sideslip", kDegree);
    const double beta_min = -20.0;
    const double beta_max = 20.0;
    (void)ImGui::SliderScalar("##beta", ImGuiDataType_Double, &c.sideslip_deg, &beta_min, &beta_max,
                              "%.2f");
    w::unit(kDegree);
    if (ImGui::SmallButton("Sea-level air")) {
        c = sim::sea_level_air(c);
    }
    w::tooltip("ISA sea level: 1.225 kg/m3, 1.789e-5 Pa s, 288.15 K");
}

void AeroWorkspace::Impl::draw_derived(const sim::FlowSummary& flow) {
    for (const std::string& problem : flow.input_errors) {
        w::error(problem);
    }
    if (!flow.runnable()) {
        return;
    }
    w::section("DERIVED");
    if (w::begin_properties("##derived", "Speed of sound")) {
        w::property("Reynolds", w::significant(flow.reynolds, 3));
        w::property("Mach", w::fixed(flow.mach, 3));
        w::property("Dynamic pressure", w::significant(flow.dynamic_pressure) + " Pa");
        w::property("Speed of sound", w::fixed(flow.speed_of_sound, 1) + " m/s");
        w::property("Kinematic visc.", w::significant(flow.kinematic_viscosity, 3) + " m2/s");
        w::property("Reference length", w::significant(flow.reference_length) + " m");
        w::end_properties();
    }
}

void AeroWorkspace::Impl::draw_solver_choice(const sim::FlowSummary& flow, bool two_d) {
    w::section("SOLVER");
    w::row_label("Solver");
    // The override is read and written here only; the combo and the notes
    // work on plain values (keeps the optional-access analysis small).
    std::optional<aero::SolverKind>& choice = experiment.solver.solver;
    const int selection = choice.has_value() ? solver_index(*choice) + 1 : 0;
    const int picked = solver_combo(selection, flow.regime.recommended, two_d);
    if (picked == 0) {
        choice.reset();
    } else if (picked > 0) {
        choice = kSolverKinds[static_cast<std::size_t>(picked - 1)];
    }
    if (flow.runnable()) {
        regime_notes(flow);
    }
}
void AeroWorkspace::Impl::draw_lattice_settings(bool two_d) {
    w::section("LATTICE");
    const double default_cells = two_d ? 40.0 : 16.0;
    w::row_label_with_unit("Resolution", "cells/L");
    double cells = experiment.solver.lbm_cells_per_length > 0.0
                       ? experiment.solver.lbm_cells_per_length
                       : default_cells;
    if (ImGui::InputDouble("##cells", &cells, 0.0, 0.0, "%.4g") && cells > 0.0) {
        experiment.solver.lbm_cells_per_length = cells;
    }
    w::unit("cells/L");
    w::tooltip("Lattice cells per reference length. The workspace runs the CPU solver: "
               "3D lattices grow with the cube of this value.");
    w::row_label_with_unit("Duration", "L/U");
    double duration = experiment.solver.lbm_duration > 0.0 ? experiment.solver.lbm_duration
                                                           : (two_d ? 60.0 : 30.0);
    if (ImGui::InputDouble("##duration", &duration, 0.0, 0.0, "%.4g") && duration > 0.0) {
        experiment.solver.lbm_duration = duration;
    }
    w::unit("L/U");
    w::tooltip("Simulated time in convective times (reference length / speed).");
}

void AeroWorkspace::Impl::draw_run_button(bool runnable) {
    ImGui::Dummy(ImVec2(0.0F, 4.0F));
    const sim::AeroRunState state = runner.latest().status.state;
    const bool busy_run = state == sim::AeroRunState::setting_up ||
                          state == sim::AeroRunState::running || state == sim::AeroRunState::paused;
    const bool waiting = geometry == nullptr || geometry_task.started();
    const bool can_run = runnable && !waiting;
    ImGui::BeginDisabled(!can_run);
    ImGui::PushStyleColor(ImGuiCol_Button, w::color(w::theme().accent, 0.85F));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, w::color(w::theme().accent_hover));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, w::color(w::theme().accent_active));
    ImGui::PushStyleColor(ImGuiCol_Text, w::color(w::theme().title));
    if (ImGui::Button(busy_run ? "Restart run" : "Run", ImVec2(-1.0F, 0.0F))) {
        (void)start_run();
    }
    ImGui::PopStyleColor(4);
    ImGui::EndDisabled();
    if (!can_run) {
        w::tooltip(waiting ? "Waiting for the geometry" : "Fix the inputs marked above first");
    }
}

void AeroWorkspace::Impl::draw_flow_panel() {
    if (!ImGui::Begin(std::string(kFlowWindow).c_str())) {
        ImGui::End();
        return;
    }
    draw_free_stream();
    const sim::FlowSummary flow = input_flow();
    draw_derived(flow);
    const bool two_d = geometry != nullptr && geometry->two_dimensional;
    draw_solver_choice(flow, two_d);
    if (flow.solver == aero::SolverKind::lattice_boltzmann) {
        draw_lattice_settings(two_d);
    }
    // Run from here as well as from the bar: the inputs are right above.
    draw_run_button(flow.runnable());
    ImGui::End();
}

// --- Results panel -------------------------------------------------------------------

void AeroWorkspace::Impl::draw_run_status(const sim::AeroProgress& p, bool live,
                                          const scene::AeroResultRecord* record) {
    const sim::AeroRunState state = p.status.state;
    if (live || state == sim::AeroRunState::setting_up || state == sim::AeroRunState::failed) {
        w::text(std::string(sim::solver_display_name(p.status.solver)) +
                (p.status.two_dimensional ? " (2D)" : " (3D)"));
        render::UiColor tint = w::theme().accent;
        if (state == sim::AeroRunState::failed) {
            tint = w::theme().warning;
        } else if (state == sim::AeroRunState::finished) {
            tint = w::theme().accent_secondary;
        }
        w::chip_after(state_label(state), tint, nullptr);
        if (live && state == sim::AeroRunState::finished) {
            if (p.result->converged) {
                w::chip_after("CONVERGED", w::theme().accent_secondary,
                              "The solver's convergence test passed.");
            } else {
                w::chip_after("NOT CONVERGED", w::theme().warning,
                              "The run finished without passing the convergence test; see the "
                              "warnings below.");
            }
        }
    } else if (record != nullptr) {
        w::text(std::string(sim::solver_display_name(record->solver)) + kDot + "recorded result");
    } else {
        w::muted("No result yet: set up the geometry and the flow, then press Run.");
    }
    if (state == sim::AeroRunState::failed) {
        w::error(p.status.message);
    }
}

aero::ReferenceValues AeroWorkspace::Impl::draw_reference(bool two_d) {
    w::section("REFERENCE");
    if (two_d) {
        w::muted("2D section: chord x 1 m, per metre of span");
    } else {
        w::row_label("Area");
        if (ImGui::BeginCombo("##area", area_label(experiment.solver.reference_area))) {
            for (const geom::ReferenceAreaKind kind :
                 {geom::ReferenceAreaKind::frontal, geom::ReferenceAreaKind::planform,
                  geom::ReferenceAreaKind::wetted}) {
                if (ImGui::Selectable(area_label(kind), kind == experiment.solver.reference_area)) {
                    experiment.solver.reference_area = kind;
                }
            }
            ImGui::EndCombo();
        }
    }
    aero::ReferenceValues shown;
    if (geometry != nullptr) {
        shown = sim::reference_values(*geometry, experiment.solver);
    }
    if (w::begin_properties("##refvalues", "Length")) {
        w::property("Area", w::significant(shown.area) + " " + kSquared);
        w::property("Length", w::significant(shown.length) + " m");
        w::end_properties();
    }
    return shown;
}

void AeroWorkspace::Impl::draw_coefficients(const ShownCoefficients& c, bool two_d) {
    w::section("COEFFICIENTS");
    const ImGuiStyle& style = ImGui::GetStyle();
    if (ImGui::BeginTable("##coefficients", two_d ? 4 : 5, ImGuiTableFlags_SizingStretchSame)) {
        struct Tile {
            const char* caption;
            std::string value;
        };
        std::vector<Tile> tiles;
        tiles.reserve(5);
        tiles.push_back({"Cd", c.have ? w::fixed(c.cd * c.area_factor, 4) : "-"});
        tiles.push_back({"Cl", c.have ? w::fixed(c.cl * c.area_factor, 4) : "-"});
        tiles.push_back({"Cm", c.have ? w::fixed(c.cm * c.moment_factor, 4) : "-"});
        if (!two_d) {
            tiles.push_back({"Cy", c.have ? w::fixed(c.cy * c.area_factor, 4) : "-"});
        }
        tiles.push_back({"L/D", c.have && c.cd != 0.0 ? w::fixed(c.cl / c.cd, 2) : "-"});
        ImGui::TableNextRow();
        for (std::size_t i = 0; i < tiles.size(); ++i) {
            ImGui::TableSetColumnIndex(static_cast<int>(i));
            ImGui::PushFont(nullptr, style.FontSizeBase * 0.8F);
            w::muted(tiles[i].caption);
            ImGui::PopFont();
            ImGui::PushFont(nullptr, style.FontSizeBase * 1.3F);
            w::text(tiles[i].value);
            ImGui::PopFont();
        }
        ImGui::EndTable();
    }
    if (c.have && std::abs(c.area_factor - 1.0) > 1e-12) {
        w::muted("Renormalised from the run's reference area " + w::significant(c.run_area) + " " +
                 kSquared);
    }
}

void AeroWorkspace::Impl::draw_breakdown(const aero::AeroResult& r, double area_factor,
                                         bool two_d) {
    if (w::begin_properties("##breakdown", "Stagnation heat flux")) {
        w::property("Cd pressure", w::fixed(r.cd_pressure * area_factor, 4));
        if (r.cd_induced != 0.0) {
            w::property("Cd induced", w::fixed(r.cd_induced * area_factor, 4));
        }
        if (r.cd_friction != 0.0) {
            w::property("Cd friction (est.)", w::fixed(r.cd_friction * area_factor, 4));
        }
        const double heat = r.stagnation_heat_flux.value_or(0.0);
        if (heat > 0.0) {
            w::property("Stagnation heat flux", w::significant(heat * 1.0e-6, 3) + " MW/m2");
        }
        w::property("Force", w::significant(geom::norm(r.force), 4) + (two_d ? " N/m" : " N"));
        w::end_properties();
    }
    if (r.cd_friction != 0.0) {
        w::wrapped("Skin friction is an estimate, reported apart and not included in Cd.",
                   ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    }
}

void AeroWorkspace::Impl::draw_lattice_controls(const sim::RunDiagnostics* diagnostics,
                                                double steps_per_second) {
    w::section("LATTICE BOLTZMANN");
    w::row_label("Steps per frame");
    if (ImGui::BeginCombo("##spf",
                          w::grouped(static_cast<std::uint64_t>(steps_per_frame)).c_str())) {
        for (const std::int64_t steps : kStepsPerFrame) {
            if (ImGui::Selectable(w::grouped(static_cast<std::uint64_t>(steps)).c_str(),
                                  steps == steps_per_frame)) {
                steps_per_frame = steps;
                runner.send(sim::SetAeroStepsPerFrame{steps});
            }
        }
        ImGui::EndCombo();
    }
    w::tooltip("Lattice steps advanced for every rendered frame in the per-frame mode.");
    w::row_label("Run to convergence");
    if (ImGui::Checkbox("##toconv", &to_convergence)) {
        runner.send(sim::SetAeroToConvergence{to_convergence});
    }
    w::tooltip("Step back to back until the run's duration is reached instead of pacing "
               "the solver with the frames.");
    if (diagnostics == nullptr || diagnostics->planned_steps == 0) {
        return;
    }
    const double fraction = std::min(1.0, static_cast<double>(diagnostics->lattice_steps) /
                                              static_cast<double>(diagnostics->planned_steps));
    const std::string overlay = w::grouped(diagnostics->lattice_steps) + " / " +
                                w::grouped(diagnostics->planned_steps) + " steps";
    ImGui::ProgressBar(static_cast<float>(fraction), ImVec2(-1.0F, 0.0F), overlay.c_str());
    if (w::begin_properties("##lattice", "Throughput")) {
        w::property("Lattice", diagnostics->lattice);
        w::property("Throughput",
                    w::grouped(static_cast<std::uint64_t>(steps_per_second)) + " steps/s");
        const double strouhal = diagnostics->strouhal.value_or(0.0);
        if (strouhal > 0.0) {
            w::property("Strouhal", w::fixed(strouhal, 4));
        }
        if (diagnostics->blockage > 0.0) {
            w::property("Blockage", w::fixed(diagnostics->blockage * 100.0, 1) + " %");
        }
        w::end_properties();
    }
}

void AeroWorkspace::Impl::draw_warnings(const aero::AeroResult* result,
                                        const scene::AeroResultRecord* record) {
    if (result != nullptr && !result->warnings.empty()) {
        w::section("WARNINGS");
        for (const aero::ValidityWarning& warning : result->warnings) {
            w::warning(warning.message);
            w::tooltip(warning.code.c_str());
        }
    } else if (record != nullptr && !record->warnings.empty()) {
        w::section("WARNINGS");
        for (const std::string& code : record->warnings) {
            w::warning(code);
        }
    }
}

void AeroWorkspace::Impl::draw_results_panel() {
    if (!ImGui::Begin(std::string(kResultsWindow).c_str())) {
        ImGui::End();
        return;
    }
    const sim::AeroProgress& p = runner.latest();
    const bool live = shows_run(p);
    const scene::AeroResultRecord* record =
        !live && loaded_result.has_value() ? &loaded_result.value() : nullptr;
    draw_run_status(p, live, record);

    const bool two_d = geometry != nullptr && geometry->two_dimensional;
    const aero::ReferenceValues shown = draw_reference(two_d);
    ShownCoefficients c;
    c.run_area = shown.area;
    c.run_length = shown.length;
    if (live) {
        c = ShownCoefficients{true,         p.result->cd,     p.result->cl,      p.result->cy,
                              p.result->cm, p.reference.area, p.reference.length};
    } else if (record != nullptr) {
        c = ShownCoefficients{true,
                              record->cd,
                              record->cl,
                              record->cy,
                              record->cm,
                              record->reference_area_m2,
                              record->reference_length_m};
    }
    // Coefficients of the run renormalised to the selected reference.
    if (c.have && shown.area > 0.0) {
        c.area_factor = c.run_area / shown.area;
        if (shown.length > 0.0) {
            c.moment_factor = c.run_area * c.run_length / (shown.area * shown.length);
        }
    }
    draw_coefficients(c, two_d);
    if (live) {
        draw_breakdown(*p.result, c.area_factor, two_d);
    }

    const sim::RunDiagnostics* diagnostics = live ? p.diagnostics.get() : nullptr;
    const sim::AeroRunState state = p.status.state;
    const bool lbm = (live || state == sim::AeroRunState::setting_up)
                         ? p.status.solver == aero::SolverKind::lattice_boltzmann
                         : input_flow().solver == aero::SolverKind::lattice_boltzmann;
    if (lbm) {
        draw_lattice_controls(diagnostics, p.status.steps_per_second);
    }
    draw_convergence(diagnostics, live ? p.result.get() : nullptr, record);
    draw_warnings(live ? p.result.get() : nullptr, record);
    ImGui::End();
}

void AeroWorkspace::Impl::draw_convergence(const sim::RunDiagnostics* diagnostics,
                                           const aero::AeroResult* result,
                                           const scene::AeroResultRecord* record) {
    std::span<const double> step;
    std::span<const double> cd;
    std::span<const double> cl;
    std::span<const double> residuals;
    if (diagnostics != nullptr) {
        step = diagnostics->history_step;
        cd = diagnostics->history_cd;
        cl = diagnostics->history_cl;
    } else if (record != nullptr) {
        step = record->history_step;
        cd = record->history_cd;
        cl = record->history_cl;
    }
    if (result != nullptr) {
        residuals = result->residuals;
    } else if (record != nullptr) {
        residuals = record->residuals;
    }
    w::section("CONVERGENCE");
    if (!residuals.empty()) {
        w::muted("Residual " + w::significant(residuals.back(), 3) +
                 (residuals.size() > 1 ? kDot + std::to_string(residuals.size()) + " updates"
                                       : std::string()));
    }
    const ImPlotFlags flags = ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect | ImPlotFlags_NoTitle;
    const float height = std::max(ImGui::GetFontSize() * 9.0F, 90.0F);
    const std::size_t n = std::min({step.size(), cd.size(), cl.size()});
    if (ImPlot::BeginPlot("##history", ImVec2(-1.0F, height), flags)) {
        if (n == 0) {
            ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoTickLabels,
                              ImPlotAxisFlags_NoTickLabels);
            ImPlot::SetupAxesLimits(0.0, 1.0, 0.0, 1.0, ImPlotCond_Always);
            ImPlot::PlotText(result != nullptr ? "Direct solve: no history" : "No history yet", 0.5,
                             0.5);
        } else {
            ImPlot::SetupAxes("lattice step", nullptr, ImPlotAxisFlags_AutoFit,
                              ImPlotAxisFlags_AutoFit);
            ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
            ImPlotSpec drag;
            drag.LineColor = w::color(w::theme().accent);
            drag.LineWeight = 1.5F;
            ImPlot::PlotLine("Cd", step.data(), cd.data(), static_cast<int>(n), drag);
            ImPlotSpec lift;
            lift.LineColor = w::color(w::theme().accent_secondary);
            lift.LineWeight = 1.5F;
            ImPlot::PlotLine("Cl", step.data(), cl.data(), static_cast<int>(n), lift);
        }
        ImPlot::EndPlot();
    }
    std::vector<double> positive;
    std::vector<double> index;
    positive.reserve(residuals.size());
    index.reserve(residuals.size());
    for (std::size_t i = 0; i < residuals.size(); ++i) {
        if (residuals[i] > 0.0 && std::isfinite(residuals[i])) {
            positive.push_back(residuals[i]);
            index.push_back(static_cast<double>(i + 1));
        }
    }
    if (positive.size() > 1 && ImPlot::BeginPlot("##residual", ImVec2(-1.0F, height * 0.6F),
                                                 flags | ImPlotFlags_NoLegend)) {
        ImPlot::SetupAxes("update", "residual", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
        ImPlotSpec spec;
        spec.LineColor = w::color(w::theme().text_muted);
        spec.LineWeight = 1.2F;
        ImPlot::PlotLine("residual", index.data(), positive.data(),
                         static_cast<int>(positive.size()), spec);
        ImPlot::EndPlot();
    }
}

// --- Experiments panel ---------------------------------------------------------------

void AeroWorkspace::Impl::draw_experiments_panel() {
    if (!ImGui::Begin(std::string(kExperimentsWindow).c_str())) {
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
        experiment.name = w::from_buffer(name_buffer);
    }
    if (ImGui::Button("Save")) {
        (void)save();
    }
    w::tooltip("Writes the inputs and, when the last finished run used exactly these inputs, "
               "its result to the experiments folder.");
    ImGui::SameLine();
    if (ImGui::Button("Refresh")) {
        listing_dirty = true;
    }
    ImGui::SameLine();
    w::muted(display_path(options.experiment_dir));
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

    w::section("SAVED");
    if (listing.experiments.empty()) {
        w::muted("No aero experiments saved yet.");
    }
    std::optional<std::size_t> load;
    constexpr ImGuiTableFlags kTableFlags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH |
                                            ImGuiTableFlags_SizingStretchProp |
                                            ImGuiTableFlags_ScrollY;
    const float list_height = std::min(ImGui::GetTextLineHeightWithSpacing() *
                                           static_cast<float>(listing.experiments.size() + 2),
                                       ImGui::GetFontSize() * 12.0F);
    if (!listing.experiments.empty() &&
        ImGui::BeginTable("##saved", 8, kTableFlags, ImVec2(0.0F, list_height))) {
        ImGui::TableSetupScrollFreeze(0, 1);
        for (const char* header : {"", "Name", "Geometry", "AoA", "Solver", "Cd", "Cl", ""}) {
            ImGui::TableSetupColumn(header);
        }
        ImGui::TableHeadersRow();
        for (std::size_t i = 0; i < listing.experiments.size(); ++i) {
            const scene::AeroExperiment& e = listing.experiments[i].experiment;
            ImGui::PushID(static_cast<int>(i));
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            bool selected = compare[i] != 0;
            if (ImGui::Checkbox("##compare", &selected)) {
                compare[i] = selected ? 1 : 0;
            }
            w::tooltip("Include in the comparison");
            ImGui::TableSetColumnIndex(1);
            w::text(e.name.empty() ? listing.experiments[i].path.stem().string() : e.name);
            ImGui::TableSetColumnIndex(2);
            w::muted(sim::geometry_label(e.geometry));
            ImGui::TableSetColumnIndex(3);
            w::text(w::fixed(e.conditions.angle_of_attack_deg, 2) + kDegree);
            ImGui::TableSetColumnIndex(4);
            w::text(e.result ? std::string(sim::solver_display_name(e.result->solver)) : "-");
            ImGui::TableSetColumnIndex(5);
            w::text(e.result ? w::fixed(e.result->cd, 4) : "-");
            ImGui::TableSetColumnIndex(6);
            w::text(e.result ? w::fixed(e.result->cl, 4) : "-");
            ImGui::TableSetColumnIndex(7);
            if (ImGui::SmallButton("Load")) {
                load = i;
            }
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    if (load) {
        const sim::StoredExperiment chosen = listing.experiments[*load];
        open_record(chosen.experiment, chosen.path.parent_path());
        set_notice("Loaded " + chosen.path.filename().string(), false);
    }

    // Comparison of the selected experiments (all when none is selected).
    std::vector<scene::AeroExperiment> chosen;
    const bool any = std::any_of(compare.begin(), compare.end(), [](char c) { return c != 0; });
    for (std::size_t i = 0; i < listing.experiments.size(); ++i) {
        if (!any || compare[i] != 0) {
            chosen.push_back(listing.experiments[i].experiment);
        }
    }
    const std::vector<sim::ComparisonTable> tables = sim::compare_by_geometry(chosen);
    w::section("COMPARISON");
    if (tables.empty()) {
        w::muted("Save runs of one geometry at several angles of attack to compare them.");
    }
    for (std::size_t t = 0; t < tables.size(); ++t) {
        const sim::ComparisonTable& table = tables[t];
        ImGui::PushID(static_cast<int>(t + 1000));
        ImGui::AlignTextToFramePadding();
        w::text(table.label);
        if (table.lift_slope_per_deg) {
            ImGui::SameLine();
            w::muted("dCl/da " + w::fixed(*table.lift_slope_per_deg, 4) + " /deg (" +
                     w::fixed(*table.lift_slope_per_rad, 3) + " /rad; thin airfoil 2pi = 6.283)");
        }
        ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - ImGui::CalcTextSize("Export CSV").x -
                        ImGui::GetStyle().FramePadding.x * 2.0F);
        if (ImGui::SmallButton("Export CSV")) {
            export_comparison(table);
        }
        if (ImGui::BeginTable("##comparison", 9,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH |
                                  ImGuiTableFlags_SizingStretchSame)) {
            for (const char* header :
                 {"AoA", "Solver", "Cd", "Cl", "Cm", "dCd", "dCl", "dCm", "L/D"}) {
                ImGui::TableSetupColumn(header);
            }
            ImGui::TableHeadersRow();
            for (const sim::ComparisonRow& row : table.rows) {
                ImGui::TableNextRow();
                const std::array<std::string, 9> cells{
                    w::fixed(row.angle_of_attack_deg, 2) + kDegree,
                    std::string(aero::solver_name(row.solver)),
                    w::fixed(row.cd, 4),
                    w::fixed(row.cl, 4),
                    w::fixed(row.cm, 4),
                    (row.delta_cd >= 0.0 ? "+" : "") + w::fixed(row.delta_cd, 4),
                    (row.delta_cl >= 0.0 ? "+" : "") + w::fixed(row.delta_cl, 4),
                    (row.delta_cm >= 0.0 ? "+" : "") + w::fixed(row.delta_cm, 4),
                    row.cd != 0.0 ? w::fixed(row.lift_to_drag, 2) : std::string("-")};
                for (std::size_t c = 0; c < cells.size(); ++c) {
                    ImGui::TableSetColumnIndex(static_cast<int>(c));
                    if (c >= 5 && c <= 7) {
                        w::muted(cells[c]);
                    } else {
                        w::text(cells[c]);
                    }
                }
            }
            ImGui::EndTable();
        }
        ImGui::PopID();
    }
    ImGui::End();
}

// --- View panel ----------------------------------------------------------------------

void AeroWorkspace::Impl::draw_view_panel() {
    if (!ImGui::Begin(std::string(kViewWindow).c_str())) {
        ImGui::End();
        return;
    }
    const bool two_d = geometry && geometry->two_dimensional;
    w::section("SURFACE");
    (void)ImGui::Checkbox("Pressure coefficient on the body", &layers.cp_surface);
    (void)ImGui::Checkbox("Force arrows", &layers.forces);
    w::section("FLOW FIELD");
    w::muted("Lattice Boltzmann runs only; 3D fields at zero angles are in body axes.");
    (void)ImGui::Checkbox("Section plane", &layers.section);
    if (layers.section) {
        w::row_label("Quantity");
        constexpr std::array<std::pair<render::FieldQuantity, const char*>, 5> kQuantities{{
            {render::FieldQuantity::speed, "Speed"},
            {render::FieldQuantity::vorticity, "Vorticity"},
            {render::FieldQuantity::vorticity_magnitude, "Vorticity magnitude"},
            {render::FieldQuantity::pressure, "Pressure coefficient"},
            {render::FieldQuantity::q_criterion, "Q-criterion"},
        }};
        const char* preview = "Speed";
        for (const auto& [quantity, label] : kQuantities) {
            if (quantity == layers.section_quantity) {
                preview = label;
            }
        }
        if (ImGui::BeginCombo("##quantity", preview)) {
            for (const auto& [quantity, label] : kQuantities) {
                if (ImGui::Selectable(label, quantity == layers.section_quantity)) {
                    layers.section_quantity = quantity;
                }
            }
            ImGui::EndCombo();
        }
        if (!two_d) {
            w::row_label("Position");
            const double lo = 0.0;
            const double hi = 1.0;
            (void)ImGui::SliderScalar("##position", ImGuiDataType_Double, &layers.section_fraction,
                                      &lo, &hi, "%.2f across the span");
        }
    }
    (void)ImGui::Checkbox("Streamlines", &layers.streamlines);
    (void)ImGui::Checkbox("Particles", &layers.particles);
    if (!two_d) {
        (void)ImGui::Checkbox("Q-criterion iso-surface", &layers.q_iso);
        if (layers.q_iso) {
            w::row_label("Level");
            const double lo = 0.005;
            const double hi = 0.5;
            (void)ImGui::SliderScalar("##qlevel", ImGuiDataType_Double, &layers.q_level, &lo, &hi,
                                      "%.3f of max Q", ImGuiSliderFlags_Logarithmic);
        }
    }
    if (!view_problem.empty()) {
        w::error("The scene could not show everything: " + view_problem);
    }
    w::section("CAMERA");
    if (ImGui::Button("Frame the body (F)")) {
        reframe_pending = true;
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("Turn", &spin)) {
        reframe_pending = true;
    }
    w::muted("Drag to orbit, wheel to zoom.");
    ImGui::End();
}

// --- Error modal ---------------------------------------------------------------------

void AeroWorkspace::Impl::draw_modal() {
    const std::string id(kErrorModal);
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
            ImGui::TextUnformatted(modal->title.c_str());
            ImGui::PopStyleColor();
            ImGui::Separator();
            w::wrapped(modal->detail, ImGui::GetStyleColorVec4(ImGuiCol_Text));
            ImGui::Dummy(ImVec2(0.0F, 4.0F));
        }
        if (ImGui::Button("Copy details")) {
            ImGui::SetClipboardText(modal ? (modal->title + "\n" + modal->detail).c_str() : "");
        }
        ImGui::SameLine();
        if (ImGui::Button("Close") || ImGui::IsKeyPressed(ImGuiKey_Escape)) {
            modal.reset();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

// --- Download prompt -----------------------------------------------------------------

void AeroWorkspace::Impl::draw_download_prompt() {
    const std::string id(kDownloadPrompt);
    if (download_prompt_requested) {
        ImGui::OpenPopup(id.c_str());
        download_prompt_requested = false;
    }
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5F, 0.5F));
    ImGui::SetNextWindowSizeConstraints(ImVec2(ImGui::GetFontSize() * 24.0F, 0.0F),
                                        ImVec2(ImGui::GetFontSize() * 42.0F, FLT_MAX));
    if (!ImGui::BeginPopupModal(id.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        return;
    }
    if (!download_prompt) {
        ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
        return;
    }
    const scene::GeometryRef& ref = *download_prompt;
    w::wrapped("This experiment wants to download a model from " + sim::url_host(ref.url) + ".",
               ImGui::GetStyleColorVec4(ImGuiCol_Text));
    ImGui::Separator();
    w::muted("URL");
    w::wrapped(ref.url, ImGui::GetStyleColorVec4(ImGuiCol_Text));
    w::muted("Recorded SHA-256");
    w::wrapped(ref.sha256.empty() ? std::string("none recorded") : ref.sha256,
               ImGui::GetStyleColorVec4(ImGuiCol_Text));
    w::muted("The download is compared with this hash before the model is read.");
    ImGui::Dummy(ImVec2(0.0F, 4.0F));
    if (ImGui::Button("Download")) {
        answer_download(true);
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel") || ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        answer_download(false);
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
}

// --- Public interface ----------------------------------------------------------------

AeroWorkspace::AeroWorkspace(AeroWorkspaceOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}

AeroWorkspace::~AeroWorkspace() = default;

void AeroWorkspace::open(scene::AeroExperiment experiment, std::filesystem::path base_dir) {
    impl_->open_record(std::move(experiment), std::move(base_dir));
}

void AeroWorkspace::update(double dt) {
    Impl& s = *impl_;
    s.poll_tasks();
    (void)s.runner.fetch();
    s.record_finished_run();
    s.runner.wake();
    (void)s.camera.update(dt, s.camera_input, no_bodies());
    s.camera_input = render::CameraInput{};
}

void AeroWorkspace::draw_bar(double fps) {
    Impl& s = *impl_;
    const ImGuiStyle& base = ImGui::GetStyle();
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,
                        ImVec2(base.FramePadding.x, base.FramePadding.y + 3.0F));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(6.0F, base.ItemSpacing.y));
    if (!ImGui::BeginMainMenuBar()) {
        ImGui::PopStyleVar(2);
        return;
    }
    const ImGuiStyle& style = ImGui::GetStyle();
    const float gap = style.ItemSpacing.x * 2.0F;
    const bool compact = ImGui::GetWindowWidth() < ImGui::GetFontSize() * 80.0F;
    w::word_mark();
    ImGui::Dummy(ImVec2(gap, 0.0F));
    w::workspace_label(std::string(kAeroWorkspace).c_str(),
                       "Aero workspace: geometry, flow conditions and solvers");
    ImGui::Dummy(ImVec2(gap * 0.5F, 0.0F));
    ImGui::Separator();

    const sim::AeroProgress& p = s.runner.latest();
    const sim::AeroRunState state = p.status.state;
    const bool active = state == sim::AeroRunState::running || state == sim::AeroRunState::paused;
    const bool ready = s.geometry != nullptr && !s.geometry_task.started();
    if (state == sim::AeroRunState::running) {
        if (w::icon_button("##pause", w::Glyph::pause, "Pause the solver (Space)", false)) {
            s.runner.send(sim::SetAeroPaused{true});
        }
    } else if (state == sim::AeroRunState::paused) {
        if (w::icon_button("##resume", w::Glyph::play, "Resume the solver (Space)", true)) {
            s.runner.send(sim::SetAeroPaused{false});
        }
    } else if (w::icon_button("##run", w::Glyph::play, "Run the solver on these inputs (Space)",
                              true, ready && state != sim::AeroRunState::setting_up)) {
        (void)s.start_run();
    }
    if (w::icon_button("##step", w::Glyph::step,
                       "Advance one frame's worth of lattice steps and pause (. or Right)", false,
                       active)) {
        s.runner.send(sim::StepAeroFrame{});
    }
    if (w::icon_button("##stop", w::Glyph::stop, "Stop the run (the last result stays shown)",
                       false, active || state == sim::AeroRunState::setting_up)) {
        s.runner.send(sim::StopAeroRun{});
    }
    ImGui::Separator();

    if (!compact) {
        w::muted("PACE");
    }
    const bool lbm = (active || state == sim::AeroRunState::setting_up)
                         ? p.status.solver == aero::SolverKind::lattice_boltzmann
                         : s.input_flow().solver == aero::SolverKind::lattice_boltzmann;
    ImGui::BeginDisabled(!lbm);
    const std::string pace =
        s.to_convergence
            ? std::string("to convergence")
            : w::grouped(static_cast<std::uint64_t>(s.steps_per_frame)) + " steps/frame";
    if (ImGui::Button((pace + "##pace").c_str())) {
        ImGui::OpenPopup("##pace_menu");
    }
    ImGui::EndDisabled();
    w::tooltip(lbm ? "Lattice Boltzmann pacing: steps per rendered frame, or back to back until "
                     "the run's duration is reached"
                   : "Direct solvers finish in one step; pacing applies to lattice Boltzmann runs");
    if (ImGui::BeginPopup("##pace_menu")) {
        if (ImGui::Selectable("Run to convergence", s.to_convergence)) {
            s.to_convergence = true;
            s.runner.send(sim::SetAeroToConvergence{true});
        }
        ImGui::Separator();
        for (const std::int64_t steps : kStepsPerFrame) {
            const std::string label =
                w::grouped(static_cast<std::uint64_t>(steps)) + " steps per frame";
            if (ImGui::Selectable(label.c_str(), !s.to_convergence && steps == s.steps_per_frame)) {
                s.to_convergence = false;
                s.steps_per_frame = steps;
                s.runner.send(sim::SetAeroToConvergence{false});
                s.runner.send(sim::SetAeroStepsPerFrame{steps});
            }
        }
        ImGui::EndPopup();
    }
    ImGui::Separator();
    if (!compact) {
        w::muted("SOLVER");
    }
    const aero::SolverKind solver =
        (active || state == sim::AeroRunState::setting_up || state == sim::AeroRunState::finished)
            ? p.status.solver
            : s.input_flow().solver;
    w::text(std::string(sim::solver_display_name(solver)));
    if (s.geometry && s.geometry->unvalidated) {
        ImGui::SameLine();
        w::chip("UNVALIDATED", w::theme().warning, "User model: no validation case");
    }

    // Right-aligned status and frame rate.
    std::string status = state_label(state);
    if (const sim::RunDiagnostics* d = p.diagnostics.get();
        d != nullptr && d->planned_steps > 0 && state != sim::AeroRunState::idle) {
        status += kDot + w::grouped(d->lattice_steps) + " / " + w::grouped(d->planned_steps);
        if (state == sim::AeroRunState::running && p.status.steps_per_second > 0.0) {
            status += kDot + w::grouped(static_cast<std::uint64_t>(p.status.steps_per_second)) +
                      " steps/s";
        }
    } else if (state == sim::AeroRunState::finished && p.result) {
        status += std::string(kDot) + "Cd " + w::fixed(p.result->cd, 4) + kDot + "Cl " +
                  w::fixed(p.result->cl, 4);
    }
    if (s.geometry_task.started() || s.import_task.started()) {
        status =
            std::string(s.import_task.started() ? s.import_task.stage() : s.geometry_task.stage()) +
            dots();
    }
    const std::string fps_text = w::fixed(fps, 0) + " fps";
    const float status_width = ImGui::CalcTextSize(status.c_str()).x;
    const float fps_width = ImGui::CalcTextSize(fps_text.c_str()).x + gap;
    const float start = ImGui::GetWindowContentRegionMax().x - status_width - fps_width;
    if (start > ImGui::GetCursorPosX()) {
        ImGui::SetCursorPosX(start);
    }
    if (state == sim::AeroRunState::failed) {
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

render::UiRect AeroWorkspace::draw_panels() {
    Impl& s = *impl_;
    const render::UiRect scene = s.dockspace();
    s.draw_geometry_panel();
    s.draw_flow_panel();
    s.draw_results_panel();
    s.draw_view_panel();
    s.draw_experiments_panel();
    s.draw_modal();
    s.draw_download_prompt();
    return scene;
}

void AeroWorkspace::handle_scene_input(const render::SceneInput& input, double dt) {
    Impl& s = *impl_;
    if (input.toggle_pause) {
        const sim::AeroRunState state = s.runner.latest().status.state;
        if (state == sim::AeroRunState::running) {
            s.runner.send(sim::SetAeroPaused{true});
        } else if (state == sim::AeroRunState::paused) {
            s.runner.send(sim::SetAeroPaused{false});
        } else if (state != sim::AeroRunState::setting_up) {
            (void)s.start_run();
        }
    }
    if (input.single_step) {
        s.runner.send(sim::StepAeroFrame{});
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

const render::Camera& AeroWorkspace::camera(const render::UiRect& scene, int width, int height) {
    Impl& s = *impl_;
    const bool moved = std::abs(scene.x - s.camera_scene.x) > 0.5 ||
                       std::abs(scene.y - s.camera_scene.y) > 0.5 ||
                       std::abs(scene.width - s.camera_scene.width) > 0.5 ||
                       std::abs(scene.height - s.camera_scene.height) > 0.5 ||
                       width != s.camera_width || height != s.camera_height;
    if (s.reframe_pending || moved) {
        s.reframe_pending = false;
        s.camera_scene = scene;
        s.camera_width = width;
        s.camera_height = height;
        s.reframe();
    }
    return s.camera;
}

AeroViewState AeroWorkspace::view_state() const {
    const Impl& s = *impl_;
    AeroViewState state;
    state.geometry = s.geometry;
    state.geometry_revision = s.geometry_revision;
    state.layers = s.layers;
    state.conditions = s.view_conditions();
    const sim::AeroProgress& p = s.runner.latest();
    // A run's view data belongs to its own geometry: show it only while that
    // geometry is the current one.
    if (p.spec && p.spec->geometry == s.geometry && s.run_inputs) {
        state.result = p.result;
        state.surface_cp = p.surface_cp;
        state.flow = p.flow_field;
        state.view_revision = p.view_revision;
        state.running = p.status.state == sim::AeroRunState::running;
    }
    return state;
}

const sim::AeroProgress& AeroWorkspace::progress() const noexcept {
    return impl_->runner.latest();
}

const scene::AeroExperiment& AeroWorkspace::experiment() const noexcept {
    return impl_->experiment;
}

bool AeroWorkspace::geometry_ready() const noexcept {
    return impl_->geometry != nullptr && !impl_->geometry_task.started();
}

std::optional<AeroShownResult> AeroWorkspace::shown_result() const {
    const Impl& s = *impl_;
    const sim::AeroProgress& p = s.runner.latest();
    if (s.shows_run(p)) {
        return AeroShownResult{p.result->cd, p.result->cl, false};
    }
    if (s.loaded_result) {
        return AeroShownResult{s.loaded_result->cd, s.loaded_result->cl, true};
    }
    return std::nullopt;
}

bool AeroWorkspace::busy() const noexcept {
    return impl_->geometry_task.started() || impl_->import_task.started();
}

bool AeroWorkspace::start_run() {
    return impl_->start_run();
}

std::optional<std::filesystem::path> AeroWorkspace::save() {
    return impl_->save();
}

void AeroWorkspace::set_view_problem(std::string problem) {
    impl_->view_problem = std::move(problem);
}

std::optional<AeroDownloadRequest> AeroWorkspace::pending_download() const {
    const Impl& s = *impl_;
    if (!s.download_prompt) {
        return std::nullopt;
    }
    return AeroDownloadRequest{s.download_prompt->url, sim::url_host(s.download_prompt->url),
                               s.download_prompt->sha256};
}

void AeroWorkspace::answer_download(bool download) {
    impl_->answer_download(download);
}

void AeroWorkspace::cancel_work() {
    impl_->cancel_work();
}

void AeroWorkspace::set_conditions(const scene::AeroConditions& conditions) {
    impl_->experiment.conditions = conditions;
}

void AeroWorkspace::set_solver_settings(const scene::AeroSolverSettings& settings) {
    impl_->experiment.solver = settings;
}

} // namespace orrery::app
