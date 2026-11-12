#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <glm/ext/vector_double2.hpp>
#include <glm/ext/vector_double3.hpp>

#include "orrery/core/body.h"
#include "orrery/core/sim_snapshot.h"
#include "orrery/render/render_settings.h"
#include "orrery/render/space_renderer.h"

// Panels of the space workspace (docs/rendering.md, "User interface").
//
// Every panel is a function of a plain data struct that returns a user-intent
// struct: no physics access, no globals, no retained state of its own. The
// caller (the application loop) turns intents into camera moves, settings or
// SimCommands. Panels draw with Dear ImGui and must be called between
// UiContext::begin_frame and UiContext::end_frame.
namespace orrery::render {

inline constexpr std::string_view kBodyInspectorWindow = "Body Inspector";
inline constexpr std::string_view kSystemMonitorWindow = "System Monitor";
inline constexpr std::string_view kRenderSettingsWindow = "Render Settings";

struct BodyListEntry {
    core::BodyId id;
    std::string name;
    core::BodyKindTag kind = core::BodyKindTag::planet;
};

struct OsculatingOrbit {
    std::string primary;             // name of the body orbited
    double semi_major_axis_au = 0.0; // negative for hyperbolic orbits
    double eccentricity = 0.0;
    double inclination_deg = 0.0;      // to the ecliptic J2000
    std::optional<double> period_days; // elliptic orbits only
};

struct BodyDetails {
    core::BodyId id;
    std::string name;
    core::BodyKindTag kind = core::BodyKindTag::planet;
    double mass_solar = 0.0;
    double mass_kg = 0.0;
    double radius_km = 0.0; // drawn physical radius (nominal if the body has none)
    bool radius_is_nominal = false;
    glm::dvec3 position_au{0.0}; // barycentric, ecliptic J2000
    glm::dvec3 velocity_km_s{0.0};
    double distance_au = 0.0; // from the origin
    double speed_km_s = 0.0;
    std::optional<OsculatingOrbit> orbit;
};

struct BodyInspectorData {
    std::vector<BodyListEntry> bodies;
    std::optional<BodyDetails> selected;
};

struct BodyInspectorIntent {
    std::optional<core::BodyId> select; // the user picked another body
    bool focus_camera = false;          // "Focus" pressed for the selected body
};

// Inspector data for `selected` (nullopt or an id not in the snapshot: no
// details). The osculating orbit is taken relative to the body's primary:
// the least massive heavier body whose sphere of influence
// r_SOI = r (m / M_heaviest)^(2/5) contains it, else the heaviest body.
[[nodiscard]] BodyInspectorData make_body_inspector_data(const core::SimSnapshot& snapshot,
                                                         std::optional<core::BodyId> selected);

[[nodiscard]] BodyInspectorIntent draw_body_inspector(const BodyInspectorData& data);

struct SystemMonitorData {
    double sim_time_days = 0.0;
    std::size_t body_count = 0;
    double frame_time_ms = 0.0;
    std::optional<double> physics_step_ms;
    std::string integrator;
    // Recorded series supplied by the caller (for example the relative
    // energy drift from core::ConservationDiagnostics against time in days).
    // Both spans have the same length.
    std::string_view series_label = "Relative energy drift";
    std::span<const double> series_time_days;
    std::span<const double> series_values;
};

struct SystemMonitorIntent {
    bool clear_series = false;
};

[[nodiscard]] SystemMonitorIntent draw_system_monitor(const SystemMonitorData& data);

struct RenderSettingsIntent {
    std::optional<RenderSettings> changed; // the edited settings, when anything changed
};

// Sections that depend on the scene.
struct RenderSettingsContext {
    bool black_hole = false; // the scene holds a black hole: show the BLACK HOLE section
};

[[nodiscard]] RenderSettingsIntent draw_render_settings(const RenderSettings& settings,
                                                        const RenderSettingsContext& context = {});

// Labels next to bodies in the scene, drawn behind the panels.
struct SceneLabel {
    glm::dvec2 pixel{0.0}; // projected centre [px], origin top-left
    double radius_px = 0.0;
    std::string name;
    core::BodyKindTag kind = core::BodyKindTag::planet;
    bool selected = false;
};

void draw_scene_labels(std::span<const SceneLabel> labels);

// Labels for the visible bodies of a frame, decluttered: the selected body
// first, then by decreasing mass, skipping any body whose projected centre
// lies within `min_spacing_px` of a label already placed.
[[nodiscard]] std::vector<SceneLabel> make_scene_labels(const core::SimSnapshot& snapshot,
                                                        std::span<const BodyScreenInfo> screen,
                                                        std::optional<core::BodyId> selected,
                                                        double min_spacing_px = 36.0);

} // namespace orrery::render
