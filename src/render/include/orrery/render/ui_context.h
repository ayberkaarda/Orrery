#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "orrery/core/result.h"
#include "orrery/render/gl_context.h"
#include "orrery/render/ui_theme.h"

// Dear ImGui (docking branch) and ImPlot layer (docs/rendering.md, "User
// interface"). ImGui stays private to orrery_render: no public header
// includes it.
//
// Two flavours:
//   - create(context): renders with the OpenGL 3 backend into whatever
//     framebuffer is bound at end_frame(); with attach_window_input the GLFW
//     backend feeds mouse and keyboard from the context's window.
//   - create_headless(): no OpenGL at all; frames are built (layout, draw
//     lists) but not rasterised. Used by unit tests and on machines without
//     a window system.
//
// Each UiContext owns its own ImGui and ImPlot contexts and makes them
// current in begin_frame(). Single-threaded, like GlContext.
namespace orrery::render {

struct UiOptions {
    double font_size_px = 15.0;
    UiTheme theme = default_theme();
};

// Pointer state for frames without the GLFW backend (offscreen, tests).
struct UiPointer {
    double x = -1.0; // [px], negative: outside the window
    double y = -1.0;
    std::array<bool, 3> buttons{}; // left, right, middle
    double wheel = 0.0;
};

struct UiFrameInput {
    int width = 0; // display size [px]
    int height = 0;
    double delta_time = 1.0 / 60.0;   // [s]
    std::optional<UiPointer> pointer; // ignored when window input is attached
};

struct UiRect {
    double x = 0.0;
    double y = 0.0;
    double width = 0.0;
    double height = 0.0;
};

class UiContext {
public:
    [[nodiscard]] static core::Result<UiContext, std::string>
    create_headless(const UiOptions& options = {});

    // `context` must be valid, is made current for initialisation, and must
    // outlive the UiContext. Fails cleanly (nothing leaked) when the context
    // is invalid or the backend cannot initialise.
    [[nodiscard]] static core::Result<UiContext, std::string>
    create(GlContext& context, const UiOptions& options = {}, bool attach_window_input = false);

    UiContext(UiContext&& other) noexcept;
    UiContext& operator=(UiContext&& other) noexcept;
    UiContext(const UiContext&) = delete;
    UiContext& operator=(const UiContext&) = delete;
    // Shuts the backends down with the GL context current (made current for
    // the purpose; the previously current context is restored afterwards,
    // and if none was current, none is current afterwards).
    ~UiContext();

    // Starts a frame; panels may be drawn until end_frame().
    void begin_frame(const UiFrameInput& input);
    // Full-viewport dockspace with a transparent central node and the default
    // layout (built on the first call): Body Inspector on the left, Render
    // Settings on the right, System Monitor at the bottom between them.
    // Returns the central (scene) area.
    UiRect dockspace();
    // Finishes the frame; with OpenGL, renders it into the bound framebuffer.
    void end_frame();

    // Placement of a window in the last finished frame.
    [[nodiscard]] std::optional<UiRect> window_rect(std::string_view name) const;
    // Whether the window is docked into the dockspace.
    [[nodiscard]] bool window_docked(std::string_view name) const;
    // Vertices submitted in the last finished frame.
    [[nodiscard]] std::size_t last_vertex_count() const noexcept;
    // True when the UI wants the pointer (the scene should ignore it).
    [[nodiscard]] bool wants_pointer() const;

    [[nodiscard]] bool has_renderer() const noexcept;
    [[nodiscard]] const UiTheme& theme() const noexcept;

private:
    struct Impl;
    explicit UiContext(std::unique_ptr<Impl> impl) noexcept;
    void release() noexcept;
    std::unique_ptr<Impl> impl_;
};

} // namespace orrery::render
