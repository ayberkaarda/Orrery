#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include "orrery/core/result.h"
#include "orrery/render/image.h"

// OpenGL 4.6 core context on a GLFW window (visible, or hidden for offscreen
// and headless rendering). GLFW and glad stay private to orrery_render.
//
// Threading: create, use and destroy contexts on one thread. GLFW is
// initialised by the first live context and terminated with the last one.
namespace orrery::render {

class GlContext;

namespace detail {
// Private to orrery_render (UI backend): the native window of a context
// (GLFWwindow*), nullptr for a moved-from context.
[[nodiscard]] void* native_window(const GlContext& context) noexcept;
} // namespace detail

struct ContextOptions {
    int width = 64;
    int height = 64;
    std::string title = "Orrery";
    // false: hidden window that never takes the input focus; rendering goes
    // to framebuffer objects, or to its default framebuffer, which is read
    // back by read_window_pixels() and never shown.
    bool visible = false;
    // Request a debug context and route KHR_debug messages to log_message().
    bool debug = true;
    // Swap interval of the window (1 = vsync); a hidden window that is never
    // swapped is unaffected.
    int swap_interval = 1;
};

struct GlInfo {
    std::string vendor;
    std::string renderer;
    std::string version;
    std::string glsl_version;
    int major = 0;
    int minor = 0;
};

struct ContentScale {
    double x = 1.0;
    double y = 1.0;
};

class GlContext {
public:
    // Fails with a readable message when no window system is available or the
    // driver does not provide OpenGL 4.6 core.
    [[nodiscard]] static core::Result<GlContext, std::string> create(const ContextOptions& options);

    GlContext(GlContext&& other) noexcept;
    GlContext& operator=(GlContext&& other) noexcept;
    GlContext(const GlContext&) = delete;
    GlContext& operator=(const GlContext&) = delete;
    ~GlContext();

    void make_current() const;
    // False after the context was moved from.
    [[nodiscard]] bool valid() const noexcept { return impl_ != nullptr; }
    // Whether this context is current on the calling thread.
    [[nodiscard]] bool is_current() const noexcept;
    [[nodiscard]] const GlInfo& info() const noexcept;

    // KHR_debug messages received with type ERROR or severity HIGH since the
    // context was created (or since the last reset).
    [[nodiscard]] std::size_t debug_error_count() const noexcept;
    void reset_debug_error_count() noexcept;

    // Window helpers for interactive use.
    void swap_buffers();
    void poll_events();
    [[nodiscard]] bool should_close() const;
    [[nodiscard]] int framebuffer_width() const;
    [[nodiscard]] int framebuffer_height() const;
    // Resizes the window's client area (both > 0); a hidden window stays
    // hidden. The framebuffer follows at once or at the next poll_events().
    void set_window_size(int width, int height);
    // The window system's content scale (DPI over the platform default; 1
    // on a standard-density display), {1, 1} for a moved-from context.
    [[nodiscard]] ContentScale content_scale() const;

    // Clears the context's OpenGL error flags and returns how many were set
    // (glGetError until GL_NO_ERROR, at most 64). Requires the context to be
    // current.
    [[nodiscard]] std::size_t drain_gl_errors() const;
    // Reads the back buffer of the window's default framebuffer, which holds
    // the frame the next swap_buffers() shows: width x height pixels from the
    // bottom left, rows top to bottom. Requires the context to be current.
    [[nodiscard]] ImageRgba8 read_window_pixels(int width, int height) const;

private:
    friend void* detail::native_window(const GlContext& context) noexcept;
    struct Impl;
    explicit GlContext(std::unique_ptr<Impl> impl) noexcept;
    std::unique_ptr<Impl> impl_;
};

} // namespace orrery::render
