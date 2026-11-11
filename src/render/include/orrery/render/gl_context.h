#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include "orrery/core/result.h"

// OpenGL 4.6 core context on a GLFW window (visible, or hidden for offscreen
// and headless rendering). GLFW and glad stay private to orrery_render.
//
// Threading: create, use and destroy contexts on one thread. GLFW is
// initialised by the first live context and terminated with the last one.
namespace orrery::render {

struct ContextOptions {
    int width = 64;
    int height = 64;
    std::string title = "Orrery";
    // false: hidden window; rendering then goes to framebuffer objects only.
    bool visible = false;
    // Request a debug context and route KHR_debug messages to log_message().
    bool debug = true;
    // Swap interval for visible windows (1 = vsync).
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

private:
    struct Impl;
    explicit GlContext(std::unique_ptr<Impl> impl) noexcept;
    std::unique_ptr<Impl> impl_;
};

} // namespace orrery::render
