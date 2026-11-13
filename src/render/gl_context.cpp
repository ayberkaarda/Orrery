#include "orrery/render/gl_context.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <utility>
#include <vector>

#include "gl_context_internal.h"
#include "gl_loader.h"
#include "orrery/render/log.h"

namespace orrery::render {

namespace {

// GLFW is a process-wide library; the first live context initialises it and
// the last one terminates it. Contexts are single-threaded (see header).
int g_glfw_users = 0;
std::string g_last_glfw_error;
std::uint64_t g_last_context_serial = 0;

void glfw_error_callback(int code, const char* description) {
    g_last_glfw_error = "GLFW error " + std::to_string(code) + ": " +
                        (description != nullptr ? description : "(no description)");
}

std::string take_glfw_error() {
    std::string error = std::exchange(g_last_glfw_error, std::string{});
    return error.empty() ? std::string("no GLFW error reported") : error;
}

// Holds one reference on the GLFW library.
class GlfwLibrary {
public:
    GlfwLibrary() = default;
    GlfwLibrary(const GlfwLibrary&) = delete;
    GlfwLibrary& operator=(const GlfwLibrary&) = delete;
    GlfwLibrary(GlfwLibrary&&) = delete;
    GlfwLibrary& operator=(GlfwLibrary&&) = delete;
    ~GlfwLibrary() {
        if (acquired_ && --g_glfw_users == 0) {
            glfwTerminate();
        }
    }

    bool acquire() {
        if (g_glfw_users == 0) {
            glfwSetErrorCallback(glfw_error_callback);
            if (glfwInit() != GLFW_TRUE) {
                return false;
            }
        }
        ++g_glfw_users;
        acquired_ = true;
        return true;
    }

private:
    bool acquired_ = false;
};

struct DebugCounter {
    mutable std::atomic<std::size_t> errors{0};
};

// Stored as the GLFW window user pointer: identifies the context of a window.
struct ContextIdentity {
    std::uint64_t serial = 0;
};

struct WindowDeleter {
    void operator()(GLFWwindow* window) const noexcept { glfwDestroyWindow(window); }
};

std::string gl_string(GLenum name) {
    const GLubyte* value = glGetString(name);
    // GL returns strings as unsigned char; the bytes are ASCII/UTF-8 text.
    return value != nullptr ? std::string(reinterpret_cast<const char*>(value)) : std::string{};
}

std::string_view debug_source_name(GLenum source) {
    switch (source) {
        case GL_DEBUG_SOURCE_API:
            return "api";
        case GL_DEBUG_SOURCE_WINDOW_SYSTEM:
            return "window-system";
        case GL_DEBUG_SOURCE_SHADER_COMPILER:
            return "shader-compiler";
        case GL_DEBUG_SOURCE_THIRD_PARTY:
            return "third-party";
        case GL_DEBUG_SOURCE_APPLICATION:
            return "application";
        default:
            return "other";
    }
}

std::string_view debug_type_name(GLenum type) {
    switch (type) {
        case GL_DEBUG_TYPE_ERROR:
            return "error";
        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
            return "deprecated";
        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
            return "undefined-behavior";
        case GL_DEBUG_TYPE_PORTABILITY:
            return "portability";
        case GL_DEBUG_TYPE_PERFORMANCE:
            return "performance";
        default:
            return "other";
    }
}

} // namespace

struct GlContext::Impl {
    GlfwLibrary library; // declared first: released after the window
    std::unique_ptr<GLFWwindow, WindowDeleter> window;
    GlInfo info;
    DebugCounter debug;
    ContextIdentity identity;
};

namespace {

void APIENTRY debug_callback(GLenum source, GLenum type, GLuint id, GLenum severity,
                             GLsizei /*length*/, const GLchar* message, const void* user) {
    const bool is_error = type == GL_DEBUG_TYPE_ERROR || severity == GL_DEBUG_SEVERITY_HIGH;
    if (is_error && user != nullptr) {
        // The user pointer is the counter registered in GlContext::create.
        static_cast<const DebugCounter*>(user)->errors.fetch_add(1);
    }
    LogLevel level = LogLevel::info;
    if (is_error) {
        level = LogLevel::error;
    } else if (severity == GL_DEBUG_SEVERITY_MEDIUM) {
        level = LogLevel::warning;
    }
    std::string text = "GL debug [";
    text += debug_source_name(source);
    text += '/';
    text += debug_type_name(type);
    text += " id " + std::to_string(id) + "]: ";
    text += message != nullptr ? message : "(no message)";
    log_message(level, text);
}

} // namespace

core::Result<GlContext, std::string> GlContext::create(const ContextOptions& options) {
    auto impl = std::make_unique<Impl>();
    if (!impl->library.acquire()) {
        return core::failure("cannot initialise GLFW (no window system available?): " +
                             take_glfw_error());
    }

    glfwDefaultWindowHints();
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_CONTEXT_DEBUG, options.debug ? GLFW_TRUE : GLFW_FALSE);
    glfwWindowHint(GLFW_VISIBLE, options.visible ? GLFW_TRUE : GLFW_FALSE);
    glfwWindowHint(GLFW_FOCUS_ON_SHOW, options.visible ? GLFW_TRUE : GLFW_FALSE);
    glfwWindowHint(GLFW_FOCUSED, options.visible ? GLFW_TRUE : GLFW_FALSE);
    impl->window.reset(
        glfwCreateWindow(options.width, options.height, options.title.c_str(), nullptr, nullptr));
    if (!impl->window) {
        return core::failure("cannot create an OpenGL 4.6 core context: " + take_glfw_error());
    }
    impl->identity.serial = ++g_last_context_serial;
    glfwSetWindowUserPointer(impl->window.get(), &impl->identity);
    glfwMakeContextCurrent(impl->window.get());

    // Loader boundary: GLFW returns a generic function pointer that glad
    // stores in its typed entry points.
    const auto loader = [](const char* name) -> void* {
        return reinterpret_cast<void*>(glfwGetProcAddress(name));
    };
    if (gladLoadGLLoader(loader) == 0) {
        return core::failure(std::string("glad could not load the OpenGL entry points"));
    }
    if (GLAD_GL_VERSION_4_6 == 0) {
        return core::failure("the driver provides OpenGL " + gl_string(GL_VERSION) +
                             "; OpenGL 4.6 core is required");
    }

    GlInfo& info = impl->info;
    info.vendor = gl_string(GL_VENDOR);
    info.renderer = gl_string(GL_RENDERER);
    info.version = gl_string(GL_VERSION);
    info.glsl_version = gl_string(GL_SHADING_LANGUAGE_VERSION);
    glGetIntegerv(GL_MAJOR_VERSION, &info.major);
    glGetIntegerv(GL_MINOR_VERSION, &info.minor);

    if (options.debug) {
        GLint flags = 0;
        glGetIntegerv(GL_CONTEXT_FLAGS, &flags);
        if ((static_cast<GLuint>(flags) & GL_CONTEXT_FLAG_DEBUG_BIT) != 0) {
            glEnable(GL_DEBUG_OUTPUT);
            glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
            glDebugMessageCallback(debug_callback, &impl->debug);
            // Notifications (buffer placement hints and similar) are noise.
            glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DEBUG_SEVERITY_NOTIFICATION, 0,
                                  nullptr, GL_FALSE);
        } else {
            log_message(LogLevel::warning,
                        "debug context requested but not provided; KHR_debug output disabled");
        }
    }
    glfwSwapInterval(options.swap_interval);
    return GlContext(std::move(impl));
}

GlContext::GlContext(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}
GlContext::GlContext(GlContext&& other) noexcept = default;
GlContext& GlContext::operator=(GlContext&& other) noexcept = default;

GlContext::~GlContext() {
    if (impl_ && impl_->window) {
        // Detach before the window (and with it the context) is destroyed.
        if (glfwGetCurrentContext() == impl_->window.get()) {
            if (impl_->info.major != 0) {
                glDebugMessageCallback(nullptr, nullptr);
            }
            glfwMakeContextCurrent(nullptr);
        }
    }
}

void GlContext::make_current() const {
    if (impl_) {
        glfwMakeContextCurrent(impl_->window.get());
    }
}

bool GlContext::is_current() const noexcept {
    return impl_ && detail::current_native_context() == impl_->window.get();
}

const GlInfo& GlContext::info() const noexcept {
    return impl_->info;
}

std::size_t GlContext::debug_error_count() const noexcept {
    return impl_ ? impl_->debug.errors.load() : 0;
}

void GlContext::reset_debug_error_count() noexcept {
    if (impl_) {
        impl_->debug.errors.store(0);
    }
}

void GlContext::swap_buffers() {
    if (impl_) {
        glfwSwapBuffers(impl_->window.get());
    }
}

void GlContext::poll_events() {
    glfwPollEvents();
}

bool GlContext::should_close() const {
    return impl_ && glfwWindowShouldClose(impl_->window.get()) == GLFW_TRUE;
}

int GlContext::framebuffer_width() const {
    int width = 0;
    int height = 0;
    if (impl_) {
        glfwGetFramebufferSize(impl_->window.get(), &width, &height);
    }
    return width;
}

int GlContext::framebuffer_height() const {
    int width = 0;
    int height = 0;
    if (impl_) {
        glfwGetFramebufferSize(impl_->window.get(), &width, &height);
    }
    return height;
}

void GlContext::set_window_size(int width, int height) {
    if (impl_ && width > 0 && height > 0) {
        glfwSetWindowSize(impl_->window.get(), width, height);
    }
}

ContentScale GlContext::content_scale() const {
    ContentScale scale;
    if (impl_) {
        float x = 1.0F;
        float y = 1.0F;
        glfwGetWindowContentScale(impl_->window.get(), &x, &y);
        scale.x = static_cast<double>(x);
        scale.y = static_cast<double>(y);
    }
    return scale;
}

std::size_t GlContext::drain_gl_errors() const {
    std::size_t errors = 0;
    if (impl_) {
        while (errors < 64 && glGetError() != GL_NO_ERROR) {
            ++errors;
        }
    }
    return errors;
}

ImageRgba8 GlContext::read_window_pixels(int width, int height) const {
    ImageRgba8 image;
    if (!impl_ || width <= 0 || height <= 0) {
        return image;
    }
    image.width = static_cast<std::uint32_t>(width);
    image.height = static_cast<std::uint32_t>(height);
    const std::size_t row = static_cast<std::size_t>(image.width) * 4U;
    std::vector<std::uint8_t> bottom_up(image.byte_size());
    GLint read_framebuffer = 0;
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &read_framebuffer);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glReadBuffer(GL_BACK);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, bottom_up.data());
    glBindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(read_framebuffer));
    image.pixels.resize(bottom_up.size());
    for (std::size_t y = 0; y < image.height; ++y) {
        const std::size_t source = (image.height - 1U - y) * row;
        std::copy_n(bottom_up.begin() + static_cast<std::ptrdiff_t>(source), row,
                    image.pixels.begin() + static_cast<std::ptrdiff_t>(y * row));
    }
    return image;
}

namespace detail {

void* current_native_context() noexcept {
    // Without a live context GLFW may be uninitialised; asking it would only
    // record a GLFW_NOT_INITIALIZED error.
    return g_glfw_users > 0 ? glfwGetCurrentContext() : nullptr;
}

void make_native_context_current(void* context) noexcept {
    if (g_glfw_users > 0) {
        glfwMakeContextCurrent(static_cast<GLFWwindow*>(context));
    }
}

void* native_window(const GlContext& context) noexcept {
    return context.impl_ ? context.impl_->window.get() : nullptr;
}

std::uint64_t current_context_serial() noexcept {
    auto* window = static_cast<GLFWwindow*>(current_native_context());
    if (window == nullptr) {
        return 0;
    }
    const auto* identity = static_cast<const ContextIdentity*>(glfwGetWindowUserPointer(window));
    return identity != nullptr ? identity->serial : 0;
}

} // namespace detail

} // namespace orrery::render
