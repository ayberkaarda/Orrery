#include "orrery/render/gl_resources.h"

#include <array>
#include <atomic>
#include <limits>

#include <glm/gtc/type_ptr.hpp>

#include "gl_context_internal.h"
#include "gl_loader.h"
#include "orrery/render/log.h"

namespace orrery::render {

namespace detail {

namespace {

std::atomic<std::size_t> g_live_handles{0};
std::atomic<std::size_t> g_skipped_deletes{0};
thread_local int t_simulated_allocation_failures = 0;

} // namespace

SimulatedAllocationFailure::SimulatedAllocationFailure() noexcept {
    ++t_simulated_allocation_failures;
}

SimulatedAllocationFailure::~SimulatedAllocationFailure() {
    --t_simulated_allocation_failures;
}

namespace {

bool allocation_failure_simulated() noexcept {
    return t_simulated_allocation_failures > 0;
}

} // namespace

GlHandle::GlHandle(std::uint32_t id, GlDeleter deleter) noexcept
    : id_(id), deleter_(deleter), context_(id != 0 ? current_context_serial() : 0) {
    if (id_ != 0) {
        g_live_handles.fetch_add(1);
    }
}

GlHandle::GlHandle(GlHandle&& other) noexcept
    : id_(std::exchange(other.id_, 0U)), deleter_(std::exchange(other.deleter_, nullptr)),
      context_(std::exchange(other.context_, 0U)) {}

GlHandle& GlHandle::operator=(GlHandle&& other) noexcept {
    if (this != &other) {
        reset();
        id_ = std::exchange(other.id_, 0U);
        deleter_ = std::exchange(other.deleter_, nullptr);
        context_ = std::exchange(other.context_, 0U);
    }
    return *this;
}

void GlHandle::reset() noexcept {
    if (id_ != 0) {
        if (context_ != 0 && current_context_serial() == context_) {
            if (deleter_ != nullptr) {
                deleter_(id_);
            }
        } else {
            g_skipped_deletes.fetch_add(1);
            log_message(LogLevel::error,
                        "GL object not deleted: the context that created it is not current "
                        "(it is released with that context)");
        }
        g_live_handles.fetch_sub(1);
    }
    id_ = 0;
    deleter_ = nullptr;
    context_ = 0;
}

GlHandleStats gl_handle_stats() noexcept {
    return {g_live_handles.load(), g_skipped_deletes.load()};
}

} // namespace detail

namespace {

void delete_shader(std::uint32_t id) {
    glDeleteShader(id);
}
void delete_program(std::uint32_t id) {
    glDeleteProgram(id);
}
void delete_buffer(std::uint32_t id) {
    glDeleteBuffers(1, &id);
}
void delete_vertex_array(std::uint32_t id) {
    glDeleteVertexArrays(1, &id);
}
void delete_texture(std::uint32_t id) {
    glDeleteTextures(1, &id);
}
void delete_framebuffer(std::uint32_t id) {
    glDeleteFramebuffers(1, &id);
}
void delete_query(std::uint32_t id) {
    glDeleteQueries(1, &id);
}

GLenum gl_stage(ShaderStage stage) {
    switch (stage) {
        case ShaderStage::vertex:
            return GL_VERTEX_SHADER;
        case ShaderStage::fragment:
            return GL_FRAGMENT_SHADER;
        case ShaderStage::compute:
            return GL_COMPUTE_SHADER;
    }
    return GL_VERTEX_SHADER;
}

std::string_view stage_name(ShaderStage stage) {
    switch (stage) {
        case ShaderStage::vertex:
            return "vertex";
        case ShaderStage::fragment:
            return "fragment";
        case ShaderStage::compute:
            return "compute";
    }
    return "unknown";
}

GLenum gl_internal_format(TextureFormat format) {
    switch (format) {
        case TextureFormat::rgba16f:
            return GL_RGBA16F;
        case TextureFormat::rgba8:
            return GL_RGBA8;
        case TextureFormat::srgb8_alpha8:
            return GL_SRGB8_ALPHA8;
        case TextureFormat::depth32f:
            return GL_DEPTH_COMPONENT32F;
        case TextureFormat::rgba32f:
            return GL_RGBA32F;
        case TextureFormat::r11f_g11f_b10f:
            return GL_R11F_G11F_B10F;
    }
    return GL_RGBA8;
}

std::string_view framebuffer_status_name(GLenum status) {
    switch (status) {
        case GL_FRAMEBUFFER_UNDEFINED:
            return "GL_FRAMEBUFFER_UNDEFINED";
        case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
            return "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT";
        case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
            return "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT";
        case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
            return "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER";
        case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
            return "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER";
        case GL_FRAMEBUFFER_UNSUPPORTED:
            return "GL_FRAMEBUFFER_UNSUPPORTED";
        case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
            return "GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE";
        case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
            return "GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS";
        default:
            return "unknown framebuffer status";
    }
}

// Clears pending error flags so that the next glGetError reports only the
// calls that follow (bounded: a lost context reports an error forever).
void clear_gl_errors() {
    for (int i = 0; i < 64 && glGetError() != GL_NO_ERROR; ++i) {
    }
}

std::string gl_error_name(GLenum error) {
    switch (error) {
        case GL_NO_ERROR:
            return "no GL error";
        case GL_INVALID_ENUM:
            return "GL_INVALID_ENUM";
        case GL_INVALID_VALUE:
            return "GL_INVALID_VALUE";
        case GL_INVALID_OPERATION:
            return "GL_INVALID_OPERATION";
        case GL_OUT_OF_MEMORY:
            return "GL_OUT_OF_MEMORY";
        default:
            return "GL error " + std::to_string(error);
    }
}

std::string allocation_failure_reason(GLenum error) {
    if (error != GL_NO_ERROR) {
        return gl_error_name(error);
    }
    return detail::allocation_failure_simulated() ? "simulated allocation failure"
                                                  : "storage missing after allocation";
}

GLint gl_integer(GLenum name) {
    GLint value = 0;
    glGetIntegerv(name, &value);
    return value;
}

// Creates a buffer object with immutable storage of `size` bytes and checks
// that the storage exists; `wrap` turns the owned name into a Buffer.
template <typename Wrap>
core::Result<Buffer, std::string> make_buffer(const void* data, std::size_t size, bool dynamic,
                                              const Wrap& wrap) {
    if (size == 0) {
        return core::failure(
            std::string("buffer: size 0 requested (OpenGL buffer storage cannot be empty)"));
    }
    if (size > static_cast<std::size_t>(std::numeric_limits<GLsizeiptr>::max())) {
        return core::failure("buffer: size " + std::to_string(size) +
                             " bytes exceeds the GLsizeiptr range");
    }
    clear_gl_errors();
    GLuint id = 0;
    glCreateBuffers(1, &id);
    if (id == 0) {
        return core::failure("buffer: glCreateBuffers failed (" + gl_error_name(glGetError()) +
                             ")");
    }
    detail::GlHandle handle(id, delete_buffer);
    glNamedBufferStorage(id, static_cast<GLsizeiptr>(size), data,
                         dynamic ? GL_DYNAMIC_STORAGE_BIT : 0U);
    const GLenum error = glGetError();
    GLint immutable = GL_FALSE;
    glGetNamedBufferParameteriv(id, GL_BUFFER_IMMUTABLE_STORAGE, &immutable);
    GLint64 stored = 0;
    glGetNamedBufferParameteri64v(id, GL_BUFFER_SIZE, &stored);
    if (error != GL_NO_ERROR || immutable != GL_TRUE || stored != static_cast<GLint64>(size) ||
        detail::allocation_failure_simulated()) {
        // `handle` deletes the name on return.
        return core::failure("buffer: allocating " + std::to_string(size) + " bytes failed (" +
                             allocation_failure_reason(error) + ")");
    }
    return wrap(std::move(handle), size);
}

std::string info_log(GLuint object, bool program) {
    GLint length = 0;
    if (program) {
        glGetProgramiv(object, GL_INFO_LOG_LENGTH, &length);
    } else {
        glGetShaderiv(object, GL_INFO_LOG_LENGTH, &length);
    }
    if (length <= 1) {
        return "(no log)";
    }
    std::string log(static_cast<std::size_t>(length), '\0');
    GLsizei written = 0;
    if (program) {
        glGetProgramInfoLog(object, length, &written, log.data());
    } else {
        glGetShaderInfoLog(object, length, &written, log.data());
    }
    log.resize(static_cast<std::size_t>(written));
    return log;
}

} // namespace

core::Result<Shader, std::string> Shader::compile(const StageSource& source) {
    auto composed = compose_shader(source.main, source.includes);
    if (!composed) {
        return core::failure(composed.error());
    }
    const ComposedShader& parts = composed.value();
    std::vector<const GLchar*> strings;
    std::vector<GLint> lengths;
    for (const std::string& part : parts.strings) {
        strings.push_back(part.data());
        lengths.push_back(static_cast<GLint>(part.size()));
    }

    detail::GlHandle handle(glCreateShader(gl_stage(source.stage)), delete_shader);
    glShaderSource(handle.get(), static_cast<GLsizei>(strings.size()), strings.data(),
                   lengths.data());
    glCompileShader(handle.get());
    GLint status = GL_FALSE;
    glGetShaderiv(handle.get(), GL_COMPILE_STATUS, &status);
    if (status != GL_TRUE) {
        return core::failure("compiling " + std::string(stage_name(source.stage)) + " shader '" +
                             std::string(source.main.name) + "' failed:\n" +
                             annotate_shader_log(info_log(handle.get(), false), parts.names));
    }
    return Shader(std::move(handle));
}

core::Result<Program, std::string> Program::create(std::span<const StageSource> stages) {
    std::vector<Shader> shaders;
    std::string names;
    for (const StageSource& stage : stages) {
        auto shader = Shader::compile(stage);
        if (!shader) {
            return core::failure(shader.error());
        }
        shaders.push_back(std::move(shader).value());
        names += (names.empty() ? "" : ", ") + std::string(stage.main.name);
    }
    detail::GlHandle handle(glCreateProgram(), delete_program);
    for (const Shader& shader : shaders) {
        glAttachShader(handle.get(), shader.id());
    }
    glLinkProgram(handle.get());
    for (const Shader& shader : shaders) {
        glDetachShader(handle.get(), shader.id());
    }
    GLint status = GL_FALSE;
    glGetProgramiv(handle.get(), GL_LINK_STATUS, &status);
    if (status != GL_TRUE) {
        return core::failure("linking program (" + names + ") failed:\n" +
                             info_log(handle.get(), true));
    }
    return Program(std::move(handle));
}

void Program::use() const {
    glUseProgram(handle_.get());
}

int Program::uniform_location(std::string_view name) const {
    const std::string terminated(name);
    return glGetUniformLocation(handle_.get(), terminated.c_str());
}

void Program::set(std::string_view name, int value) const {
    glProgramUniform1i(handle_.get(), uniform_location(name), value);
}

void Program::set(std::string_view name, const glm::ivec2& value) const {
    glProgramUniform2iv(handle_.get(), uniform_location(name), 1, glm::value_ptr(value));
}

void Program::set(std::string_view name, float value) const {
    glProgramUniform1f(handle_.get(), uniform_location(name), value);
}

void Program::set(std::string_view name, const glm::vec2& value) const {
    glProgramUniform2fv(handle_.get(), uniform_location(name), 1, glm::value_ptr(value));
}

void Program::set(std::string_view name, const glm::mat3& value) const {
    glProgramUniformMatrix3fv(handle_.get(), uniform_location(name), 1, GL_FALSE,
                              glm::value_ptr(value));
}

void Program::set(std::string_view name, const glm::vec3& value) const {
    glProgramUniform3fv(handle_.get(), uniform_location(name), 1, glm::value_ptr(value));
}

void Program::set(std::string_view name, const glm::vec4& value) const {
    glProgramUniform4fv(handle_.get(), uniform_location(name), 1, glm::value_ptr(value));
}

void Program::set(std::string_view name, const glm::mat4& value) const {
    glProgramUniformMatrix4fv(handle_.get(), uniform_location(name), 1, GL_FALSE,
                              glm::value_ptr(value));
}

core::Result<Buffer, std::string> Buffer::create(std::span<const std::byte> bytes, bool dynamic) {
    return make_buffer(
        bytes.data(), bytes.size(), dynamic,
        [](detail::GlHandle handle, std::size_t size) { return Buffer(std::move(handle), size); });
}

core::Result<Buffer, std::string> Buffer::create_uninitialized(std::size_t size, bool dynamic) {
    return make_buffer(nullptr, size, dynamic, [](detail::GlHandle handle, std::size_t stored) {
        return Buffer(std::move(handle), stored);
    });
}

void Buffer::update(std::size_t offset, std::span<const std::byte> bytes) const {
    glNamedBufferSubData(handle_.get(), static_cast<GLintptr>(offset),
                         static_cast<GLsizeiptr>(bytes.size()), bytes.data());
}

VertexArray VertexArray::create() {
    GLuint id = 0;
    glCreateVertexArrays(1, &id);
    return VertexArray(detail::GlHandle(id, delete_vertex_array));
}

void VertexArray::set_vertex_buffer(std::uint32_t binding, const Buffer& buffer, std::size_t offset,
                                    std::size_t stride) const {
    glVertexArrayVertexBuffer(handle_.get(), binding, buffer.id(), static_cast<GLintptr>(offset),
                              static_cast<GLsizei>(stride));
}

void VertexArray::set_float_attribute(std::uint32_t location, std::uint32_t binding, int components,
                                      std::uint32_t relative_offset) const {
    glEnableVertexArrayAttrib(handle_.get(), location);
    glVertexArrayAttribFormat(handle_.get(), location, components, GL_FLOAT, GL_FALSE,
                              relative_offset);
    glVertexArrayAttribBinding(handle_.get(), location, binding);
}

void VertexArray::set_binding_divisor(std::uint32_t binding, std::uint32_t divisor) const {
    glVertexArrayBindingDivisor(handle_.get(), binding, divisor);
}

GpuTimer GpuTimer::create() {
    GLuint id = 0;
    glCreateQueries(GL_TIME_ELAPSED, 1, &id);
    return GpuTimer(detail::GlHandle(id, delete_query));
}

void GpuTimer::begin() const {
    glBeginQuery(GL_TIME_ELAPSED, handle_.get());
}

void GpuTimer::end() const {
    glEndQuery(GL_TIME_ELAPSED);
}

std::uint64_t GpuTimer::elapsed_ns() const {
    GLuint64 value = 0;
    glGetQueryObjectui64v(handle_.get(), GL_QUERY_RESULT, &value);
    return value;
}

void VertexArray::set_index_buffer(const Buffer& buffer) const {
    glVertexArrayElementBuffer(handle_.get(), buffer.id());
}

void VertexArray::bind() const {
    glBindVertexArray(handle_.get());
}

core::Result<Texture, std::string> Texture::create_2d(int width, int height, TextureFormat format,
                                                      int samples) {
    const std::string size_text = std::to_string(width) + "x" + std::to_string(height);
    const GLint max_size = gl_integer(GL_MAX_TEXTURE_SIZE);
    if (width < 1 || height < 1 || width > max_size || height > max_size) {
        return core::failure("texture: size " + size_text + " is outside [1, " +
                             std::to_string(max_size) + "]");
    }
    const GLint max_samples =
        gl_integer(format == TextureFormat::depth32f ? GL_MAX_DEPTH_TEXTURE_SAMPLES
                                                     : GL_MAX_COLOR_TEXTURE_SAMPLES);
    if (samples < 1 || (samples > 1 && samples > max_samples)) {
        return core::failure("texture: " + std::to_string(samples) + " samples is outside [1, " +
                             std::to_string(max_samples) + "]");
    }
    clear_gl_errors();
    GLuint id = 0;
    glCreateTextures(samples > 1 ? GL_TEXTURE_2D_MULTISAMPLE : GL_TEXTURE_2D, 1, &id);
    if (id == 0) {
        return core::failure("texture: glCreateTextures failed (" + gl_error_name(glGetError()) +
                             ")");
    }
    detail::GlHandle handle(id, delete_texture);
    const GLenum internal_format = gl_internal_format(format);
    if (samples > 1) {
        glTextureStorage2DMultisample(id, samples, internal_format, width, height, GL_TRUE);
    } else {
        glTextureStorage2D(id, 1, internal_format, width, height);
    }
    const GLenum error = glGetError();
    GLint immutable = GL_FALSE;
    glGetTextureParameteriv(id, GL_TEXTURE_IMMUTABLE_FORMAT, &immutable);
    GLint stored_width = 0;
    GLint stored_height = 0;
    glGetTextureLevelParameteriv(id, 0, GL_TEXTURE_WIDTH, &stored_width);
    glGetTextureLevelParameteriv(id, 0, GL_TEXTURE_HEIGHT, &stored_height);
    if (error != GL_NO_ERROR || immutable != GL_TRUE || stored_width != width ||
        stored_height != height || detail::allocation_failure_simulated()) {
        // `handle` deletes the name on return.
        return core::failure("texture: allocating " + size_text + " with " +
                             std::to_string(samples) + " sample(s) failed (" +
                             allocation_failure_reason(error) + ")");
    }
    if (samples == 1) {
        glTextureParameteri(id, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTextureParameteri(id, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTextureParameteri(id, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTextureParameteri(id, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }
    return Texture(std::move(handle), width, height, samples, format);
}

core::Result<Texture, std::string> Texture::create_cube(int size, TextureFormat format,
                                                        int levels) {
    const std::string size_text = std::to_string(size) + "x" + std::to_string(size);
    const GLint max_size = gl_integer(GL_MAX_CUBE_MAP_TEXTURE_SIZE);
    if (size < 1 || size > max_size) {
        return core::failure("cube map: face size " + size_text + " is outside [1, " +
                             std::to_string(max_size) + "]");
    }
    int max_levels = 1;
    while ((size >> max_levels) > 0) {
        ++max_levels;
    }
    if (levels < 1 || levels > max_levels) {
        return core::failure("cube map: " + std::to_string(levels) + " levels is outside [1, " +
                             std::to_string(max_levels) + "]");
    }
    if (format == TextureFormat::depth32f) {
        return core::failure(std::string("cube map: depth formats are not supported"));
    }
    clear_gl_errors();
    GLuint id = 0;
    glCreateTextures(GL_TEXTURE_CUBE_MAP, 1, &id);
    if (id == 0) {
        return core::failure("cube map: glCreateTextures failed (" + gl_error_name(glGetError()) +
                             ")");
    }
    detail::GlHandle handle(id, delete_texture);
    glTextureStorage2D(id, levels, gl_internal_format(format), size, size);
    const GLenum error = glGetError();
    GLint immutable = GL_FALSE;
    glGetTextureParameteriv(id, GL_TEXTURE_IMMUTABLE_FORMAT, &immutable);
    GLint stored_size = 0;
    glGetTextureLevelParameteriv(id, 0, GL_TEXTURE_WIDTH, &stored_size);
    if (error != GL_NO_ERROR || immutable != GL_TRUE || stored_size != size ||
        detail::allocation_failure_simulated()) {
        return core::failure("cube map: allocating 6 faces of " + size_text + " with " +
                             std::to_string(levels) + " level(s) failed (" +
                             allocation_failure_reason(error) + ")");
    }
    glTextureParameteri(id, GL_TEXTURE_MIN_FILTER,
                        levels > 1 ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
    glTextureParameteri(id, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTextureParameteri(id, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTextureParameteri(id, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTextureParameteri(id, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTextureParameteri(id, GL_TEXTURE_MAX_LEVEL, levels - 1);
    return Texture(std::move(handle), size, size, 1, format);
}

void Texture::bind(std::uint32_t unit) const {
    glBindTextureUnit(unit, handle_.get());
}

Framebuffer Framebuffer::create() {
    GLuint id = 0;
    glCreateFramebuffers(1, &id);
    return Framebuffer(detail::GlHandle(id, delete_framebuffer));
}

void Framebuffer::attach_color(std::uint32_t index, const Texture& texture) const {
    glNamedFramebufferTexture(handle_.get(), GL_COLOR_ATTACHMENT0 + index, texture.id(), 0);
    const std::array<GLenum, 1> buffers{GL_COLOR_ATTACHMENT0 + index};
    glNamedFramebufferDrawBuffers(handle_.get(), 1, buffers.data());
    glNamedFramebufferReadBuffer(handle_.get(), GL_COLOR_ATTACHMENT0 + index);
}

void Framebuffer::attach_depth(const Texture& texture) const {
    glNamedFramebufferTexture(handle_.get(), GL_DEPTH_ATTACHMENT, texture.id(), 0);
}

core::Result<std::monostate, std::string> Framebuffer::check() const {
    const GLenum status = glCheckNamedFramebufferStatus(handle_.get(), GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        return core::failure("framebuffer " + std::to_string(handle_.get()) +
                             " is incomplete: " + std::string(framebuffer_status_name(status)));
    }
    return std::monostate{};
}

void Framebuffer::bind() const {
    glBindFramebuffer(GL_FRAMEBUFFER, handle_.get());
}

} // namespace orrery::render
