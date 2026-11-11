#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/ext/vector_float4.hpp>

#include "orrery/core/result.h"
#include "orrery/render/shader_compose.h"

// RAII wrappers for OpenGL 4.6 objects (direct state access). Object names
// are plain 32-bit integers in this header so that no OpenGL or loader
// header leaks into users of orrery_render. Every wrapper is move-only,
// deletes its object on destruction and must be used while the GlContext
// that created it is current.
//
// Contexts do not share objects, so a name is only meaningful in the context
// that created it. Each wrapper records that context; its destructor deletes
// the object only while that context is current. Otherwise (the context is
// not current, or already destroyed, which frees its objects) the delete is
// skipped, counted in gl_handle_stats() and logged as an error, because the
// same name may belong to a different object in the current context.
namespace orrery::render {

namespace detail {

using GlDeleter = void (*)(std::uint32_t);

// Owns one OpenGL object name of the context current at construction.
class GlHandle {
public:
    GlHandle() = default;
    GlHandle(std::uint32_t id, GlDeleter deleter) noexcept;
    GlHandle(GlHandle&& other) noexcept;
    GlHandle& operator=(GlHandle&& other) noexcept;
    GlHandle(const GlHandle&) = delete;
    GlHandle& operator=(const GlHandle&) = delete;
    ~GlHandle() { reset(); }

    [[nodiscard]] std::uint32_t get() const noexcept { return id_; }
    void reset() noexcept;

private:
    std::uint32_t id_ = 0;
    GlDeleter deleter_ = nullptr;
    std::uint64_t context_ = 0; // serial of the owning GlContext
};

// Process-wide diagnostics of GlHandle ownership.
struct GlHandleStats {
    std::size_t live = 0;            // names currently owned by wrappers
    std::size_t skipped_deletes = 0; // deletes skipped: owning context not current
};
[[nodiscard]] GlHandleStats gl_handle_stats() noexcept;

// Test hook for the allocation failure path: while an instance is alive on
// the calling thread, every Buffer and Texture storage allocation is treated
// as failed after the storage call (as if the driver had reported
// GL_OUT_OF_MEMORY). Drivers commonly accept even absurd sizes and commit
// memory lazily, so a real failure cannot be provoked reliably.
class SimulatedAllocationFailure {
public:
    SimulatedAllocationFailure() noexcept;
    SimulatedAllocationFailure(const SimulatedAllocationFailure&) = delete;
    SimulatedAllocationFailure& operator=(const SimulatedAllocationFailure&) = delete;
    SimulatedAllocationFailure(SimulatedAllocationFailure&&) = delete;
    SimulatedAllocationFailure& operator=(SimulatedAllocationFailure&&) = delete;
    ~SimulatedAllocationFailure();
};

} // namespace detail

enum class ShaderStage : std::uint8_t { vertex, fragment, compute };

// One stage: a main file with a #version line plus snippets composed in
// after it (see shader_compose.h).
struct StageSource {
    ShaderStage stage = ShaderStage::vertex;
    ShaderSource main;
    std::vector<ShaderSource> includes;
};

class Shader {
public:
    // On failure the message names the stage and contains the driver log with
    // references rewritten to "file.glsl:line".
    [[nodiscard]] static core::Result<Shader, std::string> compile(const StageSource& source);
    [[nodiscard]] std::uint32_t id() const noexcept { return handle_.get(); }

private:
    explicit Shader(detail::GlHandle handle) noexcept : handle_(std::move(handle)) {}
    detail::GlHandle handle_;
};

class Program {
public:
    // Compiles every stage and links them.
    [[nodiscard]] static core::Result<Program, std::string>
    create(std::span<const StageSource> stages);

    void use() const;
    [[nodiscard]] std::uint32_t id() const noexcept { return handle_.get(); }
    // -1 if the uniform does not exist or was optimised out.
    [[nodiscard]] int uniform_location(std::string_view name) const;

    // Setters ignore uniforms that do not exist (location -1), as OpenGL does.
    void set(std::string_view name, int value) const;
    void set(std::string_view name, float value) const;
    void set(std::string_view name, const glm::vec3& value) const;
    void set(std::string_view name, const glm::vec4& value) const;
    void set(std::string_view name, const glm::mat4& value) const;

private:
    explicit Program(detail::GlHandle handle) noexcept : handle_(std::move(handle)) {}
    detail::GlHandle handle_;
};

class Buffer {
public:
    // Immutable storage initialised with `bytes`; `dynamic` allows update().
    // Fails without creating anything for an empty span (OpenGL has no
    // zero-size storage) or a size beyond GLsizeiptr, and fails with the
    // buffer deleted when the driver cannot allocate the storage.
    [[nodiscard]] static core::Result<Buffer, std::string> create(std::span<const std::byte> bytes,
                                                                  bool dynamic = false);
    // Immutable storage of `size` bytes with undefined contents; same rules.
    [[nodiscard]] static core::Result<Buffer, std::string>
    create_uninitialized(std::size_t size, bool dynamic = false);
    template <typename T>
    [[nodiscard]] static core::Result<Buffer, std::string> from(std::span<const T> values,
                                                                bool dynamic = false) {
        return create(std::as_bytes(values), dynamic);
    }

    void update(std::size_t offset, std::span<const std::byte> bytes) const;
    [[nodiscard]] std::uint32_t id() const noexcept { return handle_.get(); }
    [[nodiscard]] std::size_t size() const noexcept { return size_; }

private:
    Buffer(detail::GlHandle handle, std::size_t size) noexcept
        : handle_(std::move(handle)), size_(size) {}
    detail::GlHandle handle_;
    std::size_t size_ = 0;
};

class VertexArray {
public:
    [[nodiscard]] static VertexArray create();

    // Binds `buffer` to a vertex buffer binding point.
    void set_vertex_buffer(std::uint32_t binding, const Buffer& buffer, std::size_t offset,
                           std::size_t stride) const;
    // Float attribute with `components` components read at `relative_offset`
    // bytes inside each vertex of `binding`.
    void set_float_attribute(std::uint32_t location, std::uint32_t binding, int components,
                             std::uint32_t relative_offset) const;
    void set_index_buffer(const Buffer& buffer) const;
    void bind() const;
    [[nodiscard]] std::uint32_t id() const noexcept { return handle_.get(); }

private:
    explicit VertexArray(detail::GlHandle handle) noexcept : handle_(std::move(handle)) {}
    detail::GlHandle handle_;
};

enum class TextureFormat : std::uint8_t {
    rgba16f,      // HDR colour
    rgba8,        // linear 8-bit colour
    srgb8_alpha8, // sRGB-encoded 8-bit colour
    depth32f,     // 32-bit float depth (log depth)
};

class Texture {
public:
    // 2D texture (samples == 1) or 2D multisample texture (samples > 1).
    // Single-sample textures use linear filtering and clamp-to-edge. Fails
    // without creating anything when width or height is outside
    // [1, GL_MAX_TEXTURE_SIZE] or samples outside [1, the format's sample
    // limit], and fails with the texture deleted when the driver cannot
    // allocate the storage.
    [[nodiscard]] static core::Result<Texture, std::string>
    create_2d(int width, int height, TextureFormat format, int samples = 1);

    void bind(std::uint32_t unit) const;
    [[nodiscard]] std::uint32_t id() const noexcept { return handle_.get(); }
    [[nodiscard]] int width() const noexcept { return width_; }
    [[nodiscard]] int height() const noexcept { return height_; }
    [[nodiscard]] int samples() const noexcept { return samples_; }
    [[nodiscard]] TextureFormat format() const noexcept { return format_; }

private:
    Texture(detail::GlHandle handle, int width, int height, int samples,
            TextureFormat format) noexcept
        : handle_(std::move(handle)), width_(width), height_(height), samples_(samples),
          format_(format) {}
    detail::GlHandle handle_;
    int width_ = 0;
    int height_ = 0;
    int samples_ = 1;
    TextureFormat format_ = TextureFormat::rgba8;
};

class Framebuffer {
public:
    [[nodiscard]] static Framebuffer create();

    void attach_color(std::uint32_t index, const Texture& texture) const;
    void attach_depth(const Texture& texture) const;
    // Completeness check with the status name in the error message.
    [[nodiscard]] core::Result<std::monostate, std::string> check() const;
    void bind() const;
    [[nodiscard]] std::uint32_t id() const noexcept { return handle_.get(); }

private:
    explicit Framebuffer(detail::GlHandle handle) noexcept : handle_(std::move(handle)) {}
    detail::GlHandle handle_;
};

} // namespace orrery::render
