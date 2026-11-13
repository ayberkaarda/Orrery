#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

#include "orrery/core/result.h"

// Graphics-free access to a compute device (ADR-006, ADR-019). orrery_aero
// reaches the GPU only through IComputeBackend; the OpenGL 4.6 implementation
// lives in orrery_render (GlComputeBackend) and NullComputeBackend keeps
// buffers in host memory for tests.
//
// Precision at the boundary is explicit: buffers are raw bytes (the caller
// packs binary32 data with float32_pack.h, the only single-precision code in
// orrery_aero), and floating-point uniforms are Float32Uniform values that
// the backend rounds to binary32 exactly once, when it hands them to the
// device. Integer uniforms are passed unchanged.
namespace orrery::aero::compute {

using Status = core::Result<std::monostate, std::string>;

// Handle of a device buffer; 0 is never a valid buffer.
struct BufferId {
    std::uint32_t value = 0;

    [[nodiscard]] constexpr bool valid() const noexcept { return value != 0; }
    friend constexpr bool operator==(BufferId, BufferId) noexcept = default;
};

// A host double that crosses the device boundary as an IEEE binary32 value.
struct Float32Uniform {
    double value = 0.0;
};

using UniformValue = std::variant<std::int32_t, std::uint32_t, Float32Uniform>;

struct Uniform {
    std::string_view name;
    UniformValue value;
};

// Shader storage buffer binding point of a kernel.
struct BufferBinding {
    std::uint32_t binding = 0;
    BufferId buffer;
};

// Number of work items (not work groups) in each dimension; the backend
// rounds up to whole work groups, so kernels must ignore items outside the
// range they were dispatched for.
struct DispatchSize {
    std::uint32_t x = 1;
    std::uint32_t y = 1;
    std::uint32_t z = 1;
};

// One kernel launch. Every view must stay valid for the duration of the
// dispatch() call only.
struct DispatchCommand {
    std::string_view kernel;
    DispatchSize global;
    std::span<const Uniform> uniforms;
    std::span<const BufferBinding> buffers;
};

struct ComputeBackendInfo {
    std::string name;   // "null", "opengl"
    std::string device; // driver renderer string, or "host memory"
    bool executes_kernels = false;
};

// Device memory as far as the backend can tell (ADR-020); an empty field is
// unknown. Solvers size their grids from it, or fall back to a conservative
// cap of their own when it is empty.
struct DeviceMemory {
    // Free device memory in bytes (vendor query; drivers report it only
    // through extensions, and only approximately).
    std::optional<std::size_t> available_bytes;
    // Largest buffer one kernel binding can address, bytes.
    std::optional<std::size_t> max_binding_bytes;
};

// Ordering: dispatches run in submission order, but a dispatch only sees the
// shader storage writes of an earlier dispatch after barrier(). upload() and
// readback() are ordered after every dispatch submitted before them (the
// backend inserts whatever synchronisation that needs), and readback()
// returns once the data is on the host.
class IComputeBackend {
public:
    IComputeBackend() = default;
    IComputeBackend(const IComputeBackend&) = delete;
    IComputeBackend& operator=(const IComputeBackend&) = delete;
    IComputeBackend(IComputeBackend&&) = delete;
    IComputeBackend& operator=(IComputeBackend&&) = delete;
    virtual ~IComputeBackend() = default;

    [[nodiscard]] virtual ComputeBackendInfo info() const = 0;
    [[nodiscard]] virtual bool has_kernel(std::string_view name) const = 0;
    // Memory limits of the device; the default knows nothing.
    [[nodiscard]] virtual DeviceMemory memory() const { return {}; }

    // Zero-initialised buffer of `bytes` bytes (bytes > 0).
    [[nodiscard]] virtual core::Result<BufferId, std::string> create_buffer(std::size_t bytes) = 0;
    // Releasing an unknown or invalid id does nothing.
    virtual void release_buffer(BufferId buffer) noexcept = 0;
    [[nodiscard]] virtual std::size_t buffer_size(BufferId buffer) const noexcept = 0;

    // Fail when the buffer is unknown or [offset, offset + size) is outside it.
    [[nodiscard]] virtual Status upload(BufferId buffer, std::size_t offset,
                                        std::span<const std::byte> bytes) = 0;
    [[nodiscard]] virtual Status readback(BufferId buffer, std::size_t offset,
                                          std::span<std::byte> bytes) = 0;

    // Fails for an unknown kernel, an unknown buffer, an empty dispatch or a
    // uniform whose type differs from the kernel's declaration. Uniforms the
    // kernel does not declare (or the compiler removed) are ignored, so one
    // uniform set can serve several kernels.
    [[nodiscard]] virtual Status dispatch(const DispatchCommand& command) = 0;

    // Makes the shader storage writes of every earlier dispatch visible to
    // later dispatches.
    virtual void barrier() = 0;
};

// Owns one buffer of a backend; the backend must outlive it.
class ScopedBuffer {
public:
    ScopedBuffer() = default;
    ScopedBuffer(IComputeBackend& backend, BufferId id) noexcept : backend_(&backend), id_(id) {}
    ScopedBuffer(ScopedBuffer&& other) noexcept
        : backend_(other.backend_), id_(std::exchange(other.id_, BufferId{})) {}
    ScopedBuffer& operator=(ScopedBuffer&& other) noexcept {
        if (this != &other) {
            reset();
            backend_ = other.backend_;
            id_ = std::exchange(other.id_, BufferId{});
        }
        return *this;
    }
    ScopedBuffer(const ScopedBuffer&) = delete;
    ScopedBuffer& operator=(const ScopedBuffer&) = delete;
    ~ScopedBuffer() { reset(); }

    [[nodiscard]] BufferId id() const noexcept { return id_; }
    void reset() noexcept {
        if (backend_ != nullptr && id_.valid()) {
            backend_->release_buffer(id_);
        }
        id_ = BufferId{};
    }

private:
    IComputeBackend* backend_ = nullptr;
    BufferId id_;
};

// Creates a buffer owned by the returned ScopedBuffer.
[[nodiscard]] core::Result<ScopedBuffer, std::string> make_buffer(IComputeBackend& backend,
                                                                  std::size_t bytes);

} // namespace orrery::aero::compute
