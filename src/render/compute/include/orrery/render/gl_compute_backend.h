#pragma once

#include <cstddef>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "orrery/aero/compute/compute_backend.h"
#include "orrery/core/result.h"

// OpenGL 4.6 implementation of orrery_aero's IComputeBackend (ADR-006,
// ADR-019). Buffers are shader storage buffers with immutable storage;
// kernels are the GLSL 460 compute shaders embedded from
// src/render/shaders/lbm: kernel "<name>" is the file <name>.comp.glsl,
// composed with the snippets of that directory (files without ".comp") that
// belong to it: "<family>_common.glsl" goes into the kernels named
// "<family>_*", any other snippet into every kernel (ADR-020).
//
// The backend belongs to the OpenGL context that was current when it was
// created: every call fails (or, for release_buffer, does nothing) while
// another context is current, and it must be destroyed while its context is
// current (GL objects of a context that is not current are not deleted; see
// gl_resources.h). Floating-point uniforms are rounded to binary32 here
// (aero::compute::round_to_float32); dispatch fails for a finite value
// outside the binary32 range.
namespace orrery::render {

class GlComputeBackend final : public aero::compute::IComputeBackend {
public:
    // Requires a current OpenGL 4.6 core context (GlContext).
    [[nodiscard]] static core::Result<std::unique_ptr<GlComputeBackend>, std::string> create();
    ~GlComputeBackend() override;
    GlComputeBackend(const GlComputeBackend&) = delete;
    GlComputeBackend& operator=(const GlComputeBackend&) = delete;
    GlComputeBackend(GlComputeBackend&&) = delete;
    GlComputeBackend& operator=(GlComputeBackend&&) = delete;

    [[nodiscard]] aero::compute::ComputeBackendInfo info() const override;
    [[nodiscard]] bool has_kernel(std::string_view name) const override;
    // Largest shader storage block (GL_MAX_SHADER_STORAGE_BLOCK_SIZE) and,
    // where the driver offers GL_NVX_gpu_memory_info or GL_ATI_meminfo, the
    // free device memory (read now; empty while the context is not current).
    [[nodiscard]] aero::compute::DeviceMemory memory() const override;
    [[nodiscard]] core::Result<aero::compute::BufferId, std::string>
    create_buffer(std::size_t bytes) override;
    void release_buffer(aero::compute::BufferId buffer) noexcept override;
    [[nodiscard]] std::size_t buffer_size(aero::compute::BufferId buffer) const noexcept override;
    [[nodiscard]] aero::compute::Status upload(aero::compute::BufferId buffer, std::size_t offset,
                                               std::span<const std::byte> bytes) override;
    [[nodiscard]] aero::compute::Status readback(aero::compute::BufferId buffer, std::size_t offset,
                                                 std::span<std::byte> bytes) override;
    [[nodiscard]] aero::compute::Status
    dispatch(const aero::compute::DispatchCommand& command) override;
    void barrier() override;

    // Kernel names, sorted.
    [[nodiscard]] std::vector<std::string> kernel_names() const;
    // Compiles and links every kernel now (otherwise each is built on its
    // first dispatch); the error names the first kernel that failed.
    [[nodiscard]] aero::compute::Status compile_all();
    // Blocks until the device has finished every submitted command (timing).
    void finish() const;

private:
    struct Impl;
    explicit GlComputeBackend(std::unique_ptr<Impl> impl) noexcept;
    std::unique_ptr<Impl> impl_;
};

} // namespace orrery::render
