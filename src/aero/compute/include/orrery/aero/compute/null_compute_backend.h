#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "orrery/aero/compute/compute_backend.h"

namespace orrery::aero::compute {

// Host-memory backend for tests and machines without a GPU. Buffers are real
// (upload and readback round-trip), every dispatch is validated against the
// buffers and recorded, and a dispatch runs a host function only when one was
// registered for its kernel name; other kernels are accepted as no-ops when
// `accept_unknown_kernels` is set and rejected otherwise.
class NullComputeBackend final : public IComputeBackend {
public:
    // A recorded dispatch, with owned copies of the names.
    struct RecordedUniform {
        std::string name;
        UniformValue value;
    };
    struct RecordedDispatch {
        std::string kernel;
        DispatchSize global;
        std::vector<RecordedUniform> uniforms;
        std::vector<BufferBinding> buffers;
        std::size_t barriers_before = 0; // barrier() calls before this dispatch
    };

    // Host view of the buffers bound to a dispatch, by binding point.
    class Bindings {
    public:
        explicit Bindings(std::map<std::uint32_t, std::span<std::byte>> views) noexcept
            : views_(std::move(views)) {}
        // Empty span when nothing is bound at `binding`.
        [[nodiscard]] std::span<std::byte> at(std::uint32_t binding) const noexcept;

    private:
        std::map<std::uint32_t, std::span<std::byte>> views_;
    };
    using HostKernel = std::function<Status(const DispatchCommand&, const Bindings&)>;

    explicit NullComputeBackend(bool accept_unknown_kernels = true) noexcept
        : accept_unknown_(accept_unknown_kernels) {}

    void register_kernel(std::string name, HostKernel kernel);

    [[nodiscard]] ComputeBackendInfo info() const override;
    [[nodiscard]] bool has_kernel(std::string_view name) const override;
    [[nodiscard]] core::Result<BufferId, std::string> create_buffer(std::size_t bytes) override;
    void release_buffer(BufferId buffer) noexcept override;
    [[nodiscard]] std::size_t buffer_size(BufferId buffer) const noexcept override;
    [[nodiscard]] Status upload(BufferId buffer, std::size_t offset,
                                std::span<const std::byte> bytes) override;
    [[nodiscard]] Status readback(BufferId buffer, std::size_t offset,
                                  std::span<std::byte> bytes) override;
    [[nodiscard]] Status dispatch(const DispatchCommand& command) override;
    void barrier() override { ++barriers_; }

    [[nodiscard]] const std::vector<RecordedDispatch>& dispatches() const noexcept {
        return dispatches_;
    }
    [[nodiscard]] std::size_t barrier_count() const noexcept { return barriers_; }
    [[nodiscard]] std::size_t live_buffers() const noexcept { return buffers_.size(); }
    void clear_log() noexcept { dispatches_.clear(); }

private:
    bool accept_unknown_ = true;
    std::uint32_t next_id_ = 1;
    std::size_t barriers_ = 0;
    std::map<std::uint32_t, std::vector<std::byte>> buffers_;
    std::map<std::string, HostKernel, std::less<>> kernels_;
    std::vector<RecordedDispatch> dispatches_;
};

} // namespace orrery::aero::compute
