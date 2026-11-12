#include "orrery/aero/compute/null_compute_backend.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <utility>

namespace orrery::aero::compute {

namespace {

// Largest buffer the host backend allocates (4 GiB).
constexpr std::size_t kMaxBufferBytes = std::size_t{1} << 32U;

bool range_ok(std::size_t size, std::size_t offset, std::size_t length) noexcept {
    return offset <= size && length <= size - offset;
}

} // namespace

std::span<std::byte> NullComputeBackend::Bindings::at(std::uint32_t binding) const noexcept {
    const auto found = views_.find(binding);
    return found == views_.end() ? std::span<std::byte>{} : found->second;
}

void NullComputeBackend::register_kernel(std::string name, HostKernel kernel) {
    kernels_.insert_or_assign(std::move(name), std::move(kernel));
}

ComputeBackendInfo NullComputeBackend::info() const {
    return ComputeBackendInfo{"null", "host memory", !kernels_.empty()};
}

bool NullComputeBackend::has_kernel(std::string_view name) const {
    return kernels_.find(name) != kernels_.end();
}

core::Result<BufferId, std::string> NullComputeBackend::create_buffer(std::size_t bytes) {
    if (bytes == 0 || bytes > kMaxBufferBytes) {
        return core::failure(std::string("null compute backend: buffer size ") +
                             std::to_string(bytes) + " outside [1, 4 GiB]");
    }
    if (next_id_ == std::numeric_limits<std::uint32_t>::max()) {
        return core::failure(std::string("null compute backend: buffer ids exhausted"));
    }
    const BufferId id{next_id_++};
    buffers_.emplace(id.value, std::vector<std::byte>(bytes, std::byte{0}));
    return id;
}

void NullComputeBackend::release_buffer(BufferId buffer) noexcept {
    buffers_.erase(buffer.value);
}

std::size_t NullComputeBackend::buffer_size(BufferId buffer) const noexcept {
    const auto found = buffers_.find(buffer.value);
    return found == buffers_.end() ? 0 : found->second.size();
}

Status NullComputeBackend::upload(BufferId buffer, std::size_t offset,
                                  std::span<const std::byte> bytes) {
    const auto found = buffers_.find(buffer.value);
    if (found == buffers_.end()) {
        return core::failure(std::string("null compute backend: upload to an unknown buffer"));
    }
    if (!range_ok(found->second.size(), offset, bytes.size())) {
        return core::failure(std::string("null compute backend: upload outside the buffer"));
    }
    if (!bytes.empty()) {
        std::memcpy(found->second.data() + offset, bytes.data(), bytes.size());
    }
    return std::monostate{};
}

Status NullComputeBackend::readback(BufferId buffer, std::size_t offset,
                                    std::span<std::byte> bytes) {
    const auto found = buffers_.find(buffer.value);
    if (found == buffers_.end()) {
        return core::failure(std::string("null compute backend: readback of an unknown buffer"));
    }
    if (!range_ok(found->second.size(), offset, bytes.size())) {
        return core::failure(std::string("null compute backend: readback outside the buffer"));
    }
    if (!bytes.empty()) {
        std::memcpy(bytes.data(), found->second.data() + offset, bytes.size());
    }
    return std::monostate{};
}

Status NullComputeBackend::dispatch(const DispatchCommand& command) {
    if (command.global.x == 0 || command.global.y == 0 || command.global.z == 0) {
        return core::failure(std::string("null compute backend: empty dispatch of '") +
                             std::string(command.kernel) + "'");
    }
    const auto kernel = kernels_.find(command.kernel);
    if (kernel == kernels_.end() && !accept_unknown_) {
        return core::failure(std::string("null compute backend: unknown kernel '") +
                             std::string(command.kernel) + "'");
    }
    std::map<std::uint32_t, std::span<std::byte>> views;
    for (const BufferBinding& binding : command.buffers) {
        const auto found = buffers_.find(binding.buffer.value);
        if (found == buffers_.end()) {
            return core::failure(std::string("null compute backend: '") +
                                 std::string(command.kernel) + "' binds an unknown buffer at " +
                                 std::to_string(binding.binding));
        }
        if (!views.emplace(binding.binding, std::span<std::byte>(found->second)).second) {
            return core::failure(std::string("null compute backend: '") +
                                 std::string(command.kernel) + "' binds two buffers at " +
                                 std::to_string(binding.binding));
        }
    }

    RecordedDispatch record;
    record.kernel = std::string(command.kernel);
    record.global = command.global;
    record.uniforms.reserve(command.uniforms.size());
    for (const Uniform& uniform : command.uniforms) {
        record.uniforms.push_back(RecordedUniform{std::string(uniform.name), uniform.value});
    }
    record.buffers.assign(command.buffers.begin(), command.buffers.end());
    record.barriers_before = barriers_;
    dispatches_.push_back(std::move(record));

    if (kernel != kernels_.end()) {
        return kernel->second(command, Bindings(std::move(views)));
    }
    return std::monostate{};
}

} // namespace orrery::aero::compute
