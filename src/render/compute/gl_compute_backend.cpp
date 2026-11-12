#include "orrery/render/gl_compute_backend.h"

#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <utility>
#include <variant>

#include "../gl_context_internal.h"
#include "../gl_loader.h"
#include "orrery/render/embedded_lbm_shaders.h"
#include "orrery/render/gl_resources.h"

namespace orrery::render {

namespace {

using aero::compute::BufferId;
using aero::compute::DispatchCommand;
using aero::compute::Float32Uniform;
using aero::compute::Status;

constexpr std::string_view kKernelSuffix = ".comp.glsl";

std::string kernel_name_of(std::string_view file) {
    return std::string(file.substr(0, file.size() - kKernelSuffix.size()));
}

bool is_kernel_file(std::string_view file) {
    return file.size() > kKernelSuffix.size() && file.ends_with(kKernelSuffix);
}

ShaderSource source_of(const lbm_shaders::EmbeddedShader& shader) {
    return {shader.name, shader.source};
}

struct UniformSlot {
    GLint location = -1;
    GLenum type = 0;
};

struct Kernel {
    Program program;
    std::array<GLuint, 3> local_size{1, 1, 1};
    std::map<std::string, UniformSlot, std::less<>> uniforms;
};

std::map<std::string, UniformSlot, std::less<>> active_uniforms(GLuint program) {
    std::map<std::string, UniformSlot, std::less<>> slots;
    GLint count = 0;
    GLint longest = 0;
    glGetProgramInterfaceiv(program, GL_UNIFORM, GL_ACTIVE_RESOURCES, &count);
    glGetProgramInterfaceiv(program, GL_UNIFORM, GL_MAX_NAME_LENGTH, &longest);
    for (GLint r = 0; r < count; ++r) {
        std::string name(static_cast<std::size_t>(longest > 0 ? longest : 1), '\0');
        GLsizei length = 0;
        glGetProgramResourceName(program, GL_UNIFORM, static_cast<GLuint>(r), longest, &length,
                                 name.data());
        name.resize(static_cast<std::size_t>(length));
        const std::array<GLenum, 2> properties{GL_TYPE, GL_LOCATION};
        std::array<GLint, 2> values{0, -1};
        glGetProgramResourceiv(program, GL_UNIFORM, static_cast<GLuint>(r), 2, properties.data(), 2,
                               nullptr, values.data());
        if (values[1] >= 0) {
            slots.emplace(std::move(name), UniformSlot{values[1], static_cast<GLenum>(values[0])});
        }
    }
    return slots;
}

GLenum gl_type_of(const aero::compute::UniformValue& value) noexcept {
    if (std::holds_alternative<std::int32_t>(value)) {
        return GL_INT;
    }
    if (std::holds_alternative<std::uint32_t>(value)) {
        return GL_UNSIGNED_INT;
    }
    return GL_FLOAT;
}

const char* type_name(GLenum type) noexcept {
    switch (type) {
        case GL_INT:
            return "int";
        case GL_UNSIGNED_INT:
            return "uint";
        case GL_FLOAT:
            return "float";
        default:
            return "another type";
    }
}

Status failed(std::string message) {
    return core::failure("opengl compute backend: " + std::move(message));
}

} // namespace

struct GlComputeBackend::Impl {
    std::uint64_t context = 0;
    std::string device;
    std::map<std::string, const lbm_shaders::EmbeddedShader*, std::less<>> kernel_sources;
    std::vector<ShaderSource> snippets;
    std::map<std::string, Kernel, std::less<>> kernels;
    std::map<std::uint32_t, Buffer> buffers;
    std::uint32_t next_id = 1;
    std::array<GLint, 3> max_groups{0, 0, 0};
    GLint max_bindings = 0;

    [[nodiscard]] bool current() const noexcept {
        return context != 0 && detail::current_context_serial() == context;
    }

    core::Result<Kernel*, std::string> kernel(std::string_view name) {
        if (auto built = kernels.find(name); built != kernels.end()) {
            return &built->second;
        }
        const auto source = kernel_sources.find(name);
        if (source == kernel_sources.end()) {
            return core::failure("opengl compute backend: unknown kernel '" + std::string(name) +
                                 "'");
        }
        const std::array<StageSource, 1> stages{
            StageSource{ShaderStage::compute, source_of(*source->second), snippets}};
        auto program = Program::create(stages);
        if (!program) {
            return core::failure("opengl compute backend: kernel '" + std::string(name) +
                                 "': " + program.error());
        }
        Kernel built{std::move(program).value(), {1, 1, 1}, {}};
        std::array<GLint, 3> local{1, 1, 1};
        glGetProgramiv(built.program.id(), GL_COMPUTE_WORK_GROUP_SIZE, local.data());
        for (std::size_t d = 0; d < 3; ++d) {
            built.local_size[d] = static_cast<GLuint>(local[d] > 0 ? local[d] : 1);
        }
        built.uniforms = active_uniforms(built.program.id());
        auto placed = kernels.emplace(std::string(name), std::move(built));
        return &placed.first->second;
    }
};

GlComputeBackend::GlComputeBackend(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}

GlComputeBackend::~GlComputeBackend() = default;

core::Result<std::unique_ptr<GlComputeBackend>, std::string> GlComputeBackend::create() {
    const std::uint64_t context = detail::current_context_serial();
    if (context == 0) {
        return core::failure(
            std::string("opengl compute backend: no OpenGL context is current on this thread"));
    }
    auto impl = std::make_unique<Impl>();
    impl->context = context;
    // GL boundary: glGetString returns the renderer name as GLubyte text.
    const auto* renderer = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
    impl->device = renderer != nullptr ? renderer : "unknown OpenGL device";
    for (const lbm_shaders::EmbeddedShader* shader : lbm_shaders::all) {
        if (is_kernel_file(shader->name)) {
            impl->kernel_sources.emplace(kernel_name_of(shader->name), shader);
        } else {
            impl->snippets.push_back(source_of(*shader));
        }
    }
    for (GLuint d = 0; d < 3; ++d) {
        glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, d,
                        &impl->max_groups[static_cast<std::size_t>(d)]);
    }
    glGetIntegerv(GL_MAX_SHADER_STORAGE_BUFFER_BINDINGS, &impl->max_bindings);
    return std::unique_ptr<GlComputeBackend>(new GlComputeBackend(std::move(impl)));
}

aero::compute::ComputeBackendInfo GlComputeBackend::info() const {
    return aero::compute::ComputeBackendInfo{"opengl", impl_->device, true};
}

bool GlComputeBackend::has_kernel(std::string_view name) const {
    return impl_->kernel_sources.find(name) != impl_->kernel_sources.end();
}

std::vector<std::string> GlComputeBackend::kernel_names() const {
    std::vector<std::string> names;
    names.reserve(impl_->kernel_sources.size());
    for (const auto& entry : impl_->kernel_sources) {
        names.push_back(entry.first);
    }
    return names;
}

Status GlComputeBackend::compile_all() {
    if (!impl_->current()) {
        return failed("its OpenGL context is not current");
    }
    for (const auto& entry : impl_->kernel_sources) {
        if (auto built = impl_->kernel(entry.first); !built) {
            return core::failure(std::string(built.error()));
        }
    }
    return std::monostate{};
}

core::Result<BufferId, std::string> GlComputeBackend::create_buffer(std::size_t bytes) {
    if (!impl_->current()) {
        return core::failure(
            std::string("opengl compute backend: its OpenGL context is not current"));
    }
    if (bytes == 0) {
        return core::failure(std::string("opengl compute backend: empty buffer"));
    }
    if (impl_->next_id == std::numeric_limits<std::uint32_t>::max()) {
        return core::failure(std::string("opengl compute backend: buffer ids exhausted"));
    }
    auto buffer = Buffer::create_uninitialized(bytes, true);
    if (!buffer) {
        return core::failure("opengl compute backend: " + buffer.error());
    }
    // Zero fill (immutable storage starts undefined).
    glClearNamedBufferData(buffer.value().id(), GL_R8UI, GL_RED_INTEGER, GL_UNSIGNED_BYTE, nullptr);
    const BufferId id{impl_->next_id++};
    impl_->buffers.emplace(id.value, std::move(buffer).value());
    return id;
}

void GlComputeBackend::release_buffer(BufferId buffer) noexcept {
    if (impl_->current()) {
        impl_->buffers.erase(buffer.value);
    }
}

std::size_t GlComputeBackend::buffer_size(BufferId buffer) const noexcept {
    const auto found = impl_->buffers.find(buffer.value);
    return found == impl_->buffers.end() ? 0 : found->second.size();
}

Status GlComputeBackend::upload(BufferId buffer, std::size_t offset,
                                std::span<const std::byte> bytes) {
    if (!impl_->current()) {
        return failed("its OpenGL context is not current");
    }
    const auto found = impl_->buffers.find(buffer.value);
    if (found == impl_->buffers.end()) {
        return failed("upload to an unknown buffer");
    }
    const std::size_t size = found->second.size();
    if (offset > size || bytes.size() > size - offset) {
        return failed("upload outside the buffer");
    }
    if (!bytes.empty()) {
        // Earlier shader writes to the buffer complete before the update.
        glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
        found->second.update(offset, bytes);
    }
    return std::monostate{};
}

Status GlComputeBackend::readback(BufferId buffer, std::size_t offset, std::span<std::byte> bytes) {
    if (!impl_->current()) {
        return failed("its OpenGL context is not current");
    }
    const auto found = impl_->buffers.find(buffer.value);
    if (found == impl_->buffers.end()) {
        return failed("readback of an unknown buffer");
    }
    const std::size_t size = found->second.size();
    if (offset > size || bytes.size() > size - offset) {
        return failed("readback outside the buffer");
    }
    if (!bytes.empty()) {
        glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
        glGetNamedBufferSubData(found->second.id(), static_cast<GLintptr>(offset),
                                static_cast<GLsizeiptr>(bytes.size()), bytes.data());
    }
    return std::monostate{};
}

Status GlComputeBackend::dispatch(const DispatchCommand& command) {
    if (!impl_->current()) {
        return failed("its OpenGL context is not current");
    }
    if (command.global.x == 0 || command.global.y == 0 || command.global.z == 0) {
        return failed("empty dispatch of '" + std::string(command.kernel) + "'");
    }
    auto found = impl_->kernel(command.kernel);
    if (!found) {
        return core::failure(std::string(found.error()));
    }
    const Kernel& kernel = *found.value();
    const GLuint program = kernel.program.id();

    for (const aero::compute::Uniform& uniform : command.uniforms) {
        const auto slot = kernel.uniforms.find(uniform.name);
        if (slot == kernel.uniforms.end()) {
            continue; // not declared or optimised out
        }
        const GLenum wanted = gl_type_of(uniform.value);
        if (slot->second.type != wanted) {
            return failed("uniform '" + std::string(uniform.name) + "' of '" +
                          std::string(command.kernel) + "' is declared " +
                          type_name(slot->second.type) + ", given " + type_name(wanted));
        }
        const GLint location = slot->second.location;
        if (const auto* i = std::get_if<std::int32_t>(&uniform.value)) {
            glProgramUniform1i(program, location, *i);
        } else if (const auto* u = std::get_if<std::uint32_t>(&uniform.value)) {
            glProgramUniform1ui(program, location, *u);
        } else {
            // The binary32 rounding of the GPU boundary.
            const auto value = static_cast<float>(std::get<Float32Uniform>(uniform.value).value);
            glProgramUniform1f(program, location, value);
        }
    }

    std::set<std::uint32_t> bound;
    for (const aero::compute::BufferBinding& binding : command.buffers) {
        const auto buffer = impl_->buffers.find(binding.buffer.value);
        if (buffer == impl_->buffers.end()) {
            return failed("'" + std::string(command.kernel) + "' binds an unknown buffer at " +
                          std::to_string(binding.binding));
        }
        if (binding.binding >= static_cast<std::uint32_t>(impl_->max_bindings) ||
            !bound.insert(binding.binding).second) {
            return failed("'" + std::string(command.kernel) + "' binding point " +
                          std::to_string(binding.binding) + " is out of range or used twice");
        }
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding.binding, buffer->second.id());
    }

    const std::array<std::uint32_t, 3> items{command.global.x, command.global.y, command.global.z};
    std::array<GLuint, 3> groups{};
    for (std::size_t d = 0; d < 3; ++d) {
        const std::uint64_t count =
            (std::uint64_t{items[d]} + kernel.local_size[d] - 1) / kernel.local_size[d];
        if (count > static_cast<std::uint64_t>(impl_->max_groups[d])) {
            return failed("'" + std::string(command.kernel) + "' needs " + std::to_string(count) +
                          " work groups in dimension " + std::to_string(d) + ", the limit is " +
                          std::to_string(impl_->max_groups[d]));
        }
        groups[d] = static_cast<GLuint>(count);
    }
    glUseProgram(program);
    glDispatchCompute(groups[0], groups[1], groups[2]);
    return std::monostate{};
}

void GlComputeBackend::barrier() {
    if (impl_->current()) {
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    }
}

void GlComputeBackend::finish() const {
    if (impl_->current()) {
        glFinish();
    }
}

} // namespace orrery::render
