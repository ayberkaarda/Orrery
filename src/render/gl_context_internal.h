#pragma once

#include <cstdint>

// Private to orrery_render: identity of the OpenGL context that is current on
// the calling thread, for code that must act in a specific context (GL object
// deletion, renderer teardown).
namespace orrery::render::detail {

// Serial number of the GlContext current on this thread, 0 if none. Serials
// are unique for the lifetime of the process, so the serial of a destroyed
// context never matches a later one.
[[nodiscard]] std::uint64_t current_context_serial() noexcept;

// The current native (GLFW) context as an opaque pointer, nullptr if none,
// and making such a context current again (nullptr detaches).
[[nodiscard]] void* current_native_context() noexcept;
void make_native_context_current(void* context) noexcept;

} // namespace orrery::render::detail
