#pragma once

#include <cstdint>

// Profiling zones (docs/observability.md).
//
//   ORRERY_ZONE("physics_step");   // measures the rest of the enclosing scope
//   ORRERY_FRAME_ZONE();           // the "frame" zone of a render loop body
//
// The macros compile to nothing unless the build defines ORRERY_PROFILING
// (CMake option ORRERY_PROFILING, off by default and in every CI preset).
// With it, every zone records into the built-in trace sink while a capture
// is active (trace_capture.h, orrery_app --trace), and is also a Tracy zone
// when the build defines ORRERY_PROFILING_TRACY (CMake option ORRERY_TRACY).
//
// Rules: the name is a string literal (it is stored by address); at most one
// zone per function (a zone in a nested scope of the same function would
// shadow the outer one, as with Tracy's ZoneScoped: nest by calling a
// function); zones go around whole steps and solves, never inside inner
// loops. A zone only reads the clock: it never touches the measured state.
namespace orrery::profile {

// Scoped zone of the built-in sink: records [construction, destruction) when
// a capture was active at construction. `name` must outlive the capture.
class Zone {
public:
    explicit Zone(const char* name) noexcept;
    ~Zone();
    Zone(const Zone&) = delete;
    Zone& operator=(const Zone&) = delete;
    Zone(Zone&&) = delete;
    Zone& operator=(Zone&&) = delete;

private:
    const char* name_ = nullptr; // null when no capture was active
    std::int64_t start_ns_ = 0;
};

} // namespace orrery::profile

#if defined(ORRERY_PROFILING)

#if defined(ORRERY_PROFILING_TRACY)
#include <tracy/Tracy.hpp>
#define ORRERY_PROFILE_TRACY_ZONE(name) ZoneScopedN(name)
#define ORRERY_PROFILE_TRACY_FRAME() FrameMark
#else
#define ORRERY_PROFILE_TRACY_ZONE(name) static_cast<void>(0)
#define ORRERY_PROFILE_TRACY_FRAME() static_cast<void>(0)
#endif

// The empty literals around `name` reject anything but a string literal.
#define ORRERY_ZONE(name)                                                                          \
    ORRERY_PROFILE_TRACY_ZONE(name);                                                               \
    const ::orrery::profile::Zone orrery_profile_zone_{"" name ""}
// Marks the frame boundary for Tracy and measures the loop body as "frame".
#define ORRERY_FRAME_ZONE()                                                                        \
    ORRERY_PROFILE_TRACY_FRAME();                                                                  \
    ORRERY_ZONE("frame")

#else

#define ORRERY_ZONE(name) static_cast<void>(0)
#define ORRERY_FRAME_ZONE() static_cast<void>(0)

#endif
