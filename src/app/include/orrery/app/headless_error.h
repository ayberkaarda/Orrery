#pragma once

#include <cstdint>
#include <string>

// Errors of the headless renderers (headless.h for space experiments,
// aero_headless.h for aero experiments).
namespace orrery::app {

enum class HeadlessErrorKind : std::uint8_t {
    no_context, // no OpenGL 4.6 core context on this machine
    failure,    // anything else (simulation, renderer setup, drawing)
};

struct HeadlessError {
    HeadlessErrorKind kind = HeadlessErrorKind::failure;
    std::string message;
};

} // namespace orrery::app
