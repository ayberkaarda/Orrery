#pragma once

// Private to orrery_render: the only place that includes the OpenGL loader
// and GLFW. glad must come first; GLFW_INCLUDE_NONE stops GLFW from pulling
// in a system OpenGL header.
//
// The vcpkg glad port (0.1.36) generates its loader for the compatibility
// profile. That loader is a superset of the 4.6 core entry points; the
// context itself is created as a 4.6 core profile, so removed legacy entry
// points simply stay null and are never called.
#include <glad/glad.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
