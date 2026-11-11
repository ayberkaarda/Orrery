#pragma once

#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "orrery/core/result.h"

// GLSL source composition and compiler-log annotation (no OpenGL calls, so it
// is unit-testable without a context).
//
// A stage is one main file that starts with a #version line, plus optional
// snippet files (for example log_depth.glsl) inserted right after that line.
// Every piece is passed as its own source string with a `#line 1 <n>`
// directive, where n is the piece index in `names` (0 = main file). Driver
// logs refer to "n:line" or "n(line)"; annotate_shader_log rewrites those
// references to "file.glsl:line".
namespace orrery::render {

struct ShaderSource {
    std::string_view name; // file name used in error messages
    std::string_view code;
};

struct ComposedShader {
    std::vector<std::string> strings; // pass to glShaderSource in this order
    std::vector<std::string> names;   // source-string number -> file name
};

[[nodiscard]] core::Result<ComposedShader, std::string>
compose_shader(const ShaderSource& main, std::span<const ShaderSource> includes);

[[nodiscard]] std::string annotate_shader_log(std::string_view log,
                                              std::span<const std::string> names);

} // namespace orrery::render
