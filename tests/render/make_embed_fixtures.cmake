# Writes the shader embedding fixtures checked by test_shader_embedding.cpp:
# shaders whose lines exceed the raw string piece limit (8000 bytes) of
# src/render/cmake/embed_shaders.cmake.
#
# Usage: cmake -DOUTPUT_DIR=<dir> -P make_embed_fixtures.cmake
#
#   long_line.frag.glsl   #version, one 20,000-byte comment line, a small main;
#                         the line repeats characters that are special to CMake
#                         or C++ (; " \ $ { } [ ])
#   utf8_line.frag.glsl   one 30,003-byte comment of 3-byte UTF-8 characters,
#                         so a byte-count split would land inside a character
#   many_lines.frag.glsl  1500 short lines (split at line boundaries)

cmake_minimum_required(VERSION 3.28)

if(NOT DEFINED OUTPUT_DIR)
    message(FATAL_ERROR "make_embed_fixtures.cmake needs -DOUTPUT_DIR=<dir>")
endif()
file(MAKE_DIRECTORY "${OUTPUT_DIR}")

string(REPEAT "ab;cd\"ef\\gh\${ij}[k] " 999 pattern)
string(REPEAT "x" 18 tail)
set(long_line "//${pattern}${tail}")
string(LENGTH "${long_line}" long_line_length)
if(NOT long_line_length EQUAL 20000)
    message(FATAL_ERROR "long_line fixture is ${long_line_length} bytes, expected 20000")
endif()
file(WRITE "${OUTPUT_DIR}/long_line.frag.glsl"
    "#version 460 core\n${long_line}\nout vec4 o_color;\nvoid main() { o_color = vec4(1.0); }\n")

# U+20AC EURO SIGN, encoded as the three bytes E2 82 AC.
string(REPEAT "€" 10000 euros)
file(WRITE "${OUTPUT_DIR}/utf8_line.frag.glsl"
    "#version 460 core\n// ${euros}\nvoid main() {}\n")

set(many_lines "#version 460 core\n")
foreach(index RANGE 1 1500)
    string(APPEND many_lines "// short line ${index}: x = y; z = \"w\";\n")
endforeach()
string(APPEND many_lines "void main() {}\n")
file(WRITE "${OUTPUT_DIR}/many_lines.frag.glsl" "${many_lines}")
