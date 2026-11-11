add_library(orrery_render STATIC
    module.cpp
    include/orrery/render/module.h)
add_library(orrery::render ALIAS orrery_render)

target_include_directories(orrery_render PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(orrery_render PUBLIC orrery_core orrery_geom orrery_aero)

# Phase 4a: render foundation (camera, GL context and RAII objects, HDR
# pipeline, headless snapshots, PNG and SSIM). Third-party dependencies are
# approved for orrery_render only (tools/check_deps.py THIRD_PARTY_DEPS):
#   glm  - public, the camera API uses glm vector, quaternion and matrix types
#   glfw - private, window and OpenGL 4.6 core context creation
#   glad - private, OpenGL function loader
# No public header includes GLFW or glad.
find_package(glm CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)

target_sources(orrery_render PRIVATE
    camera.cpp
    gl_context.cpp
    gl_resources.cpp
    gl_loader.h
    hdr_pipeline.cpp
    image.cpp
    log.cpp
    png.cpp
    shader_compose.cpp
    ssim.cpp
    snapshot.cpp
    test_scene.cpp
    tone_map.cpp
    include/orrery/render/camera.h
    include/orrery/render/gl_context.h
    include/orrery/render/gl_resources.h
    include/orrery/render/hdr_pipeline.h
    include/orrery/render/image.h
    include/orrery/render/log.h
    include/orrery/render/log_depth.h
    include/orrery/render/png.h
    include/orrery/render/shader_compose.h
    include/orrery/render/snapshot.h
    include/orrery/render/ssim.h
    include/orrery/render/test_scene.h
    include/orrery/render/tone_map.h)
target_link_libraries(orrery_render
    PUBLIC glm::glm-header-only
    PRIVATE glfw glad::glad)

# Shaders: src/render/shaders/*.glsl are embedded at build time into one
# generated header (no runtime file reads). The header is private to the
# library sources and to the render tests, which check the embedding.
file(GLOB ORRERY_RENDER_SHADERS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.glsl)
set(ORRERY_RENDER_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(ORRERY_RENDER_SHADER_HEADER ${ORRERY_RENDER_GENERATED_DIR}/orrery/render/embedded_shaders.h)
add_custom_command(
    OUTPUT ${ORRERY_RENDER_SHADER_HEADER}
    COMMAND ${CMAKE_COMMAND}
        -DOUTPUT=${ORRERY_RENDER_SHADER_HEADER}
        "-DSHADERS=${ORRERY_RENDER_SHADERS}"
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_shaders.cmake
    DEPENDS ${ORRERY_RENDER_SHADERS} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_shaders.cmake
    COMMENT "Embedding render shaders"
    VERBATIM)
target_sources(orrery_render PRIVATE ${ORRERY_RENDER_SHADER_HEADER})
target_include_directories(orrery_render PRIVATE ${ORRERY_RENDER_GENERATED_DIR})

orrery_configure_target(orrery_render)
