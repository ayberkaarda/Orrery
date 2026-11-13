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

# Phase 4b: space scene visuals (icosphere bodies, starfield, trails) and the
# Dear ImGui / ImPlot user-interface layer. Both UI libraries are private
# (tools/check_deps.py THIRD_PARTY_DEPS): no public header includes ImGui.
#   imgui  - docking branch with the GLFW and OpenGL 3 backends
#   implot - plots in the System Monitor panel
find_package(imgui CONFIG REQUIRED)
find_package(implot CONFIG REQUIRED)
target_sources(orrery_render PRIVATE
    body_visuals.cpp
    icosphere.cpp
    render_settings.cpp
    space_renderer.cpp
    starfield.cpp
    trails.cpp
    ui_context.cpp
    ui_panels.cpp
    ui_theme.cpp
    include/orrery/render/body_visuals.h
    include/orrery/render/icosphere.h
    include/orrery/render/render_settings.h
    include/orrery/render/space_renderer.h
    include/orrery/render/starfield.h
    include/orrery/render/trails.h
    include/orrery/render/ui_context.h
    include/orrery/render/ui_panels.h
    include/orrery/render/ui_theme.h)
target_link_libraries(orrery_render PRIVATE imgui::imgui implot::implot)

# Phase 4c: application bar (workspace tab, transport, time warp, scenario
# selector) and scene input of the interactive window.
target_sources(orrery_render PRIVATE
    ui_app_bar.cpp
    include/orrery/render/ui_app_bar.h)

# Phase 5: black-hole lensing pass (Schwarzschild ray marching in a compute
# shader, environment cube map, thin accretion disk with black-body colour).
# The double-precision reference of the geodesics lives in orrery_core.
target_sources(orrery_render PRIVATE
    black_hole.cpp
    black_hole_pass.cpp
    black_hole_pass.h
    blackbody_spectrum.cpp
    include/orrery/render/black_hole.h
    include/orrery/render/blackbody_spectrum.h)

# Phase 8a: aero visualisation (docs/aero-visualisation.md): Cp surface,
# section planes, streamlines and particles, Q-criterion iso-surface and force
# vectors, drawn from plain data (orrery_aero results, orrery_geom meshes).
target_sources(orrery_render PRIVATE
    aero/aero_colormap.cpp
    aero/aero_compute.cpp
    aero/aero_field.cpp
    aero/aero_internal.h
    aero/aero_isosurface.cpp
    aero/aero_streamlines.cpp
    aero/aero_surface.cpp
    include/orrery/render/aero_colormap.h
    include/orrery/render/aero_compute.h
    include/orrery/render/aero_field.h
    include/orrery/render/aero_isosurface.h
    include/orrery/render/aero_streamlines.h
    include/orrery/render/aero_surface.h
    include/orrery/render/aero_view.h)

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

# Phase 7b: OpenGL 4.6 implementation of orrery_aero's IComputeBackend. Its
# kernels (shaders/lbm/*.glsl, GLSL 460 compute) are embedded into a header
# of their own and run by name; snippets without ".comp" are composed into
# every kernel.
file(GLOB ORRERY_RENDER_LBM_SHADERS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/shaders/lbm/*.glsl)
set(ORRERY_RENDER_LBM_SHADER_HEADER ${ORRERY_RENDER_GENERATED_DIR}/orrery/render/embedded_lbm_shaders.h)
add_custom_command(
    OUTPUT ${ORRERY_RENDER_LBM_SHADER_HEADER}
    COMMAND ${CMAKE_COMMAND}
        -DOUTPUT=${ORRERY_RENDER_LBM_SHADER_HEADER}
        -DNAMESPACE=orrery::render::lbm_shaders
        "-DSHADERS=${ORRERY_RENDER_LBM_SHADERS}"
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_shaders.cmake
    DEPENDS ${ORRERY_RENDER_LBM_SHADERS} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_shaders.cmake
    COMMENT "Embedding LBM compute shaders"
    VERBATIM)
target_sources(orrery_render PRIVATE
    compute/gl_compute_backend.cpp
    compute/include/orrery/render/gl_compute_backend.h
    ${ORRERY_RENDER_LBM_SHADER_HEADER})
target_include_directories(orrery_render PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/compute/include)

# Phase 8a: the aero visualisation shaders (shaders/aero/*.glsl, drawing) and
# compute kernels (shaders/aero/compute/aero_vis_*.glsl, run through
# GlComputeBackend) are embedded into one header of their own.
file(GLOB_RECURSE ORRERY_RENDER_AERO_SHADERS CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/shaders/aero/*.glsl)
set(ORRERY_RENDER_AERO_SHADER_HEADER ${ORRERY_RENDER_GENERATED_DIR}/orrery/render/embedded_aero_shaders.h)
add_custom_command(
    OUTPUT ${ORRERY_RENDER_AERO_SHADER_HEADER}
    COMMAND ${CMAKE_COMMAND}
        -DOUTPUT=${ORRERY_RENDER_AERO_SHADER_HEADER}
        -DNAMESPACE=orrery::render::aero_shaders
        "-DSHADERS=${ORRERY_RENDER_AERO_SHADERS}"
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_shaders.cmake
    DEPENDS ${ORRERY_RENDER_AERO_SHADERS} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_shaders.cmake
    COMMENT "Embedding aero visualisation shaders"
    VERBATIM)
target_sources(orrery_render PRIVATE ${ORRERY_RENDER_AERO_SHADER_HEADER})

orrery_configure_target(orrery_render)
