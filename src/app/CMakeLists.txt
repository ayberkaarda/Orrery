# Phase 4c: the application loop (docs/app.md).
#
#   orrery_sim       graphics-free simulation side: fixed-timestep Simulation,
#                    physics thread, triple buffer, SimCommand queue, space
#                    experiments. oneTBB (approved, private) backs the command
#                    queue (tools/check_deps.py THIRD_PARTY_DEPS).
#   orrery_app_core  command line, view policy and the headless renderer.
#   orrery_app       the executable; the interactive window is compiled only
#                    for builds with a window system (not ORRERY_HEADLESS).
# All three share the public include root include/ (orrery/sim, orrery/app).
find_package(TBB CONFIG REQUIRED)

add_library(orrery_sim STATIC
    sim/command_queue.cpp
    sim/physics_thread.cpp
    sim/simulation.cpp
    sim/space_experiment.cpp
    include/orrery/sim/command_queue.h
    include/orrery/sim/physics_thread.h
    include/orrery/sim/sim_command.h
    include/orrery/sim/simulation.h
    include/orrery/sim/space_experiment.h
    include/orrery/sim/triple_buffer.h)
add_library(orrery::sim ALIAS orrery_sim)
target_include_directories(orrery_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(orrery_sim PUBLIC orrery_core orrery_scene)
target_link_libraries(orrery_sim PRIVATE TBB::tbb)
orrery_configure_target(orrery_sim)

# Phase 8b: the graphics-free side of the Aero workspace (docs/aero-workspace.md):
# geometry preparation, flow summary and regime choice, solver thread,
# background imports (URL downloads through orrery_geom_net), experiment
# files and the comparison table. Still free of OpenGL, GLFW and ImGui.
target_sources(orrery_sim PRIVATE
    aero/aero_comparison.cpp
    aero/aero_flow.cpp
    aero/aero_geometry.cpp
    aero/aero_import.cpp
    aero/aero_runner.cpp
    aero/aero_solve.cpp
    aero/include/orrery/sim/aero_comparison.h
    aero/include/orrery/sim/aero_flow.h
    aero/include/orrery/sim/aero_geometry.h
    aero/include/orrery/sim/aero_import.h
    aero/include/orrery/sim/aero_runner.h
    aero/include/orrery/sim/aero_solve.h
    aero/include/orrery/sim/background_task.h)
target_include_directories(orrery_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/aero/include)
target_link_libraries(orrery_sim PUBLIC orrery_aero orrery_geom)
target_link_libraries(orrery_sim PRIVATE orrery_geom_net)

add_library(orrery_app_core STATIC
    cli.cpp
    frame_stats.cpp
    headless.cpp
    space_view.cpp
    include/orrery/app/cli.h
    include/orrery/app/frame_stats.h
    include/orrery/app/headless.h
    include/orrery/app/space_view.h)
add_library(orrery::app_core ALIAS orrery_app_core)
target_include_directories(orrery_app_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(orrery_app_core PUBLIC orrery_sim orrery_render)
# Bundled experiments and scenarios are found next to the executable
# (assets/) or, for builds run from the tree, in the source assets directory.
target_compile_definitions(orrery_app_core PRIVATE
    ORRERY_SOURCE_ASSET_DIR="${PROJECT_SOURCE_DIR}/assets")
orrery_configure_target(orrery_app_core)

add_executable(orrery_app
    main.cpp
    build_info.h)
if(NOT ORRERY_HEADLESS)
    target_sources(orrery_app PRIVATE
        window_app.cpp
        window_app.h)
endif()

target_include_directories(orrery_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(orrery_app PRIVATE orrery_app_core)

if(ORRERY_HEADLESS)
    target_compile_definitions(orrery_app PRIVATE ORRERY_HEADLESS=1)
endif()

orrery_configure_target(orrery_app)
