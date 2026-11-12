# Target graph (direct dependencies):
#   core   -> (none)
#   geom   -> core
#   atmo   -> core
#   aero   -> core, geom
#   scene  -> core, geom, aero, atmo
#   render -> core, geom, aero
#   sim      -> core, scene          (physics thread, commands; src/app/sim)
#   app_core -> sim, render          (command line, headless renderer)
#   app      -> app_core             (the orrery_app executable)
#
# core, geom, atmo and sim must never link against OpenGL, GLFW or ImGui;
# tools/check_deps.py enforces this.

add_subdirectory(core)
add_subdirectory(geom)
add_subdirectory(atmo)
add_subdirectory(aero)
add_subdirectory(scene)
add_subdirectory(render)
add_subdirectory(app)
