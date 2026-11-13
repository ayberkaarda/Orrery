# Target graph (direct dependencies):
#   core   -> (none)
#   geom   -> core
#   geom_net -> core                 (URL model loader; src/geom/net)
#   atmo   -> core, aero             (Cd(Mach) table from the aero solvers)
#   aero   -> core, geom
#   scene  -> core, geom, aero, atmo
#   render -> core, geom, aero
#   sim      -> core, scene, aero, geom, geom_net
#                                    (physics thread, commands, aero solver
#                                    thread and imports; src/app/sim, src/app/aero)
#   app_core -> sim, render          (command line, headless renderer)
#   app      -> app_core             (the orrery_app executable)
#
# core, geom, geom_net, atmo and sim must never link against OpenGL, GLFW or ImGui;
# tools/check_deps.py enforces this.

add_subdirectory(core)
add_subdirectory(geom)
add_subdirectory(geom/net)
add_subdirectory(atmo)
add_subdirectory(aero)
add_subdirectory(scene)
add_subdirectory(render)
add_subdirectory(app)
