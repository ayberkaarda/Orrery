add_executable(orrery_app
    main.cpp
    build_info.h)

target_include_directories(orrery_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(orrery_app PRIVATE orrery_scene orrery_render)

if(ORRERY_HEADLESS)
    target_compile_definitions(orrery_app PRIVATE ORRERY_HEADLESS=1)
endif()

orrery_configure_target(orrery_app)
