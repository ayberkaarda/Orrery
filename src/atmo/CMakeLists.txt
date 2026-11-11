add_library(orrery_atmo STATIC
    module.cpp
    include/orrery/atmo/module.h)
add_library(orrery::atmo ALIAS orrery_atmo)

target_include_directories(orrery_atmo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(orrery_atmo PUBLIC orrery_core)

orrery_configure_target(orrery_atmo)
