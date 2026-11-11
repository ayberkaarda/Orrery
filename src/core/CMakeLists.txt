add_library(orrery_core STATIC
    module.cpp
    include/orrery/core/module.h)
add_library(orrery::core ALIAS orrery_core)

target_include_directories(orrery_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_definitions(orrery_core PRIVATE
    ORRERY_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    ORRERY_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    ORRERY_VERSION_PATCH=${PROJECT_VERSION_PATCH})

orrery_configure_target(orrery_core)
