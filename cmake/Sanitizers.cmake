# Sanitizer support.
#   MSVC:      /fsanitize=address
#   GCC/Clang: -fsanitize=address,undefined

include_guard(GLOBAL)

if(ORRERY_SANITIZE AND MSVC)
    # /RTC run-time checks and incremental linking are incompatible with ASan.
    foreach(var
            CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELWITHDEBINFO)
        string(REGEX REPLACE "/RTC[1csu]+" "" ${var} "${${var}}")
        set(${var} "${${var}}" CACHE STRING "" FORCE)
    endforeach()
    foreach(var
            CMAKE_EXE_LINKER_FLAGS_DEBUG CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO
            CMAKE_SHARED_LINKER_FLAGS_DEBUG CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO)
        # Whole tokens only; leave /INCREMENTAL:NO unchanged on later configures.
        # Repeat because adjacent options can share a matched whitespace boundary.
        while("${${var}}" MATCHES "(^|[ \t])/INCREMENTAL(:YES)?([ \t]|$)")
            string(REGEX REPLACE "(^|[ \t])/INCREMENTAL(:YES)?([ \t]|$)"
                "\\1/INCREMENTAL:NO\\3" ${var} "${${var}}")
        endwhile()
        set(${var} "${${var}}" CACHE STRING "" FORCE)
    endforeach()
endif()

function(orrery_target_sanitizers target)
    if(NOT ORRERY_SANITIZE)
        return()
    endif()

    if(MSVC)
        target_compile_options(${target} PRIVATE /fsanitize=address)
        target_link_options(${target} PRIVATE /DEBUG /INCREMENTAL:NO)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        set(flags -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
        target_compile_options(${target} PRIVATE ${flags})
        target_link_options(${target} PRIVATE ${flags})
    else()
        message(WARNING "ORRERY_SANITIZE is not supported for ${CMAKE_CXX_COMPILER_ID}")
    endif()
endfunction()
