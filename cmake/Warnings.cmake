# Project warning policy. Applied per target so third-party code is unaffected.

include_guard(GLOBAL)

function(orrery_target_warnings target)
    if(MSVC)
        set(warnings
            /W4
            /permissive-
            /w14242 # conversion, possible loss of data
            /w14254 # larger bit field assigned to smaller bit field
            /w14263 # member function does not override base virtual
            /w14265 # class has virtual functions but non-virtual destructor
            /w14287 # unsigned/negative constant mismatch
            /w14296 # expression is always true/false
            /w14311 # pointer truncation
            /w14545 # expression before comma evaluates to a function
            /w14546 # function call before comma missing argument list
            /w14547 # operator before comma has no effect
            /w14549 # operator before comma has no effect
            /w14555 # expression has no effect
            /w14619 # unknown warning number in pragma
            /w14640 # thread-unsafe static member initialization
            /w14826 # conversion is sign-extended
            /w14905 # wide string literal cast to LPSTR
            /w14906 # string literal cast to LPWSTR
            /w14928 # illegal copy-initialization
        )
        if(ORRERY_WARNINGS_AS_ERRORS)
            list(APPEND warnings /WX)
        endif()
    else()
        set(warnings
            -Wall
            -Wextra
            -Wpedantic
            -Wshadow
            -Wnon-virtual-dtor
            -Wold-style-cast
            -Wcast-align
            -Wunused
            -Woverloaded-virtual
            -Wconversion
            -Wsign-conversion
            -Wdouble-promotion
            -Wnull-dereference
            -Wformat=2
            -Wimplicit-fallthrough
        )
        if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            list(APPEND warnings
                -Wmisleading-indentation
                -Wduplicated-cond
                -Wduplicated-branches
                -Wlogical-op
                -Wuseless-cast
            )
        endif()
        if(ORRERY_WARNINGS_AS_ERRORS)
            list(APPEND warnings -Werror)
        endif()
    endif()

    target_compile_options(${target} PRIVATE ${warnings})
endfunction()
