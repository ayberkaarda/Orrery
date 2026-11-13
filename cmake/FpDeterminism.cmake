# Deterministic floating-point semantics.
#
# Physics results must be reproducible bit-for-bit across runs on the same
# platform/compiler: no FMA contraction, no fast-math reassociation.
#   MSVC:      /fp:precise
#   GCC/Clang: -fno-fast-math -ffp-contract=off
#
# -ffp-contract=off is passed last: on some compilers -fno-fast-math resets the
# contraction mode to its default, which may be "on" or "fast".
#
# tools/check_deps.py verifies the final compile and link commands of every
# configured tree (a later per-target or per-source option can still override
# these defaults; the checker rejects that).

include_guard(GLOBAL)

# Options that change floating-point results. Keep in sync with
# FAST_MATH_FLAG in tools/check_deps.py.
set(_orrery_fast_math_regex
    "(-ffast-math|-Ofast|[-/]fp:fast|[-/]fp:contract|[-/]Qfast_transcendentals"
    "|-funsafe-math-optimizations|-fassociative-math|-freciprocal-math"
    "|-ffp-contract=fast|-ffp-contract=on|-ffp-model=fast|-ffp-model=aggressive"
    "|-ffinite-math-only|-fno-signed-zeros|-fno-honor-nans|-fno-honor-infinities"
    "|-fapprox-func|-fcx-limited-range|-fcx-fortran-rules|-mdaz-ftz"
    "|-menable-unsafe-fp-math|-mrecip)")
string(JOIN "" _orrery_fast_math_regex ${_orrery_fast_math_regex})

# Refuse to configure if fast-math has been injected through global flags.
# Linker flags are included: with GCC, -ffast-math/-Ofast at link time links
# crtfastmath.o, which enables flush-to-zero for the whole process.
foreach(var
        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_RELWITHDEBINFO CMAKE_CXX_FLAGS_MINSIZEREL
        CMAKE_EXE_LINKER_FLAGS CMAKE_EXE_LINKER_FLAGS_DEBUG
        CMAKE_EXE_LINKER_FLAGS_RELEASE CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO
        CMAKE_EXE_LINKER_FLAGS_MINSIZEREL
        CMAKE_SHARED_LINKER_FLAGS CMAKE_MODULE_LINKER_FLAGS)
    if("${${var}}" MATCHES "${_orrery_fast_math_regex}")
        message(FATAL_ERROR
            "${var} contains a fast-math flag ('${CMAKE_MATCH_1}'); "
            "this breaks the floating-point determinism contract.")
    endif()
endforeach()

function(orrery_target_fp_determinism target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /fp:precise)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE -fno-fast-math -ffp-contract=off)
        # With link-time optimisation (Lto.cmake) the link step runs the
        # optimiser again; it gets the same floating-point options.
        target_link_options(${target} PRIVATE
            "$<$<BOOL:$<TARGET_PROPERTY:INTERPROCEDURAL_OPTIMIZATION>>:-fno-fast-math;-ffp-contract=off>")
    else()
        message(WARNING
            "Floating-point determinism flags are unknown for ${CMAKE_CXX_COMPILER_ID}")
    endif()
endfunction()
