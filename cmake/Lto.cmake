# Link-time optimisation of the release builds (spec phase 11, "release
# profile (LTO)"; docs/performance.md).
#
# ORRERY_LTO turns on CMake's interprocedural optimisation for every target
# created after this file is included (MSVC /GL + /LTCG, GCC -flto, Clang
# -flto=thin). The release presets set it; the CI and sanitizer presets do
# not. A toolchain without support configures without LTO and says so.
#
# Floating-point determinism (FpDeterminism.cmake) survives LTO: MSVC keeps
# /fp:precise per function in the /GL intermediate code, Clang encodes the
# contraction mode in the IR, and GCC streams -ffp-contract with each
# function. The GCC/Clang link step additionally repeats -fno-fast-math
# -ffp-contract=off (orrery_target_fp_determinism) so the link-time
# optimiser can never fall back to its default contraction mode. The golden
# 10k-step hashes (tests/core/test_determinism.cpp) are the check.

include_guard(GLOBAL)

option(ORRERY_LTO "Link-time (interprocedural) optimisation of the first-party targets" OFF)

if(ORRERY_LTO)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT _orrery_ipo_supported OUTPUT _orrery_ipo_output LANGUAGES CXX)
    if(_orrery_ipo_supported)
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
        message(STATUS "ORRERY_LTO: link-time optimisation enabled")
    else()
        message(WARNING "ORRERY_LTO: not supported by this toolchain, building without it: "
            "${_orrery_ipo_output}")
    endif()
endif()
