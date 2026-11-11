# Orrery

Orrery is a physics lab: space dynamics (N-body gravity, integrators, black holes,
atmospheric entry) and aerodynamics (LBM, panel method, Newtonian flow), built in C++20
with an OpenGL 4.6 front end. Physics runs in `double`; rendering uses `float` with a
floating origin.

**Status: Phase 0** (skeleton, tooling, ADRs). No physics is implemented yet.

## Build

Requirements: CMake >= 4.1, Ninja, a C++20 compiler (GCC 13, Clang 17 or MSVC 2022),
and [vcpkg](https://github.com/microsoft/vcpkg) in manifest mode.

Linux:

```sh
export VCPKG_ROOT=/path/to/vcpkg
cmake --preset linux-release
cmake --build --preset linux-release
ctest --preset linux-release
```

Windows (in a Developer PowerShell for Visual Studio with x64 tools):

```powershell
$env:VCPKG_ROOT = "C:\vcpkg"
cmake --preset release
cmake --build --preset release
ctest --preset release
```

Linux presets: `linux-debug-asan` (AddressSanitizer/UBSan), `linux-release`,
`linux-ci-headless` (no window, used by CI). Windows presets: `debug-asan`
(AddressSanitizer), `release`, `ci-headless` (no window, used by CI).

## Layers

| Target | Role |
|---|---|
| `orrery_core` | Units, N-body, integrators. No OpenGL/ImGui dependency. |
| `orrery_geom` | Primitives, import, repair, voxelization. No OpenGL dependency. |
| `orrery_aero` | LBM, panel method, Newtonian; abstract `ComputeBackend`. |
| `orrery_atmo` | Atmosphere models, entry dynamics bridge. |
| `orrery_scene` | JSON scenario format, embedded data. |
| `orrery_render` | OpenGL 4.6, shaders, camera, GPU compute backend. |
| `orrery_app` | GLFW + ImGui, main loop, panels. |

Dependency direction is enforced in CI by `tools/check_deps.py`.

## Code quality

`.clang-format` (LLVM base, 4 spaces, 100 columns), `.clang-tidy` (warnings are errors),
cppcheck, and an ASan job all run in CI.
