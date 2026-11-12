# ADR-019: Compute backend with byte buffers, named kernels and binary32 at the boundary

## Context
ADR-006 puts all GPU compute behind an `IComputeBackend` interface so that
`orrery_aero` stays free of OpenGL; the OpenGL implementation lives in
`orrery_render`. Phase 7b is the first user (the D2Q9 lattice Boltzmann
solver), and Phase 7c (D3Q19) will be the second. Three questions had to be
settled when the interface was written:

- How typed the interface is. A templated or typed buffer API would leak
  element types (and single precision) into `orrery_aero`, whose sources must
  stay free of `float` (tools/check_deps.py, precision check).
- How kernels are identified. The GL backend must find the shader source and
  compile it; `orrery_aero` must not include render headers or GLSL.
- What precision the device works in. Consumer GPUs run binary64 at 1/16 to
  1/64 of the binary32 rate, and the D2Q9 target (2048 x 1024 at 200 steps/s)
  is set for binary32; the CPU reference is binary64 and the acceptance test
  asks for an L2 difference below 1e-4.

## Decision
- Buffers are untyped byte ranges (`create_buffer`, `upload`, `readback`,
  `release_buffer`, RAII `ScopedBuffer`). The only conversion between host
  doubles and device binary32 is `float32_pack.cpp`, the single file of
  `orrery_aero` on the `FLOAT_ALLOWLIST`. Floating-point uniforms are
  `Float32Uniform{double}` values that the backend rounds once, when it hands
  them to the device; integer uniforms are passed unchanged.
- A kernel is a name. The GL backend maps name `X` to the embedded shader
  `src/render/shaders/lbm/X.comp.glsl`, composed with the snippets of the same
  directory, compiles it on first use and caches it with its uniform table
  (types checked on every dispatch). A dispatch gives the number of work
  items; the backend rounds up to work groups from the shader's local size.
- Ordering is explicit and minimal: dispatches see earlier storage writes
  only after `barrier()`; `upload` and `readback` are ordered after all
  earlier dispatches by the backend itself.
- `NullComputeBackend` keeps buffers in host memory, validates and records
  every dispatch, and runs host functions registered for kernel names. It
  tests the GPU drivers' protocol (layout, bindings, ping-pong, conversions)
  without a GPU.
- The LBM kernels store populations shifted by their lattice weight,
  f_i - w_i, in binary32, and every host-side sum (forces, mass) is done in
  double after readback.

## Consequences
- `orrery_aero` builds and is tested on machines without OpenGL; it is in
  `GRAPHICS_FREE_TARGETS` of the dependency checker.
- A new GPU solver adds `.comp.glsl` files and a driver in `orrery_aero`; the
  backend needs no change. Kernel names are strings, so a typo is found at
  the first dispatch (an error, never a crash); the render tests check that
  every embedded kernel compiles.
- Precision: on the shared tests the binary32 GPU solver agrees with the CPU
  reference within 6e-5 (velocity L2). During the symmetry-breaking start of
  vortex shedding the physical instability amplifies the rounding difference
  to 7e-4 before the shedding locks in and the difference returns to 1e-4
  (docs/lbm.md, "GPU precision"). A binary64 kernel variant is the
  documented way out if that transient must match.
- Rejected: a typed `Buffer<T>` API (puts `float` into `orrery_aero`),
  passing GLSL source through the interface (render concerns in aero),
  CUDA or OpenCL backends (ADR-006), binary64 by default (about 2x memory
  traffic and 1/16 rate arithmetic on the target GPUs for a difference that
  only shows during an instability transient).

## Status
Accepted
