# ADR-020: Kernel families and device memory in the compute backend

## Context
ADR-019 composed every snippet of `src/render/shaders/lbm` (the files without
`.comp`) into every kernel. Phase 7c adds a second lattice, D3Q19, whose
kernels need their own shared snippet: its storage blocks, uniforms and
tables (`u_nx`, `LBM3_CX`, ...) cannot be composed into the D2Q9 kernels and
the other way round (redefinitions do not compile, and a kernel must not
silently pick up the other lattice's tables).

The D3Q19 lattice is also the first one large enough to exhaust a device:
256^3 nodes need about 2.9 GB of device memory, and one population buffer
(19 binary32 values per node, 1.3 GB at 256^3) can exceed the largest buffer a
shader storage binding may address (`GL_MAX_SHADER_STORAGE_BLOCK_SIZE`, as
low as 2^27 bytes by the GL 4.3 minimum). The specification sizes the grid
"64^3 .. 256^3 by VRAM", so a solver must be able to ask how much the device
holds before it allocates, and refuse with an explicit error.

## Decision
- A snippet named `<family>_common.glsl` is composed only into the kernels
  whose name starts with `<family>_` (`lbm_d2q9_common.glsl` into
  `lbm_d2q9_*`, `lbm_d3q19_common.glsl` into `lbm_d3q19_*`); any other
  snippet is still composed into every kernel. Kernel names stay the
  identity (`<name>.comp.glsl`).
- `IComputeBackend::memory()` returns a `DeviceMemory` with two optional
  fields: the free device memory and the largest buffer one binding can
  address. It is a virtual with a default that knows nothing, so existing
  backends and test doubles need no change. `GlComputeBackend` reports
  `GL_MAX_SHADER_STORAGE_BLOCK_SIZE` and, where the driver offers
  `GL_NVX_gpu_memory_info` or `GL_ATI_meminfo`, the free memory;
  `NullComputeBackend` reports what a test sets with `set_memory`.
- The D3Q19 GPU driver refuses, before allocating, a population buffer above
  the largest binding and a lattice above the free memory. The `IAeroSolver`
  adapter sizes its budget from the free memory when the device reports it,
  and otherwise (and on the CPU) from a conservative cap of 256^3 nodes; a
  lattice above the budget is refused with the largest resolution that
  would fit.

## Consequences
- Each lattice keeps its own GLSL namespace; adding a third family needs no
  backend change. The D2Q9 kernels compile exactly as before.
- Free memory is a hint: drivers report it only through vendor extensions,
  it changes with other processes, and allocation can still fail (that
  failure is reported, never a crash). Where neither extension exists the
  cap applies.
- Rejected: one population buffer per direction (38 bindings, above the
  guaranteed 8), splitting the lattice into bricks (a domain decomposition
  the target sizes do not need), and querying memory through a render
  context API outside the backend (aero must stay free of OpenGL).

## Status
Accepted (amends ADR-019).
