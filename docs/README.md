# Orrery Documentation

Reference documentation for Orrery V1 (space dynamics, aerodynamics and
atmospheric entry). Start with the project [README](../README.md) for
building and running; the pages below describe the physics, the design
decisions and each part of the application.

## Physics and solvers

| Document | Contents |
|---|---|
| [physics.md](physics.md) | Formula index of every implemented formula and model (source, implementation, envelope, approximation bound, test), the approximation tables and the measured validation values |
| [physics_index.json](physics_index.json) | Machine-readable twin of the formula index, checked by `tools/check_docs.py` |
| [aero.md](aero.md) | The three aerodynamic solvers (panel method, modified Newtonian, lattice Boltzmann), regime selection and validity envelopes |
| [lbm.md](lbm.md) | Lattice Boltzmann D2Q9 and D3Q19: collision, boundaries, units, compute backend, GPU precision and validation |
| [atmosphere.md](atmosphere.md) | US Standard Atmosphere 1976, Mars and exponential models, the Cd(Mach) bridge, drag and the entry experiment |
| [geometry-import.md](geometry-import.md) | Primitives, model import, repair and the watertight report, winding number, voxels and SDF, panel meshes |
| [url-import-security.md](url-import-security.md) | Threat model, limits and tests of the URL model loader |

## Rendering and application

| Document | Contents |
|---|---|
| [rendering.md](rendering.md) | Renderer conventions: camera, log depth, HDR chain, headless snapshots, bodies, starfield, trails, user interface, black-hole lensing |
| [aero-visualisation.md](aero-visualisation.md) | Aero view: Cp colour maps, section planes, streamlines and particles, Q-criterion iso-surfaces, force arrows |
| [app.md](app.md) | Application layer: physics thread, time-warp budget, window and workspaces, shortcuts, configuration, error surface, command line and headless rendering |
| [aero-workspace.md](aero-workspace.md) | Aero workspace: geometry, flow conditions, solver thread, experiments and the aero headless renderer |
| [entry-workspace.md](entry-workspace.md) | Entry workspace: setup, profile plots, ground track, experiments and the entry headless renderer |
| [observability.md](observability.md) | Profiling zones, Tracy integration and the `--trace` output |
| [screenshots.md](screenshots.md) | How the README screenshots are regenerated headlessly |

## Planning

| Document | Contents |
|---|---|
| [roadmap.md](roadmap.md) | The three-layer roadmap (V1, V2, V3), abstractions deliberately absent from V1, the coupling pattern, non-goals |
| [limitations.md](limitations.md) | Specification items not implemented in V1 (or done differently), what the code does instead, and their status |

## Architecture decision records

Index with status: [adr/README.md](adr/README.md).

| ADR | Decision |
|---|---|
| [001](adr/001-unit-systems.md) | Space in AU, solar masses and days; aero in SI; LBM in lattice units with a conversion layer |
| [002](adr/002-integrators.md) | Symplectic leapfrog by default, Yoshida 4th order and adaptive IAS15; RK4 for tests only |
| [003](adr/003-schwarzschild-lensing.md) | Schwarzschild lensing for black holes; Kerr deferred to V3 |
| [004](adr/004-pseudo-newtonian-bh.md) | Paczynski-Wiita pseudo-Newtonian black-hole dynamics instead of full GR N-body |
| [005](adr/005-graphics-api.md) | OpenGL 4.6 with GLSL compute |
| [006](adr/006-gpu-compute.md) | GPU compute in GLSL, no CUDA |
| [007](adr/007-toolchain.md) | C++20, CMake and vcpkg |
| [008](adr/008-lbm-solver.md) | Lattice Boltzmann D3Q19 BGK with Smagorinsky subgrid model; no Navier-Stokes finite volumes |
| [009](adr/009-aero-regimes.md) | One solver per aerodynamic regime, recommended automatically with user override |
| [010](adr/010-url-import.md) | URL model import restricted to HTTPS with size and ratio limits and a SHA-256 cache |
| [011](adr/011-threading.md) | Physics on its own thread with triple-buffered snapshots |
| [012](adr/012-entry-dynamics.md) | Entry dynamics with constant mass, no rotation and a tabulated Cd(Mach) |
| [013](adr/013-no-monolithic-domain.md) | No monolithic PhysicsDomain; an abstraction appears at its second user |
| [014](adr/014-typed-coupling.md) | Cross-domain coupling by typed values, not shared state |
| [015](adr/015-roadmap-layers.md) | Three-layer roadmap with no future-domain code in V1 |
| [016](adr/016-app-loop-timing.md) | Integral time accumulator with a dropping step budget |
| [017](adr/017-black-hole-render-strategy.md) | Half-resolution geodesic trace with a full-resolution sky lookup |
| [018](adr/018-geometry-kernels.md) | Geometry kernels in orrery_geom; assimp behind a validation gate |
| [019](adr/019-compute-backend.md) | Compute backend with byte buffers, named kernels and binary32 at the boundary |
| [020](adr/020-compute-kernel-families-and-memory.md) | Kernel families and device memory in the compute backend |
| [021](adr/021-entry-drag-splitting.md) | Atmospheric drag as a split-step velocity kick (Strang splitting) |

## Checks

`py tools/check_docs.py` (`python3` on Linux; also the ctest `docs.check`)
verifies every relative link (inline and reference-style), image and
heading anchor in these pages and the READMEs, and keeps [physics.md](physics.md) and
[physics_index.json](physics_index.json) in step with the sources.
