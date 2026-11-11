# Architecture Decision Records

| ADR | Title | Status |
|---|---|---|
| [001](001-unit-systems.md) | Unit systems per domain | Accepted |
| [002](002-integrators.md) | Symplectic default integrator, adaptive IAS15 | Accepted |
| [003](003-schwarzschild-lensing.md) | Schwarzschild lensing for black holes, Kerr deferred | Accepted; superseded-by-roadmap (V3) |
| [004](004-pseudo-newtonian-bh.md) | Pseudo-Newtonian black hole dynamics | Accepted |
| [005](005-graphics-api.md) | OpenGL 4.6 with GLSL compute | Accepted |
| [006](006-gpu-compute.md) | GLSL compute, no CUDA | Accepted |
| [007](007-toolchain.md) | C++20, CMake and vcpkg | Accepted |
| [008](008-lbm-solver.md) | Lattice Boltzmann D3Q19 BGK with Smagorinsky subgrid | Accepted |
| [009](009-aero-regimes.md) | Separate solvers per aerodynamic regime | Accepted |
| [010](010-url-import.md) | Restricted URL model import | Accepted |
| [011](011-threading.md) | Physics thread with triple-buffered snapshots | Accepted |
| [012](012-entry-dynamics.md) | Entry dynamics: constant mass, no rotation, tabulated Cd(Mach) | Accepted; superseded-by-roadmap (V3) |
| [013](013-no-monolithic-domain.md) | No monolithic PhysicsDomain; abstract on second use | Accepted |
| [014](014-typed-coupling.md) | Typed data flow for cross-domain coupling | Accepted |
| [015](015-roadmap-layers.md) | Three-layer roadmap with no future-domain code in v1 | Accepted |

Each record has Context, Decision, Consequences (including rejected alternatives) and Status sections.
