# ADR-008: Lattice Boltzmann D3Q19 BGK with Smagorinsky subgrid

## Context
Aerodynamic flow around arbitrary imported geometry is needed, time dependent, at low to moderate Reynolds number. Body-fitted grid generation is hard for untrusted meshes, whereas voxelised solids fit LBM naturally. LBM is local and maps well to GPUs.

## Decision
Use D3Q19 with BGK collision (D2Q9 for 2D tests) and a Smagorinsky subgrid viscosity for higher Reynolds numbers. Boundaries: halfway bounce-back on solids, Zou-He velocity inlet, convective or extrapolated outlet. Forces come from momentum exchange. No finite-volume Navier-Stokes solver in v1.

## Consequences
Simple, parallel, robust on voxelised geometry. BGK becomes unstable at low relaxation time, the scheme is weakly compressible (requires lattice Mach below 0.1), Smagorinsky is a crude closure, and staircase walls limit accuracy near surfaces; all are listed in the approximations table. LBM recovers Navier-Stokes only in a limit, so it is presented as a method, not as the same model.

Alternatives rejected: FVM / OpenFOAM, due to meshing complexity and distribution burden.

## Status
Accepted
