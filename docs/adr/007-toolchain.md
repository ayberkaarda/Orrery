# ADR-007: C++20, CMake and vcpkg

## Context
The project needs deterministic double-precision numerics, tight control of floating-point flags, mature linear algebra (Eigen), mesh libraries (libigl, assimp) and GPU compute.

## Decision
C++20 built with CMake presets; dependencies come from a vcpkg manifest. Floating-point contraction and fast-math are disabled for determinism.

## Consequences
Direct access to the numerical and geometry ecosystem and fine control of compilation. Longer builds and manual memory discipline, mitigated by sanitizers and clang-tidy in CI.

Alternatives rejected: Rust, since the aero and geometry library ecosystem is thinner for this use; Electron, because a browser runtime is a poor host for heavy native compute and GL control.

## Status
Accepted
