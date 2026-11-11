# ADR-005: OpenGL 4.6 with GLSL compute

## Context
The renderer needs HDR targets, compute shaders for ray marching and marching cubes, and reliable headless operation in CI. The priority of the project is physics, not API plumbing.

## Decision
Use OpenGL 4.6 core with GLSL compute shaders. Headless rendering uses EGL with Mesa software drivers in CI.

## Consequences
Low boilerplate, mature tooling, compute available on target hardware and in CI. No explicit multi-queue control, and the API is in maintenance mode.

Alternatives rejected: Vulkan, because of the boilerplate (memory, synchronisation, pipelines) for no physics benefit; WebGPU-native, because implementations and tooling are still maturing.

## Status
Accepted
