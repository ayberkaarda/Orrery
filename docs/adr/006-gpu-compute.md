# ADR-006: GLSL compute, no CUDA

## Context
LBM and field visualisation benefit from GPU parallelism. Users may own hardware from any vendor.

## Decision
All GPU compute is GLSL compute, reached from orrery_aero only through the IComputeBackend interface. A CPU (TBB) path serves as fallback and as test ground truth.

## Consequences
Vendor-neutral, and shares the GL context with rendering, which avoids interop copies. GPU and CPU results are not expected to be bit-identical, so tests use tolerances. Peak performance can be lower than hand-tuned CUDA.

Alternatives rejected: CUDA, because it ties the application to one vendor's hardware and toolchain.

## Status
Accepted
