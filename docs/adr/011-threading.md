# ADR-011: Physics thread with triple-buffered snapshots

## Context
Rendering at variable frame rate must not stall the fixed-step simulation, and the simulation must not stall input.

## Decision
Physics runs on its own thread with a fixed-timestep accumulator. Rendering reads immutable SimSnapshot objects through a lock-free triple buffer; the reverse direction uses a SimCommand queue. Mutex blocking across threads at frame level is prohibited.

## Consequences
Smooth rendering and stepping that is independent of frame rate. Requires immutable snapshots and discipline about shared state, and adds a copy cost per snapshot.

Alternatives rejected: a single thread, where a heavy step directly drops frame rate and time-warp couples to rendering.

## Status
Accepted
