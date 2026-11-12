# ADR-016: Integral time accumulator with a dropping step budget

## Context
ADR-011 puts physics on its own thread with a fixed-timestep accumulator. The
accumulator must give the same final state whatever the frame pacing (the
determinism contract), and time warps up to 1e7 must not make a slow machine
fall ever further behind.

## Decision
Owed simulated time is an integer count of nanoseconds. A tick adds
`wall_ns * warp` with an integer warp in [1, 1e7] and a wall time clamped to
250 ms; a step consumes `round(dt * 86400e9)` ns while the integrator steps by
exactly `dt`. A tick takes at most a fixed number of steps (and,
interactively, a wall-clock budget); simulated time beyond the budget is
dropped and reported as a limited warp with the achieved rate. Each tick
credits the elapsed time under the state that held during it before applying
the commands that arrived meanwhile, and command work per tick is bounded. The render
thread wakes the physics thread with a non-blocking semaphore release per
frame and sends commands through a concurrent queue.

## Consequences
The step count depends only on the total credited wall time, so runs are
bit-identical for any pacing and match direct integrator calls. A warp the
machine cannot sustain degrades visibly instead of spiralling. The warp is
integral (1-2-5 presets in the UI), and a positive wall-clock budget makes
interactive runs machine dependent, so headless runs and tests use the
count budget only.

Alternatives rejected: a double-precision accumulator (pacing-dependent
rounding changes the step count), variable dt (breaks symplectic
reproducibility), and carrying unbounded debt (spiral of death).

## Status
Accepted
