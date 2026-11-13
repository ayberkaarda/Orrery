# ADR-002: Symplectic default integrator, adaptive IAS15

## Context
Long N-body runs are dominated by secular energy and angular momentum drift. Non-symplectic schemes accumulate systematic energy error, while symplectic schemes keep the energy error bounded for Hamiltonian systems at fixed step. Close encounters and atmospheric drag (non-Hamiltonian) need adaptive steps.

## Decision
Leapfrog (kick-drift-kick) is the default; Yoshida 4th order is available for higher accuracy. IAS15 (adaptive, 15th order Gauss-Radau) is used for close encounters and for entry with drag. RK4 exists for tests and comparison only. The step policy uses the pairwise free-fall time with a per-frame substep cap. All integrators sit behind IIntegrator.

## Consequences
Bounded energy error on long runs and a clear accuracy ladder. Adaptive stepping costs more per step, and variable step size breaks strict symplecticity, which is why fixed-step schemes remain the default.

Alternatives rejected: RK4 in production. It is not symplectic, so energy drifts monotonically over many orbits even though its local error is small.

## Status
Accepted; the step policy is amended by ADR-016 (see the amendment below).

## Amendment (2026-11-13)
The step policy above was not implemented as written; this note records what
the code does, and the Decision text is kept as decided. There is no
pairwise free-fall-time step selection. The fixed-step integrators (Leapfrog
KDK, Yoshida 4) step by the experiment's fixed `dt_days`; IAS15 chooses its
own internal steps with its error control (h' = h (epsilon / err)^(1/7))
inside each such step. The per-frame cap is ADR-016's step budget: an
advance takes at most a fixed number of steps (the experiment's
`max_steps_per_frame`, default 4096) and, interactively, a wall-clock
budget, and drops the simulated time beyond it. Choosing a dt suited to close encounters is left to the
experiment, or to IAS15.
