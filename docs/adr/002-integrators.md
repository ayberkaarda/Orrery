# ADR-002: Symplectic default integrator, adaptive IAS15

## Context
Long N-body runs are dominated by secular energy and angular momentum drift. Non-symplectic schemes accumulate systematic energy error, while symplectic schemes keep the energy error bounded for Hamiltonian systems at fixed step. Close encounters and atmospheric drag (non-Hamiltonian) need adaptive steps.

## Decision
Leapfrog (kick-drift-kick) is the default; Yoshida 4th order is available for higher accuracy. IAS15 (adaptive, 15th order Gauss-Radau) is used for close encounters and for entry with drag. RK4 exists for tests and comparison only. The step policy uses the pairwise free-fall time with a per-frame substep cap. All integrators sit behind IIntegrator.

## Consequences
Bounded energy error on long runs and a clear accuracy ladder. Adaptive stepping costs more per step, and variable step size breaks strict symplecticity, which is why fixed-step schemes remain the default.

Alternatives rejected: RK4 in production. It is not symplectic, so energy drifts monotonically over many orbits even though its local error is small.

## Status
Accepted
