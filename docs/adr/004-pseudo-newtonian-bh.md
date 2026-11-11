# ADR-004: Pseudo-Newtonian black hole dynamics

## Context
Full general-relativistic N-body dynamics needs either numerical relativity or a post-Newtonian expansion for every pair, which is not tractable for interactive many-body work. What users need is a strong, qualitatively correct central potential: a horizon, an innermost stable circular orbit and a photon sphere.

## Decision
Black holes act through the Paczynski-Wiita potential, Phi = -GM/(r - r_s), with r_s = 2GM/c^2. It places the ISCO at 3 r_s (6GM/c^2), the same as Schwarzschild. Bodies crossing r_s are absorbed with mass and momentum conserved. Tidal disruption uses a Roche criterion.

## Consequences
Cheap, stable and testable (ISCO stability test). It is a potential model, not GR: no correct periapsis precession, no gravitational waves, no frame dragging. This is recorded in the approximations table.

Alternatives rejected: full GR N-body. Intractable at interactive rates and out of scope.

## Status
Accepted
