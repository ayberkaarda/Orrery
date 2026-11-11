# ADR-012: Entry dynamics: constant mass, no rotation, tabulated Cd(Mach)

## Context
Atmospheric entry needs drag on an orbiting probe. A full treatment needs attitude dynamics, ablation and mass loss, and real-gas effects, which would pull in 6-DOF rigid body dynamics and thermochemistry.

## Decision
Drag is F = 1/2 rho v^2 Cd(Mach) A, with Cd sampled from the aero solver at a few Mach numbers and linearly interpolated. Mass is constant, the probe does not rotate (heat shield assumed forward), and heating is an indicative stagnation-point correlation. Drag enters the integrator as an IForceModel. The roadmap (V3, rigid body) lifts these limits.

## Consequences
Simple and testable by a work-energy balance. Cannot predict attitude, ablation or accurate heating; recorded in the approximations table.

Alternatives rejected: 6-DOF with ablation, a candidate for the V3 rigid-body work.

## Status
Accepted; superseded-by-roadmap (V3)
