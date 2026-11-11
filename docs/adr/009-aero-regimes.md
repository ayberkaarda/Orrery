# ADR-009: Separate solvers per aerodynamic regime

## Context
No single method is valid across the regimes of interest. Potential flow ignores viscosity and separation; LBM is limited to low Mach number; impact theory applies only to hypersonic flow.

## Decision
Three solvers behind IAeroSolver: LBM (low to moderate Re, arbitrary shapes), panel method (inviscid, incompressible, thin bodies at low angle of attack) and modified Newtonian (hypersonic). A selector recommends a solver from Re and Mach; the user can override. Use outside the validity envelope produces a warning but is not blocked.

## Consequences
Each solver is simple and has a clear validity envelope. Users must understand which one they are running, and results across solvers are not directly comparable.

Alternatives rejected: one all-purpose solver, which would be either wrong somewhere or enormously complex.

## Status
Accepted
