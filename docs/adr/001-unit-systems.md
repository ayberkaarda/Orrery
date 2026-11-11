# ADR-001: Unit systems per domain

## Context
Orbital dynamics spans scales from planetary radii to tens of AU. In SI, positions are around 1e11 m and the Sun's GM is about 1.3e20 m^3/s^2, so unit conversions are noisy and obscure the structure of the problem. Flow solvers, by contrast, are naturally expressed in metres, kilograms and seconds, and the lattice Boltzmann method works in dimensionless lattice units (dx = dt = 1).

## Decision
The space domain uses AU / solar mass / day, so the gravitational parameter of the Sun equals the Gauss constant squared (k^2, k = 0.01720209895 rad/day) and typical values are O(1). The aero domain uses SI. LBM runs internally in lattice units with an explicit SI <-> lattice conversion layer that fixes dx, dt and tau from the target Reynolds number and enforces a lattice Mach number below 0.1. Public APIs use strongly typed Quantity<Dim> values, not bare doubles.

## Consequences
Each domain is numerically well scaled. Conversions happen only at documented boundaries (entry bridge, UI, scenario I/O) and are covered by round-trip tests. Cost: boundary code must convert carefully.

Alternatives rejected: a single global unit system. SI everywhere loses conditioning and readability in orbital code; AU/day everywhere makes Reynolds number and pressure reasoning awkward and invites unit mistakes.

## Status
Accepted
