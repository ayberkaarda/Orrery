# ADR-003: Schwarzschild lensing for black holes, Kerr deferred

## Context
Black hole visualisation needs light bending, an Einstein ring and a disk appearance. Kerr geometry adds frame dragging, an asymmetric shadow and a harder geodesic problem (extra conserved quantity, ergosphere), plus its own dynamics. The v1 scope is space, aero and entry.

## Decision
Light is traced as null geodesics in the Schwarzschild metric using the orbit equation in u = 1/r, integrated with RK4 and a capped step count. Disk emission is a thin-disk temperature profile with Doppler beaming and gravitational redshift. Black hole spin is stored on the body but affects neither rendering nor dynamics in v1. This decision is intentionally temporary: the roadmap (V3, relativity) lifts it.

## Consequences
Simple geometry, cheap enough for half-resolution compute. Rotating black holes are not represented faithfully; this is stated in docs/physics.md.

Alternatives rejected: Kerr in v1. Extra complexity with no benefit for the core deliverables.

## Status
Accepted; superseded-by-roadmap (V3)
