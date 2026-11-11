# ADR-015: Three-layer roadmap with no future-domain code in v1

## Context
The long-term scope (fluid, thermal, waves, electromagnetism, plasma, chaos, optics, statistical mechanics, rigid body, relativity) is large. Designing all of it at once yields speculative structure.

## Decision
Roadmap in three layers: V1 space, aero and entry; V2 fluid, thermal, waves and acoustics; V3 electromagnetism, plasma and MHD, chaos, optics, statistical mechanics, rigid body and elasticity, relativity, reaction-diffusion. Version gates control transitions. While V1 is in progress no directories, classes, enum values or JSON fields are created for later domains.

## Consequences
Focused delivery and honest interfaces. Some refactoring is expected when V2 begins (extracting shared infrastructure).

Alternatives rejected: planning everything in one release.

## Status
Accepted
