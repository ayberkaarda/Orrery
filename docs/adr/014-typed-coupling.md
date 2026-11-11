# ADR-014: Typed data flow for cross-domain coupling

## Context
Domains interact: orbit state to atmosphere, aero coefficients to drag, drag to integrator. Shared mutable state hides dependencies and breaks determinism and testing.

## Decision
Coupling uses explicit value types crossing boundaries (AeroCoefficientTable, DragForce), wired in EntryExperiment. Domains never read each other's internal state. Each link is tested alone, then an end-to-end energy balance checks the whole chain.

## Consequences
Clear dependencies, easy per-link tests, deterministic behaviour. More small types and explicit wiring.

Alternatives rejected: shared world state or an ECS, which lets any domain read and write anything and makes conservation errors hard to localise.

## Status
Accepted
