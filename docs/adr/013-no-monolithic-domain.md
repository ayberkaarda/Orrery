# ADR-013: No monolithic PhysicsDomain; abstract on second use

## Context
Physics domains differ widely (ODEs on particles, grid PDEs, panel linear systems). A single interface designed up front tends to fit none of them and leaks assumptions.

## Decision
No PhysicsDomain, ISimulation or IPhysicsModule base class. An interface exists only if at least two concrete implementations exist in v1 (IIntegrator, IForceModel, IAeroSolver, IComputeBackend). Candidates such as IFluidSolver or IFieldSolver are only listed in the roadmap.

## Consequences
Less speculative code, and abstractions shaped by real use. Some refactoring will be needed when a second user appears; this is accepted deliberately.

Alternatives rejected: a generic simulation module interface.

## Status
Accepted
