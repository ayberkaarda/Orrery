# ADR-021: Atmospheric drag as a split-step velocity kick

## Context
ADR-012 puts drag into the integrator as an IForceModel. The core force-model interface is a function of positions only (`accelerations(bodies, positions, out)`), and the integrators rely on that: IAS15 predicts positions at its Gauss-Radau substeps but has no velocity predictor, and the symplectic schemes kick with position-dependent accelerations. Drag depends on the velocity relative to the air. Passed to an integrator unchanged, a drag model could only read the velocities stored at the start of the call and would freeze the drag over the whole step (first order, and inconsistent with IAS15's error control).

## Decision
`AtmosphericDrag` implements `core::IForceModel` and reads the velocities from the BodyStore it is given, but EntryExperiment never hands it to an integrator. Each entry step is a Strang splitting of length dt:

1. drag kick over dt/2 at fixed positions (classical RK4 on dv/dt = a_drag(r, v), each stage through `AtmosphericDrag::accelerations` with the stage velocity written into a scratch BodyStore);
2. gravity step over dt with an unchanged core integrator (IAS15 by default) and `NewtonianGravity`;
3. drag kick over dt/2.

The kinetic energy change of each kick is the exact work of the drag in that kick, so the energy balance has an exact bookkeeping term and an independent check (trapezoidal integral of F . v over the series). core stays untouched; the coupling stays a typed value flow (ADR-014).

## Consequences
Second order in dt overall (the splitting error), symmetric, deterministic. With the default dt = 0.5 s the peak deceleration of a lunar-return entry changes by about 1e-4 relative between dt = 0.5 s and 0.1 s, and the energy balance closes to 2e-5. IAS15 restarts its step prediction after each kick (the state no longer matches the committed one); with dt far below the orbital period it accepts one internal step per call. Velocity-dependent forces elsewhere would reuse the same pattern.

Alternatives rejected: a velocity-aware IForceModel with a velocity predictor in IAS15 (changes core's interface and every integrator for one user, ADR-013); drag frozen at the start of each integrator call (first order, and the energy balance would only hold for very small steps).

## Status
Accepted (Phase 9a)
