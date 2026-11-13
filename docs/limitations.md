# Planned items not implemented in V1

Features that were planned for V1 but are not implemented, or are done
differently, in one place. Each row names the item, what the code does
instead, and the status. The general limitations of the models (validity
envelopes, approximations) are in the [README](../README.md#limitations)
and in [physics.md](physics.md); this page lists only what was planned and
is absent or done differently.

| Planned item | What V1 does instead | Status |
|---|---|---|
| Dual-filter bloom pass in the HDR chain | An analytic glare sprite marks the stars (Moffat core, r^-3 wings; [rendering.md](rendering.md#10-bodies)); the HDR chain tone-maps without a bloom pass ([rendering.md](rendering.md#4-hdr-chain)) | not implemented; the HDR values a bloom pass would need are already there |
| Hill spheres and spheres of influence as objects; a cancellable worker that integrates ghost (predicted) trajectories | The Laplace sphere of influence only chooses the primary of the inspector's Kepler elements (`app.soi_primary`, [physics.md](physics.md#formula-index)) | not implemented; no V1 feature depends on it |
| Hill/SOI sphere and ghost-orbit visuals in the space view | Trails of the past path only ([rendering.md](rendering.md#12-trails)) | not implemented (needs the Hill sphere and ghost-trajectory items above) |
| Rayleigh + Mie atmosphere with a 16-sample ray march for bodies | The entry view draws a single-colour exponential haze in 16 samples; planets in the space view have no atmosphere (A16 in [physics.md](physics.md)) | not implemented; the haze is a visual cue, not a scattering model |
| Step policy from the pairwise free-fall time with a per-frame substep cap | Fixed `dt_days` per experiment, IAS15's own error control, and ADR-016's step budget as the per-frame cap ([ADR-002 amendment](adr/002-integrators.md#amendment-2026-11-13)) | done differently; recorded in ADR-002 |
| Shared `ScalarField3D` / `VectorField3D` and `IFieldSampler` | The concrete `render::FlowField`, `ScalarGrid`, `VectorGrid` and `aero::VelocityField` | deferred to V2 phase 12 ([roadmap.md](roadmap.md#3-abstractions-what-exists-and-what-deliberately-does-not)), when a second user appears (ADR-013) |
