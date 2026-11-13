# Orrery Roadmap

Orrery grows in three layers (ADR-015): V1 (space, aero, entry) is what this
repository implements; V2 (fluid, thermal, waves) and V3 (electromagnetism,
plasma, chaos, optics, statistical mechanics, rigid bodies, relativity, MHD,
reaction-diffusion) are planned. **While V1 is open the roadmap is a document,
not code**: no directory, class, interface, enum value or JSON field exists
for a later domain. This page records what is planned, which abstractions are
deliberately absent, and the coupling pattern every future domain follows.

## 1. Three layers

| Version | Domains | Main coupling experiment |
|---|---|---|
| **V1** (this repository) | Space (N-body, black holes, orbits); Aero (lattice Boltzmann, panel method, modified Newtonian); Atmosphere and entry | Gravity -> Atmosphere -> Aero -> Drag -> Orbit |
| **V2** | Fluid Lab (incompressible Navier-Stokes, compressible Euler, instabilities, wakes); Thermal (heat equation, convection, radiation, thermodynamics); Waves and acoustics | Rayleigh-Benard (fluid + thermal); entry heat flux -> thermal analysis; shocks (fluid + waves) |
| **V3** | Electromagnetism; Plasma and MHD; Chaos; Optics; Statistical mechanics and diffusion; Rigid body and elasticity; Relativity (special relativity to Schwarzschild and Kerr); Reaction-diffusion | Charged probe (EM + N-body); magnetosphere (plasma + space); MHD (fluid + EM); 6-DOF entry (rigid + atmosphere); optical lens against black-hole lensing |

Order in which new domains are added from V2 on: fluids, thermal,
waves and acoustics, electromagnetism, chaos, optics, plasma, statistical
mechanics, rigid body, relativity, MHD, general relativity.

Target experiment at the end of V2: a capsule entering the Mars atmosphere,
showing in **one** experiment its trajectory, atmospheric density, Mach
number, Cd, lift and drag, pressure distribution, heat flux and g-load.

Versions are gated. V2 opens only after the V1 final gate, a `v1.0` tag set
by the developer, a measured report against the project's numerical targets,
and complete `physics.md` and `roadmap.md`; V3 opens the same way after
`v2.0`. Within a version no code for a later version is written.

## 2. What V1 contains

| Domain | Library | Reference |
|---|---|---|
| Units, bodies, gravity, integrators, Barnes-Hut, IAS15, black holes, collisions, events | `orrery_core` | [physics.md](physics.md), ADR-001 to ADR-004 |
| Primitives, import, repair, winding number, voxels and SDF, panels, URL import | `orrery_geom` | [geometry-import.md](geometry-import.md), [url-import-security.md](url-import-security.md), ADR-010, ADR-018 |
| Panel method, modified Newtonian, lattice Boltzmann D2Q9 and D3Q19, regime selection | `orrery_aero` | [aero.md](aero.md), [lbm.md](lbm.md), ADR-008, ADR-009, ADR-019, ADR-020 |
| US76, Mars and exponential atmospheres, Cd(Mach) bridge, drag, entry | `orrery_atmo` | [atmosphere.md](atmosphere.md), ADR-012, ADR-014, ADR-021 |
| Scenarios and experiment records | `orrery_scene` | [physics.md, scenario format](physics.md#scenario-format) |
| OpenGL 4.6 renderer, black-hole lensing, aero and entry views | `orrery_render` | [rendering.md](rendering.md), [aero-visualisation.md](aero-visualisation.md), ADR-005, ADR-006, ADR-017 |
| Application, workspaces, physics thread, headless rendering | `orrery_app` | [app.md](app.md), [aero-workspace.md](aero-workspace.md), [entry-workspace.md](entry-workspace.md), ADR-011, ADR-016 |

### V1 phases

V1 was built in phases; the documents name them where a feature arrived
("Phase 8a"). Each phase was merged as one pull request.

| Phase | Content |
|---|---|
| 0 | Project skeleton, tooling, the first ADRs |
| 1 | Units, body model, scenario format |
| 2 | Gravity and integrators |
| 3 (3a, 3b) | Octree (Barnes-Hut), IAS15 and the bundled scenarios; black holes, collisions and other events |
| 4 (4a, 4b, 4c) | Render foundation; space scene visuals and the user interface layer; application loop, physics thread and headless command line |
| 5 | Black-hole rendering |
| 6 (6a, 6b) | Geometry pipeline; URL model loader |
| 7 (7a, 7b, 7c) | Solver interface, panel method and Newtonian theory; compute backend and 2D lattice Boltzmann; 3D lattice Boltzmann |
| 8 (8a, 8b) | Aero visualisation; Aero workspace |
| 9 (9a, 9b) | Atmospheres and the entry bridge; Entry workspace |
| 10 (10a) | Application polish and README; observability (profiling zones, trace output) |
| 11 | Performance, link-time optimisation, CI restructuring, packaging, documentation and screenshots |
| 12 onwards | V2 (section 5) |

## 3. Abstractions: what exists and what deliberately does not

The rule (ADR-013): **an abstraction is extracted only at its second real
user.** There is no `PhysicsDomain`, `ISimulation` or `IPhysicsModule` base
class; domains that share no behaviour are not forced to look alike.

Interfaces that V1 has, because V1 already has at least two implementations
of each:

| Interface | Implementations in V1 |
|---|---|
| `core::IIntegrator` | `LeapfrogKdk`, `Yoshida4`, `Rk4`, `IAS15` |
| `core::IForceModel` | `NewtonianGravity`, `BarnesHutGravity`, `PaczynskiWiitaGravity`, `atmo::AtmosphericDrag` |
| `aero::IAeroSolver` | `PanelSolver2D`, `PanelSolver3D`, `NewtonianSolver`, `LbmAeroSolver2D`, `LbmAeroSolver3D` |
| `aero::compute::IComputeBackend` | `GlComputeBackend` (in `orrery_render`), `NullComputeBackend` |
| `atmo::IAtmosphere` | `UsStandardAtmosphere1976`, `MarsAtmosphere`, `ExponentialAtmosphere` |

Small shared pieces that exist because V1 needs them: `ConservationDiagnostics`
(energy, momentum and angular momentum drift), the `Experiment` record whose
`domain` accepts only `"space"`, `"aero"` and `"entry"`, and the determinism
contract (fixed reduction order, no fast-math, golden hashes) that every new
domain inherits.

Candidates that are **deliberately not in V1**. Each is listed with the user
that would justify it; none may be created before that user exists.

| Candidate | Extracted from (first user) | Second user that justifies it | Planned in |
|---|---|---|---|
| `FieldGrid<T, Dim>`: uniform Cartesian grid, ghost cells, MAC or collocated layout, CPU and GPU mirrors | the LBM lattices (`Lbm2DSolver`, `Lbm3DSolver`) | incompressible Navier-Stokes | V2, phase 12 |
| `IFieldSolver` with exactly four members: setup, step, fields, diagnostics | the LBM solvers behind `IAeroSolver` | Navier-Stokes, Euler, heat equation | V2, phase 12 |
| `IStencilKernel` / a GPU dispatch template | the LBM compute kernels of `IComputeBackend` | Navier-Stokes and wave FDTD kernels | V2, phase 12 |
| `FieldViewer`: sections, iso-surfaces, streamlines, colour maps | the aero visualisation (`orrery_render` aero view) | thermal and EM fields | V2, phase 12 |
| `ScalarField3D` / `VectorField3D` + `IFieldSampler` | today the concrete `render::FlowField` and `aero::VelocityField` | thermal and EM fields | V2, phase 12 |
| Multigrid Poisson solver (GPU V-cycle, red-black Gauss-Seidel) | none in V1 | Navier-Stokes pressure, EM potential, implicit thermal steps | V2, phase 12 |
| Workspace registry (data-driven tabs) | the three hard-coded workspaces | a fourth workspace | V2, phase 12 |
| `Experiment.domain` values `"fluid"`, `"thermal"`, `"waves"` with schema version 2 and migration | the V1 schema | the first V2 experiment | V2, phase 12 |
| `CouplingGraph`: typed edges between domains, no cycles, explicit unit conversion per edge | the entry chain wired in `EntryExperiment` | the second coupling (Rayleigh-Benard) | V2, phase 18 |
| `IParticleSolver` | the N-body core | particle-in-cell plasma, molecular dynamics | V3 (phases 21, 25), only if both appear |
| Boris pusher in the integrator family | none | charged probes, particle-in-cell | V3, phase 20 |

Phase 12 of V2 is a refactoring phase with no new physics: the shared
infrastructure is moved out of the V1 code, every V1 test and golden image
must still pass unchanged, and LBM performance may not drop by more than 5
percent. Outside phase 12, V2 phases may not generalise V1 code.

## 4. Coupling pattern

Coupling is **typed data flow, not shared state** (ADR-014). A domain never
reads another domain's internal state; a value with a defined type and unit
crosses the boundary. V1 has exactly one chain, wired explicitly in
`atmo::EntryExperiment` with no globals or singletons:

```
core (orbital state)  --position, velocity-->   atmo (density, Mach)
aero (Cd(Mach) samples, reference area) --AeroCoefficientTable--> atmo (drag force)
atmo (DragForce)  --split-step velocity kick-->  core (integrator)
```

Each link is tested on its own, then the whole chain is tested end to end
against a conservation law:

| Link | Value crossing it | Tested by |
|---|---|---|
| state -> atmosphere | altitude, relative wind, density, temperature, Mach number | `tests/atmo/test_atmosphere.cpp` (US76 tables, Mars fit, "Mach number of a state") |
| aero solver -> table | `AeroCoefficientTable` (Mach, Cd, reference area) | `tests/atmo/test_aero_table.cpp` ("Cd table built from the aerodynamic solver for the Apollo capsule") |
| table -> drag | `DragForce` = -1/2 rho abs(v_rel) v_rel Cd(M) A | `tests/atmo/test_drag.cpp` ("Drag force F = 1/2 rho v^2 Cd(Mach) A against the relative wind", "AtmosphericDrag as a core force model on probes") |
| drag -> integrator | Strang split step: drag kick, gravity step, drag kick (ADR-021) | `tests/atmo/test_entry.cpp` ("Gravity integrators agree on the split step", "Stiff drag follows the analytic quadratic-drag solution") |
| whole chain | mechanical energy loss equals drag work within 1 percent | `tests/atmo/test_entry.cpp` ("Entry energy balance: mechanical energy loss equals drag work": 2.1e-5 at dt 0.5 s) |

This is the template for every later coupling (fluid + thermal through
Boussinesq buoyancy, EM + particles through the Lorentz force, fluid + EM in
MHD, rigid body + atmosphere in 6-DOF entry):

1. Name the value type that crosses each boundary, with its units and the
   frame it is expressed in. Unit conversion happens at the edge, never inside
   a domain.
2. Wire the chain explicitly in one experiment object; no domain holds a
   pointer to another domain's state.
3. Two-way coupling only through an explicit sub-step (as the Strang split of
   ADR-021 or a Boussinesq sub-step); the graph of domains has no cycles.
4. Test each link alone against an analytic or published reference.
5. Test the chain end to end with a balance that closes (energy, momentum,
   charge or mass) and state the tolerance.
6. Keep determinism: fixed iteration order, no fast-math, a golden hash or a
   bit-identical run-to-run test.
7. Document every new simplification as an `APPROXIMATION` in the code and a
   row in [physics.md](physics.md); "method" (numerical scheme) and "model"
   (equations) stay separate, so LBM and Navier-Stokes appear side by side,
   never one in place of the other.

## 5. V2: Fluid Lab, Thermal, Waves and Acoustics

V2 keeps every V1 rule and adds four: abstractions are extracted only in the
refactoring phase 12; every experiment is labelled "model (PDE) + method
(scheme)", for example `Navier-Stokes / projection-MAC`,
`Navier-Stokes / LBM-D3Q19`, `Euler / FV-HLLC`; no PDE solver closes its
phase without an analytic or benchmark reference ("looks right" is not
acceptance); and all grid solvers share the single `FieldGrid`.

**Fluid Lab** (phases 13 to 15). An incompressible Navier-Stokes solver on a
MAC staggered grid with Chorin projection, MacCormack advection and implicit
Crank-Nicolson viscosity, validated on Poiseuille and Couette flow, the
lid-driven cavity against Ghia et al. (1982) and the cylinder Strouhal number
against the V1 LBM. A compressible Euler solver (finite volume, HLLC Riemann
solver, MUSCL-Hancock with minmod limiting) on the Sod, Lax and Shu-Osher
tubes, oblique shocks on a wedge and bow shocks on blunt bodies, shown next
to the V1 Newtonian pressures. Instabilities: Kelvin-Helmholtz and
Rayleigh-Taylor growth rates, the Strouhal-Reynolds curve, the Blasius
boundary layer and separation.

**Thermal** (phase 16). Heat equation in one to three dimensions, explicit
for teaching and implicit (ADI or Crank-Nicolson with multigrid) by default,
with Dirichlet, Neumann and Robin boundaries and a material table. Natural
convection by Boussinesq coupling to Navier-Stokes (de Vahl Davis cavity),
surface-to-surface radiation with simple view factors, a 0D thermodynamics
lab (ideal-gas processes, Carnot and Otto cycles, P-V and T-S diagrams), and
a bridge from the V1 Sutton-Graves heat flux into a 1D heat-shield slab
(still without ablation).

**Waves and acoustics** (phase 17). Damped and driven oscillators reusing the
V1 Yoshida integrator, FDTD for the wave equation with PML absorbing
boundaries, standing waves, interference, diffraction, dispersion and the
Doppler effect, linearised-Euler acoustics with the speed of sound from the
thermal field, and the shock bridge (steepening of a large pulse in Euler,
agreement with acoustics in the small-amplitude limit). No audio output.

**Couplings and hardening** (phases 18 and 19). Rayleigh-Benard convection
(critical Rayleigh number 1708 within 10 percent), the single-panel Mars
entry with energy balance within 2 percent, sound refraction in a
temperature gradient, the `CouplingGraph` with cycle detection; nightly
large-grid tests, a V2 section of `physics.md`, `docs/numerics.md` with the
stability condition and measured numerical diffusion of every scheme, and
`v2.0`.

Numerical targets include Navier-Stokes 2D 1024^2 at 60 steps/s and 3D
128^3 at 20 steps/s on a GTX 1660-class GPU, multigrid Poisson 256^3 below
50 ms, the Ghia profiles within 3 percent and Sod L1 error below 0.01 at 400
cells, with zero regressions of V1 tests and golden images.

## 6. V3: EM, Plasma and MHD, Chaos, Optics, Statistical Mechanics, Rigid Body, Relativity, Reaction-Diffusion

Phase order matters in V3: electromagnetism, then plasma, then MHD; chaos,
optics, statistical mechanics, rigid body and reaction-diffusion are
independent of each other; relativity touches the core and runs alone. MHD
is the riskiest phase and may ship with an "experimental" badge, which is
then shown, not hidden. Lifting a V1 decision (ADR-003 Kerr, ADR-012 6-DOF)
happens only in its phase and with a new ADR.

**Electromagnetism.** Coulomb superposition and field lines, Biot-Savart
for wires, loops and solenoids, the Boris pusher added to the integrator
family (Larmor motion, E x B and grad-B drifts, magnetic mirrors), Maxwell
FDTD on the Yee grid with PML, and a charged probe flying under gravity and
the Lorentz force by operator splitting (bit-identical to V1 when q = 0).
**Plasma.** Electrostatic and electromagnetic particle-in-cell (cloud-in-cell
weighting, Esirkepov current deposition): plasma frequency, Landau damping,
two-stream instability, Debye screening, Van Allen trapping in a dipole
field, and a Chapman-Ferraro magnetopause estimate. **MHD** (experimental):
ideal MHD with HLLD and constrained transport (div B at machine precision),
Brio-Wu, Orszag-Tang, magnetic rotor and Alfven waves.

**Chaos.** Double and triple pendulum, Lorenz, Rossler, Duffing, van der Pol
and Henon-Heiles with Dormand-Prince and Yoshida integrators; Poincare
sections, Lyapunov spectra, bifurcation diagrams, basins of attraction, and
the three-body sensitivity experiment on the V1 scenarios. **Optics.** A
geometric ray tracer (Snell, total internal reflection, lenses, prisms,
dispersion, ABCD matrices), wave optics from the FDTD solvers, and a
side-by-side of a thin lens with Schwarzschild lensing. **Statistical
mechanics.** Hard-sphere and soft-sphere gases recovering Maxwell-Boltzmann
and PV = NkT, Brownian motion and the Einstein relation, and the Ising model
with Metropolis and Wolff updates (2D critical temperature 2.269 within 1
percent).

**Rigid body and elasticity.** 6-DOF bodies with quaternions and inertia
tensors computed from V1 meshes, sequential-impulse contact with GJK/EPA,
mass-spring lattices, Euler-Bernoulli beams and modal analysis, and the
6-DOF entry that supersedes ADR-012 (trim angle against Apollo data; still no
ablation). **Relativity.** A special-relativity lab, Schwarzschild and Kerr
geodesics (Mercury's perihelion advance of 42.98 arcsec per century, Kerr
ISCO), Kerr lensing that supersedes ADR-003, and a proper-time diagnostic
that never changes the Newtonian plus Paczynski-Wiita N-body dynamics.
**Reaction-diffusion.** Gray-Scott, Turing patterns, Fisher-KPP front speeds
and the Belousov-Zhabotinsky Oregonator on the shared GPU field grid.

V3 closes with the coupling experiments (magnetosphere, MHD disk as input to
the black-hole renderer, 6-DOF Mars entry across five domains, gas
microscopics against the heat equation), hardening and a project report of
every version's numerical targets with the open deviations, and `v3.0`.

## 7. Non-goals

Not planned in any version (they may appear only as documentation on a
future "V4 candidate" list):

- ablation and material burning, finite-element structural analysis,
  quantum mechanics, full general-relativistic N-body dynamics;
- propulsion, manoeuvre planning and mission design;
- k-epsilon or SST turbulence models, claims of DNS beyond LES, multiphase
  flow, surface tension, combustion, phase change; a claim of validated
  transonic results in any version;
- procedural planetary surfaces, networking or multiple users, mobile or web
  builds, macOS (no OpenGL 4.6), audio output;
- licence or copyright verification of user-loaded models (the user's
  responsibility).

Out of scope for V1 but planned: Navier-Stokes and Euler (V2), Kerr and
frame dragging (V3 relativity), 6-DOF entry and heat-shield thermal analysis
(V3 rigid body, V2 thermal), rigid contact and fluid-structure interaction
(V3). "Easy to add" is not a reason to add them early.

## 8. ADR numbering for V2 and V3

The original plan reserved the numbers 016 to 023 for the V2 decisions and
024 to 033 for the V3 decisions. V1 has already used 016 to 021 (app loop timing, black
hole render strategy, geometry kernels, compute backend, kernel families,
entry drag splitting; see the [ADR index](adr/README.md)). Future records
therefore take the next free number when they are written, and each states
which planned decision it records:

| Planned decision | Version |
|---|---|
| `FieldGrid` and four-member `IFieldSolver` extracted from V1 LBM and aero code | V2 |
| Navier-Stokes: MAC grid, Chorin projection, MacCormack advection | V2 |
| Euler: finite volume, HLLC, MUSCL | V2 |
| One GPU multigrid Poisson solver for every elliptic problem | V2 |
| Thermal: implicit ADI or Crank-Nicolson by default | V2 |
| Boussinesq coupling only; no fully compressible thermal flow | V2 |
| Waves: FDTD with PML; no spectral method | V2 |
| `CouplingGraph` as an acyclic graph; two-way coupling only by explicit sub-steps | V2 |
| Boris pusher in the core integrator family; charged probes by operator splitting | V3 |
| Maxwell FDTD on the Yee grid with PML; no finite elements | V3 |
| Particle-in-cell with cloud-in-cell weighting and Esirkepov current | V3 |
| MHD with HLLD and constrained transport; experimental release allowed | V3 |
| Chaos ODEs with Dormand-Prince, Yoshida for Hamiltonian systems | V3 |
| Ising with Metropolis and Wolff | V3 |
| 6-DOF entry, superseding ADR-012 (no ablation) | V3 |
| Kerr lensing and geodesics, superseding ADR-003 | V3 |
| Rigid contact by sequential impulses | V3 |
| Relativity as diagnostic and comparison only; N-body stays Newtonian plus Paczynski-Wiita | V3 |
