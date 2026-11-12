# Aerodynamic solvers (orrery_aero)

Orrery computes aerodynamic loads with three separate solvers, one per flow
regime (ADR-009). None of them is valid everywhere; each one states its
validity envelope, and a computation outside it still runs but carries a
warning in its result. Formulas, sources and the list of approximations are
in `physics.md` (section "Aerodynamic solvers (orrery_aero, Phase 7a)").

| Solver | Header | Model | Validity envelope |
|---|---|---|---|
| Panel method, 2D and 3D | `orrery/aero/panel_method.h` | Inviscid, incompressible potential flow (APPROXIMATION A5) | Mach <= 0.3, Re >= 1e5, angle of attack and sideslip within 10 degrees |
| Modified Newtonian | `orrery/aero/newtonian.h` | Impact theory, Cp = Cp_max sin^2(theta) (APPROXIMATION A7) | Mach >= 5 |
| Lattice Boltzmann | phases 7b and 7c | Weakly compressible kinetic scheme (APPROXIMATIONS A2 to A4) | Mach <= 0.3, Re <= 1e5 |

## Common interface

`orrery/aero/aero_solver.h` declares `IAeroSolver`:

```cpp
solver.setup(geometry, conditions); // validates, throws std::invalid_argument
while (!solver.step()) {}           // direct solvers finish in one step
const AeroResult& r = solver.result();
```

- `FlowConditions` (`flow_conditions.h`): speed, density, dynamic viscosity,
  temperature, angle of attack, sideslip, gamma and gas constant, all SI. The
  Mach number is derived from the speed of sound sqrt(gamma R T) and never
  given. Body axes: x downstream along the body, y along the span, z up; the
  free stream at zero angles blows along +x.
- `AeroGeometry`: a closed, outward triangle mesh for 3D solvers, an optional
  trailing-edge polyline for lifting bodies, a 2D section for the 2D panel
  method, and the reference area, length and moment centre (the user chooses
  the area: `geom::reference_quantities`).
- `AeroResult` (`aero_result.h`): Cd, Cl, Cy, Cm; force and moment; the
  pressure drag and the induced drag; a skin-friction estimate reported
  separately and never added to Cd; the surface pressure field (one sample
  per surface element: panel, triangle or wall sample); an optional velocity
  field on a regular grid (volume solvers); residual history, iteration count
  and convergence flag; an optional stagnation heat flux; and the validity
  warnings. `set_coefficients` derives the coefficients from force and moment
  for any solver.

## Regime selection and warnings

`select_solver` (`regime.h`) recommends a solver from the Mach and Reynolds
numbers:

| Conditions | Recommendation |
|---|---|
| Mach >= 5 | Newtonian |
| Mach < 0.3, Re >= 1e5 | panel method |
| Mach < 0.3, Re < 1e5 | lattice Boltzmann |
| 0.3 <= Mach < 1 | panel method, flagged: no solver is valid |
| 1 <= Mach < 5 | Newtonian, flagged: no solver is valid |

The user may override the recommendation; the override is honoured and the
envelope warnings of the solver actually chosen are returned. Nothing is ever
blocked.

Every solver runs `check_validity` (or `check_envelope` with its own
envelope) on its conditions and appends the warnings to its result. A warning
has a stable code, `"<solver>.<quantity>"` (`panel.mach`, `panel.reynolds`,
`panel.angle_of_attack`, `panel.sideslip`, `newtonian.mach`, `lbm.mach`, ...),
and a message with the value and the envelope. Solver-specific warnings use
the same scheme (`panel.sideslip_ignored`, `panel.not_converged`).

## Panel method

- 2D (`PanelSolver2D`): Hess-Smith, constant source on each straight panel
  plus one vorticity common to all panels, fixed by the Kutta condition at
  the trailing edge (first and last panel of the section). Sections come from
  `naca_section_geometry`, which uses the NACA generator of orrery_geom.
  Moments are taken about the quarter chord.
- 3D (`PanelSolver3D`): constant source and doublet on every triangle with
  the zero internal perturbation potential (Dirichlet) condition, after Katz
  and Plotkin. Lifting bodies shed flat wake strips (geom::add_wake) along the
  free stream from the trailing edge; the wake strength of each segment is the
  upper-minus-lower doublet jump at the trailing edge (Kutta condition).
  Dense LU up to `PanelOptions::dense_limit` (4000) unknowns, restarted GMRES
  above. Lift and pressure drag from the surface pressure, induced drag and a
  cross-check of the lift from the Trefftz plane.
- Friction: a flat-plate estimate (laminar Blasius below Re = 5e5,
  Prandtl-Schlichting above) times wetted area over reference area, reported
  as `cd_friction`.

Known limits beyond the potential-flow model itself:

- Constant-strength triangles are first order: the sphere pressure error
  halves when the panel size halves (rms 0.052 at 480 panels, 0.026 at 1984).
- The Kutta condition averages all panels at each trailing-edge vertex. A
  triangulation that is not symmetric between upper and lower surface (the
  diagonal split of `geom::naca_wing`) still leaves a small antisymmetric
  circulation near the tips: at zero incidence the AR 6 wing shows CDi =
  2.5e-4 instead of 0, growing as the trailing-edge panels get thinner.
  Comparing only the two triangles on each trailing-edge segment made this
  two times larger at 16 and 24 stations and seventy times larger at 48
  (CDi = 0.14), which is why the averaging is used.
- Thick sections lift more than thin-airfoil theory: the inviscid NACA 0012
  slope is 1.098 times 2 pi, near the 10 percent limit of the project
  criterion; measured sections lift less because of the boundary layer.

## Modified Newtonian theory

`NewtonianSolver` integrates Cp = Cp_max sin^2(theta) over the windward
triangles (zero in the shadow), with Cp_max from the Rayleigh pitot formula
for the free-stream Mach number and gamma. With a nose radius in
`NewtonianOptions`, the Sutton-Graves stagnation heat flux is reported as an
indication only. Below Mach 5 the result carries `newtonian.mach`.

## Validation

`tests/aero_validation` (executable `orrery_test_aero_validation`) holds the
literature cases; `tests/aero` the analytic unit tests. Measured values are
listed in `physics.md`.
