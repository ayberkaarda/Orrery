# Orrery Physics Reference

Skeleton. Each section lists formulas, sources and validity envelopes. Sections are extended by the phase that implements the corresponding solver.

## 1. Units

- Space domain: AU / solar mass / day. The gravitational parameter of the Sun is k^2 with the Gauss constant k = 0.01720209895 rad/day; G*M of other bodies is expressed in the same units (ADR-001).
- Aero domain: SI (m, kg, s).
- LBM: lattice units (dx = dt = 1). Conversion fixes the physical dx and dt from the target Reynolds number, Re = U L / nu, and the lattice viscosity nu_lat = c_s^2 (tau - 1/2), with c_s^2 = 1/3. Lattice Mach number Ma_lat = U_lat / c_s must stay below 0.1 so that density variations (order Ma^2) remain small.
- All physics runs in `double`; rendering uses `float` with a floating origin.

| Constant | Value | Source |
|---|---:|---|
| Astronomical unit (AU) | 149597870700 m | IAU 2012 Resolution B2 |
| Day | 86400 s | IAU 2012 Resolution B2 |
| Gauss constant, k | 0.01720209895 | IAU 1976 |

### 1.1 Implementation (orrery_core, Phase 1)

- `Quantity<Dimension<L, M, T>, UnitSystem>` (`src/core/include/orrery/core/quantity.h`) stores one `double` in the base units of its unit system (`SiUnits` or `SpaceUnits`). Multiplication and division add and subtract the integer exponents at compile time; addition, comparison and assignment require identical dimension and unit system. Construction from and extraction to `double` are explicit (`Quantity(v)`, `.value()`). Angles are a separate `Angle` type in radians.
- Conversion (`to_si`, `to_space` in `units.h`) multiplies or divides by a single factor AU^L * Msun^M * day^T, so a round trip changes a value by at most a few ulps.
- GM_sun in space units is k^2 = 2.959122082855911e-4 AU^3 day^-2; G in space units is also k^2 (AU^3 Msun^-1 day^-2). Converted with the IAU 2012 AU and the 86400 s day, k^2 AU^3 / day^2 = 1.3271244004194e20 m^3 s^-2, within about 7.1e-12 relative (7.08e-12) of the measured heliocentric gravitational constant 1.32712440041e20 m^3 s^-2 (IERS Conventions 2010, TDB-compatible). Gravitational parameters therefore convert exactly between the two systems.
- APPROXIMATION (solar mass in kilograms): Msun = GM_sun / G with G = 6.67430e-11 m^3 kg^-1 s^-2 (CODATA 2018), giving 1.98841e30 kg. G is known only to 2.2e-5 relative, so every mass converted between space and SI units carries that uncertainty. Validity: any use that tolerates 2.2e-5 relative mass error; dynamics in a single unit system, which depend on G*M only, are not affected.

## 2. Space dynamics

- Newtonian gravity: a_i = sum_j G m_j (r_j - r_i) / (|r_j - r_i|^2 + eps^2)^(3/2), Plummer softening eps.
- Direct summation O(N^2) as reference; Barnes-Hut octree with opening angle theta for large N.
- Integrators (ADR-002): Leapfrog KDK (second order, symplectic), Yoshida 4th order (composition of leapfrog substeps), IAS15 (adaptive Gauss-Radau, Rein and Spiegel 2015), RK4 for tests only.
- Black hole (ADR-004): Paczynski-Wiita potential Phi = -GM/(r - r_s), r_s = 2GM/c^2; photon sphere 1.5 r_s, ISCO 3 r_s.
- Light bending (ADR-003): null geodesics in Schwarzschild metric, d^2u/dphi^2 + u = 3GM u^2/c^2 with u = 1/r.
- Diagnostics: osculating Kepler elements, Hill sphere, energy and angular momentum drift.

### 2.1 Body model and Kepler elements (orrery_core, Phase 1)

- `BodyStore` is a structure of arrays in space units (id, name, kind, mass, position x/y/z, velocity x/y/z). Bodies keep insertion order, removal preserves the order of the remaining bodies, and ids increase monotonically and are never reused, so iteration and id lookup (binary search on the sorted id column) are deterministic. `BodyKind` is Star, Planet, Moon, Asteroid, BlackHole{spin a* in [0, 1]} or Probe{aero_ref}. `SimState` holds the simulation time (days), the `BodyStore` and the unit system tag (space).
- APPROXIMATION (probes as test particles): a probe has a mass but exerts no gravitational force (`exerts_gravity` is false). Valid while the probe mass is negligible against every body whose motion matters; the neglected effect on a primary is of relative order m_probe / M_primary.
- Elements -> state. Elliptic (0 <= e < 1, a > 0): Kepler's equation M = E - e sin E; perifocal position (a (cos E - e), a sqrt(1 - e^2) sin E), velocity sqrt(mu a) / r (-sin E, sqrt(1 - e^2) cos E), r = a (1 - e cos E). Hyperbolic (e > 1, a < 0): M = e sinh H - H; position |a| (e - cosh H, sqrt(e^2 - 1) sinh H), velocity sqrt(mu |a|) / r (-sinh H, sqrt(e^2 - 1) cosh H), r = |a| (e cosh H - 1). The perifocal frame is rotated by R3(Omega) R1(i) R3(omega) (Vallado, *Fundamentals of Astrodynamics and Applications*, 4th ed., sections 2.2 and 2.6).
- State -> elements: h = r x v; eccentricity vector e = ((v^2 - mu/r) r - (r . v) v) / mu; vis-viva 1/a = 2/r - v^2/mu; i = atan2(|h_xy|, h_z) (well conditioned near 0 and pi); node, periapsis and true anomaly from atan2 of projections onto the node line N and the in-plane direction h_hat x N.
- Anomaly solvers: safeguarded Newton iteration on a bracket that always contains the root. Elliptic bracket [M - e, M + e] with Danby's start E0 = M + 0.85 e sign(sin M), on M reduced to [-pi, pi). Hyperbolic bracket [asinh(M/e), asinh(M/(e - 1))], started at the upper end where f >= 0 so Newton descends monotonically on the convex function. A Newton step leaving the bracket is replaced by bisection, so convergence is guaranteed; extreme hyperbolic anomalies that overflow return an error.
- Conventions and limits (explicit `KeplerError` results, no silent fallbacks): parabolic orbits (|e - 1| < 1e-9) are rejected; the sign of a must match e; rectilinear states (|h| < 1e-12 |r| |v|) and r = 0 are rejected. For e < 1e-13 the orbit is treated as circular (omega = 0, M measured from the node); for sin i < 1e-14 as equatorial (Omega = 0, angles measured from +x in the direction of motion). The thresholds sit just above rounding noise because this treatment misplaces the state by about 2 e or sin i relative. Above them the undefined angle loses precision as 1e-16 / e or 1e-16 / sin i, while the longitudes (Omega + omega + M, or omega - Omega + M for retrograde orbits) and the Cartesian state stay accurate.
- The elliptic mean anomaly is obtained from the true anomaly (consistent with omega for small e); the hyperbolic one from e sinh H = (r . v) / sqrt(mu |a|), which avoids the cancellation in 1 + e cos(nu) near the asymptotes.
- Measured accuracy (tests/core/test_kepler.cpp, MSVC x64 Release): elements -> state -> elements over a grid of 10080 elliptic cases (a 0.3 to 40 AU, e 0 to 0.99, i 0 to pi, node, periapsis and mean anomaly over all quadrants): max relative error of a 8.5e-14, of e 2.9e-15 (relative to max(e, 0.01)), max absolute angle error 2.3e-14 rad, reconstructed position 4.1e-14 and velocity 2.0e-14 relative. 216 hyperbolic cases (a -0.5 to -10 AU, e 1.01 to 10, M -5 to 10): a 1.3e-13, e 9.2e-14, angles 4.4e-14, position 4.6e-14, velocity 3.7e-14. Test bound: 1e-12. Earth-like orbit: T = 2 pi sqrt(a^3 / k^2) = 365.2568983 days for a = 1 AU.
- APPROXIMATION (two-body Keplerian motion): elements are the osculating conic of the instantaneous state about a point mass with mu = G (M + m); perturbations by other bodies, oblateness and relativistic corrections are ignored. Exact at the instant of conversion; period, mean motion and mean-anomaly propagation are valid only while perturbations are small over the time span considered.

## 3. Aerodynamics solvers

| Solver | Model | Intended regime |
|---|---|---|
| Lattice Boltzmann (D3Q19 / D2Q9, BGK, Smagorinsky) | Weakly compressible kinetic scheme recovering Navier-Stokes in the low Mach limit | Low to moderate Re, arbitrary geometry, unsteady |
| Panel method (source + doublet, Kutta condition) | Incompressible inviscid potential flow | Thin bodies and wings at low angle of attack |
| Modified Newtonian impact theory | Cp = Cp_max sin^2(theta) on windward faces, zero in shadow | Hypersonic, blunt bodies |

Notes:

- LBM forces come from the momentum exchange method; Cd and Cl are reported instantaneous and as a moving average.
- Panel method: dense LU up to 4000 panels, GMRES above. Pressure drag from the potential solution is separate from skin friction, which is estimated from a flat-plate correlation (Schlichting) and reported separately.
- Newtonian: Cp_max from the Rayleigh pitot relation for the given Mach number.
- Regime selection (ADR-009) recommends a solver from Re and Mach; the user may override. Out-of-envelope use emits a warning and is not blocked.
- LBM is a numerical method and the Navier-Stokes equations are a model; the documentation and UI keep that distinction.
- For the isothermal, single-relaxation-time BGK LBM in lattice units, the shear kinematic viscosity follows from the second-order equilibrium and relaxation of the non-conserved momentum flux: nu = c_s^2 (tau - 0.5), with c_s^2 = 1/3 for the standard D2Q9 and D3Q19 lattices. Stability requires tau > 0.5; this relation does not by itself establish accuracy, which also depends on resolution and low lattice Mach number (Kruger et al., *The Lattice Boltzmann Method*, 2017).
- Halfway bounce-back places a no-slip wall halfway between a fluid node and its solid neighbor; it is a simple voxel-wall treatment whose accuracy depends on wall placement and resolution. Zou-He velocity or pressure boundaries reconstruct unknown incoming populations from the prescribed macroscopic state while enforcing mass and momentum constraints; they apply at boundary nodes and require compatible flow direction and lattice populations (Kruger et al., *The Lattice Boltzmann Method*, 2017).
- Panel methods solve an incompressible, inviscid potential-flow boundary value problem, enforcing the body boundary condition with source and doublet distributions; the Kutta condition selects circulation at a sharp trailing edge. They do not resolve viscous boundary layers or separated flow (Katz & Plotkin, *Low-Speed Aerodynamics*).
- Modified Newtonian impact theory estimates windward surface pressure from local inclination using a stagnation pressure coefficient; it is an approximate hypersonic model and does not resolve shock-layer structure or chemistry (Anderson, *Hypersonic and High-Temperature Gas Dynamics*).

## 4. Atmosphere and entry

- Atmosphere models: US Standard Atmosphere 1976 (tabulated to 86 km, exponential above), a simple tabulated Mars profile, and a generic exponential rho = rho0 exp(-h/H).
- Drag on a probe: F = 1/2 rho v^2 Cd(Mach) A, with Cd(Mach) sampled from the aero solver at several Mach numbers and linearly interpolated (ADR-012).
- Stagnation-point heat flux: Sutton-Graves correlation, q proportional to sqrt(rho / R_nose) v^3. Indicative only.
- Entry runs as a chain core state -> atmosphere (density, Mach) -> drag force -> integrator (ADR-014). Validation: kinetic energy loss equals the work done by drag within 1 percent.
- Outputs: altitude, speed, deceleration (g load), heat flux time series, ground track; outcomes are landing or skip-out.

## 5. APPROXIMATIONS

Every simplification used by the solvers, with the range in which it is acceptable.

| ID | Approximation | Where | Validity envelope | Outside the envelope |
|---|---|---|---|---|
| A1 | Uniform density assumed for centre of mass | Geometry reference quantities | Solid, roughly homogeneous bodies | Hollow or layered bodies give wrong CoM and moment arms |
| A2 | Smagorinsky subgrid closure in LBM | LBM, ADR-008 | Applied for Re above about 2000; adds eddy viscosity from the local strain rate | Over-damps transition and thin shear layers; not a substitute for resolved DNS |
| A3 | Weak compressibility, lattice Mach below 0.1 | LBM | Subsonic flow, density change of order Ma^2 | Compressible or transonic flow is not captured; conversion layer refuses larger Ma_lat |
| A4 | Staircase (voxel) walls with halfway bounce-back | LBM | Resolution sufficient for the body features | Curved-wall pressure and skin friction are first-order accurate at best |
| A5 | Inviscid, incompressible potential flow; fixed wake | Panel method | Attached flow, thin bodies, low angle of attack | Separation, stall, shocks, and viscous drag are absent; drag comes only from induced contribution plus a separate friction estimate |
| A6 | Flat-plate skin friction estimate | Panel method | Attached boundary layer | Curved, separated or transitional flow |
| A7 | Modified Newtonian impact theory | Hypersonic solver | Mach above 5 | Warning below Mach 5; ignores shock layer shape and shadow-side pressure |
| A8 | Sutton-Graves stagnation-point heating | Entry heating | Indicative only, for blunt stagnation-point estimates | No radiative heating, no catalytic or chemical effects |
| A9 | Constant mass, no ablation | Entry dynamics, ADR-012 | Short exposure or low heating | Heat shield mass loss changes mass and shape |
| A10 | No rotation; heat shield always forward | Entry dynamics, ADR-012 | Statically stable capsule shape | Tumbling, attitude oscillation, lift-induced skip are not modelled |
| A11 | Tabulated Cd(Mach) with linear interpolation | Atmosphere bridge | Smooth Cd variation between table points | Sharp changes near Mach 1 are under-resolved with few points |
| A12 | Paczynski-Wiita pseudo-Newtonian potential | Black hole dynamics, ADR-004 | Radial structure at ISCO and horizon scale | No frame dragging, gravitational waves, or correct periapsis precession |
| A13 | Schwarzschild metric only; thin-disk emission | Black hole rendering, ADR-003 | Non-rotating hole | Spinning holes (Kerr) have different shadow and disk appearance |
| A14 | Plummer softening | N-body force | Used to bound close-encounter forces | Alters dynamics at separations comparable to eps |
| A15 | Barnes-Hut multipole truncation | N-body force | Controlled by opening angle theta | Larger theta increases acceleration error |
| A16 | Rayleigh + Mie atmosphere with 16-sample ray march | Rendering | Visual plausibility | Not a radiative transfer solution |
| A17 | Fixed-size step cap per frame for adaptive integration | Integrators | Normal time-warp | Extreme time-warp or violent encounters may lag simulated time |

## 6. Validation set

Located in `tests/aero_validation/`. Each case cites its literature reference in the test source.

| Case | Solver | Expected | Tolerance / range |
|---|---|---|---|
| Circular cylinder, Re 100 to 200 | LBM 2D | Strouhal number, drag coefficient | St 0.16 to 0.20; Cd 1.2 to 1.5 |
| Sphere, Re 10 to 1000 | LBM 3D | Cd versus Re follows Schiller-Naumann | within 15 percent |
| NACA 0012, angle of attack 0 to 8 degrees | Panel | Lift slope near 2 pi per radian; embedded XFOIL reference table | within 10 percent |
| Flat plate normal to flow | LBM | Cd about 1.28 | within 15 percent |
| Apollo-type capsule, Mach 20 | Newtonian | Cd about 1.3 (published capsule data) | within 10 percent |
| User-supplied model | any | No validation | UI shows an "unvalidated geometry" badge |

Related project-level criteria: sphere Cd at Re 100 between 1.0 and 1.2; cylinder Strouhal number at Re 150 between 0.16 and 0.20; NACA 0012 lift slope 2 pi per radian within 10 percent.

Core validation (analytic): two-body analytic orbits, energy drift, Barnes-Hut versus direct, IAS15 figure-eight orbit, ISCO stability, determinism hash, Kepler element round trip.

## Scenario format

Scenarios are JSON documents loaded by `orrery_scene` (`orrery/scene/scenario.h`); bundled examples live in `assets/scenarios/`. Schema version 1:

- `version` (integer, required): must be 1; checked before any other key.
- `units` (required): `length` = `"AU"`, `mass` = `"Msun"`, `time` = `"day"`, `angle` = `"deg"` or `"rad"`. Only the angle unit is selectable; everything else is in space units (ADR-001).
- `metadata` (required): `source` and `epoch` strings (provenance of the initial conditions), optional `description`.
- `name` (optional string), `time` (optional, days since the epoch, default 0).
- `bodies` (required array). Each body has `name` (unique, non-empty), `kind` (`star`, `planet`, `moon`, `asteroid`, `blackhole`, `probe`), `mass` (finite, non-negative; positive for probes and black holes), and exactly one of:
  - `state`: `position` [3] in AU and `velocity` [3] in AU/day, absolute in the scenario's inertial frame;
  - `elements`: `parent` (name of another body), `a` (AU), `e`, `i`, `raan`, `argp`, `ma` (mean anomaly), relative to the parent with mu = G (M_parent + m_body), G = k^2.
  Black holes may give `spin` in [0, 1]; probes may give `aero_ref` (name of a Cd(Mach) table). These keys are rejected for other kinds.

Validation is strict: unknown or duplicate keys, wrong types, non-finite numbers, negative eccentricity, unknown parents, probe parents, parent cycles and duplicate names are rejected, and every error carries the path of the offending value (for example `bodies[2].elements.e`). Orbits the Kepler conversion rejects (parabolic, inconsistent semi-major axis, inclination outside [0, pi]) propagate the `KeplerError`. Bodies enter the `BodyStore` in file order; parents are resolved first.

The serializer writes numbers in the shortest form that reads back to the same double, so a state survives save and load bit for bit (acceptance criterion: relative error below 1e-12); element angles in degrees cost at most a few ulp. Probe `aero_ref` names are numbered in order of first use (`AeroRef` 1, 2, ...; 0 means none).

Experiment records (`orrery/scene/experiment.h`) hold `version` (1), `domain`, `inputs`, `conditions`, `solver` (`name`, `version`, `settings`) and `result`; parameter values are numbers or strings. `domain` accepts only `"space"`, `"aero"` and `"entry"`.

Number literals: a literal whose significand has a nonzero digit but whose value lies below the double range (for example `-1e-400`) would round to a signed zero and slip past the sign checks; it is rejected with `invalid_value` at its path. A literal written as zero (`0`, `-0.0`, `0e-400`) is an intentional zero and is accepted. `-0.0` compares equal to 0, so it is a zero mass (still rejected for probes and black holes) or a zero eccentricity; its sign bit is kept in memory and on save. Subnormal values (for example `5e-324`) are nonzero and accepted.

Resource limits (`orrery/scene/limits.h`, shared by scenarios and experiment records). Reading and parsing check them before the corresponding memory is allocated, and an input beyond any of them fails with `limit_exceeded`:

| Limit | Value |
|---|---|
| Document size, as text or file | 16 MiB (16,777,216 bytes) |
| Nesting depth of objects and arrays (root = 1) | 64 |
| Members of one object or elements of one array | 200,000 |
| JSON values in one document | 2,000,000 |
| Bodies in one scenario | 100,000 |
| Distinct probe `aero_ref` names in one scenario | 65,536 (far below 2^32, so an `AeroRef` never wraps to 0) |

Files are read in chunks with checked reads: a read error after a successful open is an `io_error` (never a short or empty read), and an empty file reads as empty text (then a `syntax_error`). No exception crosses the scene API: running out of memory while reading, parsing, validating or building is reported as `out_of_memory`, and any other unexpected exception as `internal_error`.