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
| A13 | Schwarzschild metric only; thin-disk emission | Black hole rendering, ADR-003, ADR-017 (details R1 to R10 in "Black hole rendering") | Non-rotating hole | Spinning holes (Kerr) have different shadow and disk appearance |
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

## Gravity and integrators (orrery_core, Phase 2)

Headers: `force_model.h` (`IForceModel`, `NewtonianGravity`), `integrator.h` (`IIntegrator`, `LeapfrogKdk`, `Yoshida4`, `Rk4`, `make_integrator`), `conservation.h` (`ConservationDiagnostics`), `state_hash.h` (`state_hash`). Everything is in space units (AU, Msun, day).

### Force model

- Newtonian gravity by direct O(N^2) summation with Plummer softening eps (eps = 0 is exact):
  a_i = sum over sources j != i of G m_j (r_j - r_i) / (|r_j - r_i|^2 + eps^2)^(3/2), with G = k^2 = 2.959122082855911e-4 AU^3 Msun^-1 day^-2 (Gauss constant).
- Potential energy consistent with the force: U = - sum over source pairs i < j of G m_i m_j / (|r_j - r_i|^2 + eps^2)^(1/2), so a_i = -grad_i U / m_i also with softening (checked by central differences to 1e-7 relative).
- Probes are test particles (`exerts_gravity` false): they are accelerated by every source, exert nothing, and do not appear in U. Integrating with or without a probe leaves the other bodies bit-identical (tested for all three integrators).
- Zero-mass bodies have no field: a pair whose source mass is 0 contributes nothing (no 0 * infinity or 0 / 0 for coincident bodies), while a zero-mass body is still accelerated by every positive-mass source. A zero-mass body on top of a point source (eps = 0) remains singular.
- Numeric range: for squared distances r^2 + eps^2 in [2^-200, 2^200] and masses that are 0 or in [2^-280, 2^280] Msun the direct formula is used (all intermediates are normal doubles). Any other pair is computed with the separation and eps scaled by an exact power of two (frexp/ldexp) and the factors ordered as (G m / d_s^2) u 2^(-2e), so no unscaled inverse cube is formed: separations of 1e110 AU, eps of 1e-200 or 1e200 AU and coincident bodies with positive eps give accurate finite forces and U, and force = -grad U still holds. Only a result that is itself outside the double range overflows or underflows.
- Contract: the direct sum is the reference model for N <= 256. Measured cost at N = 256 (32640 pairs): 0.15 to 0.17 ms per force evaluation, varying from run to run (MSVC x64 Release, single thread; observational, not asserted; the test times the whole leapfrog loop and divides by the force evaluations). The range check costs nothing measurable: a same-binary comparison against the loop without it gave 0.147-0.158 ms versus 0.151-0.153 ms with bit-identical accelerations. Larger N is correct but scales as N^2.
- Fixed summation order: pairs (i, j) are visited as i = 0..N-1, j = i+1..N-1 in body order, so each acceleration accumulates its terms in ascending source index; U is reduced in the same order. No unordered containers, no threads.
- APPROXIMATION (Plummer softening, see also A14): 1/r^2 is replaced by r / (r^2 + eps^2)^(3/2) to bound close-encounter forces. Relative force error about 1.5 eps^2 / r^2 for r >> eps; dynamics are altered for r comparable to eps. eps = 0 disables it; the solar-system tests use eps = 0.
- APPROXIMATION (Newtonian point masses): no general relativity, oblateness, tides or radiation forces. The largest neglected planetary effect is relativistic (Mercury's perihelion advance of 43 arcsec per century, relative force correction about 3e-8). Valid for separations far above the bodies' radii.

### Integrators (ADR-002)

| Integrator | Scheme | Order | Symplectic | Force evaluations per step |
|---|---|---|---|---|
| `LeapfrogKdk` (default) | v += a dt/2; x += v dt; v += a dt/2 | 2 | yes | 2 |
| `Yoshida4` | Leapfrog KDK composed with weights (w1, w0, w1), merged half-kicks: kick w1/2, drift w1, kick (w1+w0)/2, drift w0, kick (w0+w1)/2, drift w1, kick w1/2 | 4 | yes | 4 |
| `Rk4` (tests only) | classical Runge-Kutta on (x, v) | 4 | no | 4 |

- Yoshida (1990) coefficients: w1 = 1 / (2 - 2^(1/3)) = 1.3512071919596578, w0 = -2^(1/3) / (2 - 2^(1/3)) = -1.7024143839193153; 2 w1 + w0 = 1 and 2 w1^3 + w0^3 = 0. 2^(1/3) is a correctly rounded literal, so the coefficients do not depend on the platform's `cbrt`.
- Why symplectic by default: the N-body problem is Hamiltonian. A fixed-step symplectic map conserves a shadow Hamiltonian within O(dt^p) of the true one, so the energy error is bounded and oscillates; RK4's energy error grows linearly in time even though its local error is small.
- `step(SimState&, dt, const IForceModel&)`: fixed step, dt must be finite (negative dt integrates backwards). On a non-finite result (for example coincident sources with eps = 0) the state is left unchanged and `IntegrationError::non_finite_state` is returned. Accelerations are not cached between steps, so a step depends only on the state it is given.
- APPROXIMATION (fixed-step time discretisation): the global error scales as dt^order and is set by the shortest dynamical time (pericentre passage). Valid when dt is a small fraction of it; close encounters need adaptive steps, which this interface does not provide.

### Conservation diagnostics

- `ConservationDiagnostics::measure` returns total mass, kinetic energy sum m v^2 / 2, potential energy from the force model (same softening), total energy, linear momentum sum m v, angular momentum sum m r x v (about the origin), centre of mass and its velocity. Only sources of gravity are included: probes receive momentum without reaction, so including them would break conservation by construction.
- Drifts against a baseline: |E - E0| / |E0|; |P - P0| / sum m |v|; |L - L0| / sum m |r x v|; |V_cm - V_cm0| / (sum m |v| / M). The scales replace |P0| and |L0| because those may vanish (barycentric frame). A zero scale gives drift 0 for an unchanged quantity and +infinity otherwise. The vector norms (`norm` in vec3.h) scale by an exact power of two when the squared length is outside [2^-900, 2^1000], so tiny or huge positive masses (tested with 1e-200 and 1e200 Msun) give the correct drift instead of 0, infinity or NaN.

### Measured acceptance values (tests/core/test_integrators.cpp, test_conservation.cpp; MSVC x64 Release)

| Criterion | Setup | Measured | Bound |
|---|---|---|---|
| Two-body vs analytic Kepler | a = 1.3 AU, e = 0.3, i = 12 deg, Sun + 3e-6 Msun, dt = T/2000; position error at T/4, T/2, T, 10 T | Yoshida4 5.8e-10, 1.4e-9, 2.7e-9, 2.7e-8 AU; Leapfrog 5.3e-6, 2.1e-5, 6.6e-5, 6.6e-4 AU; RK4 1.3e-11, 3.6e-11, 7.2e-11, 1.1e-9 AU | 5e-8 / 1e-3 / 2e-9 AU |
| Kepler period | a = 1 AU, e = 0.5, Yoshida4 dt = T/1000, five successive periapsis passages | 365.256350 d each, relative error 2.4e-10 vs 2 pi sqrt(a^3 / mu) | 1e-9 |
| Circular two-body, 100 orbits, Leapfrog | a = 1 AU, dt = T/150000 = 2.4e-3 d | 3.8e-7 AU | < 1e-6 AU |
| Same with Yoshida4 | dt = T/1200 = 0.30 d | 4.4e-7 AU | < 1e-6 AU |
| Sun + 8 planets, 1000 years, Yoshida4 | dt = 0.25 d (1461000 steps, about 1.6 s, observational), energy sampled every 50 steps | max abs(dE/E0) 1.4e-10, final 2.6e-14 | < 1e-9; every sample finite |
| Momentum (same run) | | max linear momentum drift 6.0e-14, angular momentum drift 1.3e-13 | 10 DBL_EPSILON sqrt(steps) = 2.7e-12 |
| Convergence order | e = 0.5, one orbit, step count doubled twice | Leapfrog 2.000, 2.000; Yoshida4 3.998, 3.999; RK4 4.13, 4.07 | expected order +- 0.15 |
| RK4 secular vs symplectic bounded | e = 0.5, dt = T/200, 1000 orbits | RK4 abs(dE/E0) 9.3e-5 after 100 orbits, 9.3e-4 after 1000 (linear); Leapfrog max 2.7e-3 over orbits 1-100 and over 901-1000; Yoshida4 max 2.0e-5 in both windows | RK4 ratio 8 to 12; symplectic max not growing (factor < 1.05) |
| Time reversibility | e = 0.6, 1000 steps forward then 1000 back | Leapfrog 2.5e-13 AU, Yoshida4 3.4e-13 AU | 1e-12 AU |

- Choice of dt for the 1000-year run: the maximum energy error is set by Mercury's periapsis passages (e = 0.21, P = 88 d). With dt = 0.5 d (176 steps per Mercury orbit) the maximum was 2.2e-9, above the 1e-9 target; dt = 0.25 d (352 steps per orbit) gives 1.4e-10. The dt = 0.5 d figure is an observation from a development run and is not re-run or asserted by the tests (it would add about 0.8 s); only the dt = 0.25 d bound is enforced. The error at the end of the run (2.6e-14) is far smaller than the maximum because the symplectic energy error oscillates rather than accumulates; it is printed and required to be finite, not bounded separately.
- Sampled maxima: every sampled drift (energy, momentum, angular momentum, and the per-step energy errors of the secular/bounded comparison) is checked with std::isfinite before it enters a maximum, and a non-finite sample fails the bound. A plain std::max(m, NaN) returns m and would discard it; a unit test of the accumulator covers this.
- Linear and angular momentum are conserved exactly by the pairwise-symmetric force up to rounding; their drift grows as a rounding random walk, about DBL_EPSILON sqrt(steps).
- Initial conditions of the solar-system run: Sun plus the eight planets (Earth and Moon merged) from the approximate J2000 mean elements of Standish (JPL, "Keplerian Elements for Approximate Positions of the Major Planets", table 1) and IAU 1976 reciprocal masses, shifted to the barycentre. They are realistic for conservation tests, not an ephemeris.

### Determinism contract

- Same initial state, dt, force model and integrator give a bit-identical state. The integration loop uses only IEEE-754 +, -, *, / and sqrt in a fixed order; all are correctly rounded, and fp contraction and fast-math are off project-wide (`/fp:precise`, `-fno-fast-math -ffp-contract=off`). Pairs outside the ordinary numeric range additionally use frexp and ldexp, which are exact (ldexp rounds only when its result is subnormal, as IEEE scaleB specifies); the golden scenario never leaves the ordinary range.
- `state_hash`: 64-bit FNV-1a (offset basis 14695981039346656037, prime 1099511628211) over the time and, per body in store order, id, kind tag (`BodyKindTag` value: star 0, planet 1, moon 2, asteroid 3, black_hole 4, probe 5), kind payload (IEEE-754 bits of a black hole's spin, a probe's `aero_ref` value, 0 otherwise), mass, position x, y, z, velocity x, y, z and radius; doubles contribute their IEEE-754 bit patterns, every word is widened to 64 bits and folded least significant byte first. Names are not hashed. The kind is hashed because it decides source membership: a planet and a probe with identical columns have different dynamics.
- Golden scenario (`golden_run` in tests/core/gravity_test_support.h): the barycentric Sun + 8 planets state written as hexadecimal double literals, plus a 1e-12 Msun probe at (0, 1, 0) AU with velocity (-0.01720209895, 0, 0) AU/day; Plummer eps = 1e-4 AU; 10000 steps of 1 day.

| State | Golden hash |
|---|---|
| Initial state (t = 0) | 0x8ad1887966f2ea12 |
| Leapfrog KDK, 10000 steps | 0x08ca0a8c4993debe |
| Yoshida4, 10000 steps | 0xd0c645137b27a3cd |
| RK4, 10000 steps | 0xab3b17d03159374d |
| IAS15 (default tolerance), 10000 output steps of 10 days (92483 adaptive steps, 1 rejected) | 0xf9cc3e89244d1214 |
| Barnes-Hut (theta 0.5, eps 1e-4 AU, leaf capacity 1) + Leapfrog KDK, 10000 steps; 1, 2, 4, all and default threads | 0x904687bcd8a69556 |

- The first four hashes are pinned by tests/core/test_determinism.cpp, the IAS15 hash by test_ias15.cpp and the Barnes-Hut hash by test_barnes_hut.cpp (every thread count must give the same pinned value). Adding the radius word (Phase 3b; radius 0 for every golden body) changed every value; the trajectories did not change, since neither the integrators nor the force models of the golden runs were modified (the kind-tag format before it gave 0xaf61005ef03001b2, 0xe0fa7b48f95f973e, 0x9edf045b7f8169ad, 0xec2fbc333855be6d). Each new golden test was checked to fail under a deliberate deterministic mutation: reordering one Barnes-Hut multipole accumulation (`ax += radial * rx + tensor * qrx` evaluated as `(ax + tensor * qrx) + radial * rx`) and truncating the last IAS15 Gauss-Radau spacing (h_7 to 0.977520613561287); the other IAS15 and Barnes-Hut tests, including the run-to-run and thread-count comparisons, still pass under these mutations. The golden uses 10-day outputs because with 1-day outputs every internal step is clipped to the output and the step controller never acts. Two other truncations left the hash unchanged (below rounding in this scenario, not a coverage gap of the hash): d_coef[0][3], which only seeds the corrector iteration, and c_coef[0][2] (3e-15 relative).
- Platforms verified: MSVC 19.44 x64 Release (`/O2 /fp:precise`) produces all six hashes. GCC 15.2 `-O3 -fno-fast-math -ffp-contract=off` on x86-64 Linux (WSL Ubuntu, standalone driver over the core sources) reproduced the previous-format pins of the four direct-force runs and the same bits for scaled-path forces and potentials; the radius-format values, IAS15 and Barnes-Hut (oneTBB) have been measured on MSVC only. Clang and non-x86 targets have not been checked.
- The golden initial state is given as literals on purpose: building it from Kepler elements goes through sin, cos and atan2, whose last-bit results differ between C runtime libraries (measured: the MSVC and glibc barycentric states differ by 1 ulp in several components, which changes every hash). Determinism across platforms therefore holds for the integration loop; scenario set-up that uses transcendental functions is reproducible only per C runtime.
## 3b: Black holes, collisions and events (orrery_core, Phase 3b)

Sources: `black_hole.h`, `paczynski_wiita_gravity.h`, `event.h`, `tidal.h`, `horizon.h`, `collision.h`, `event_step.h` (src/core); tests: tests/core/test_black_hole.cpp, test_collisions.cpp, test_events.cpp.

### Constants and characteristic radii
- c = 299792458 m/s, exact. In space units c = 299792458 * 86400 / 149597870700 = 173.1446326742403 AU/day (derived from defined constants only).
- r_s = 2 G M / c^2 with G = k^2 AU^3 Msun^-1 day^-2: 1.97412e-8 AU = 2953.25 m per solar mass (test: equals 2 GM_sun / c^2 in SI to 1e-14).
- Photon sphere 1.5 r_s, ISCO 3 r_s (same in Schwarzschild and Paczynski-Wiita), marginally bound orbit 2 r_s.
- Spin (`BlackHole::spin`, a* in [0, 1]) is stored only. Kerr geometry is not implemented (ADR-003); no 3b quantity depends on spin, and absorption leaves it unchanged.

### Paczynski-Wiita force model (`PaczynskiWiitaGravity : IForceModel`, ADR-004)
- Pair potential U_ij = -G m_i m_j / (r - R_ij), R_ij = sum of the Schwarzschild radii of the black holes in the pair (0 for a pair without black hole, which is then Newtonian). One black hole: Phi = -GM/(r - r_s). Two black holes: the PW potential of the total mass. Central, equal-and-opposite pair forces: momentum and angular momentum are conserved by the force law (measured sum(m a)/sum(m |a|) = 8.1e-17).
- Circular orbits: v^2 = G M r / (r - r_s)^2, T = 2 pi r (r - r_s) / sqrt(G M r); stable only for r > 3 r_s.
- Far field: a_PW / a_N - 1 = (r/(r - r_s))^2 - 1 = 2 r_s/r + O((r_s/r)^2).
- Softening policy: Plummer eps applies to Newtonian pairs only, through the same ordinary path, power-of-two scaled path and path selection as `NewtonianGravity` (bit-identical accelerations and potential for systems without black holes, for every separation). Black-hole pairs are never softened.
- Horizon floor (defined behaviour for r <= r_s): with x = r - R_ij and h = 1e-3 R_ij, the force magnitude is held at G m_i m_j / h^2 for x < h and the potential continues linearly, U = -G m_i m_j (2h - x)/h^2, so force and potential stay consistent and finite everywhere; at r = 0 the pair force is zero. No NaN for any finite separation.
- Range: black-hole pairs never form r^2 or (r - R_ij)^2 r. The separation is scaled by a power of two for r and the unit direction u, and with gap = max(r - R_ij, h) = g 2^k, g in [0.5, 1), the field is ((G m / g^2) u) 2^(-2k), so forces and potentials are accurate to rounding for every finite separation and black-hole masses in [1e-30, 1e30] Msun (a field below the smallest normal double is rounded as a subnormal). Test: 1e30 Msun hole and a star at 1e110 AU give 2.959122e-194 AU/day^2 (analytic, relative 1e-14, where the former gap^2 r formula overflowed to a zero force); a central difference of U at r = 2^400 AU matches the force to 1e-8. Newtonian pairs share the range of `NewtonianGravity` (star and probe at 1e110 and 1e140 AU: analytic to 1e-14 and bit-identical to `NewtonianGravity`).

### Encounter phase and events
`EventStepper::step` drives the integrator through `IIntegrator::step_observed`, which reports every accepted integration segment with the positions at both ends: for a fixed-step integrator the whole step, for IAS15 every accepted internal step (reported before anything is committed). The first segment in which an encounter would occur (`encounter_pending`) ends the integrator call there, `resolve_encounters` runs on that segment (its start positions, start time and length), and integration resumes from the resulting state for the rest of the requested dt. The swept chord is therefore never longer than one accepted IAS15 step, which its error control keeps short near a pericentre: a pericentre passage through a star inside one long output interval is detected, and the result does not depend on how the run is split into output intervals beyond the internal steps those split points shorten (test: a = 1.05 AU, e = 19/21 orbit entering a 0.2 AU star or tidal radius, one full period requested as 1, 7 or 64 intervals). If an integrator call fails the state before `step` is restored and no event is appended. `resolve_encounters` runs the phases in this fixed order:
1. Tidal disruption (detection only): bodies with radius R > 0 and mass m > 0 (not black holes, not probes) whose swept path enters r_t = R (M/m)^(1/3) while being outside r_t at the start of the step. No event when r_t <= r_s (swallowed whole). The body is not modified (no fragmentation physics).
2. Horizon crossing: a body whose centre comes within r_s of a black hole (r_s1 + r_s2 for two black holes) is absorbed. The black hole always survives against another kind; between black holes the heavier survives (lower id on a tie). Probes are absorbed too (their mass and momentum go to the hole). After each absorption the hole moves on the merged centre-of-mass path and is swept again with its new Schwarzschild radius over the rest of the step, so a body that reaches only the enlarged horizon is absorbed in the same step (test: two 1e6 Msun holes 1.5 r_s apart merge, a probe crossing only the merged 2 r_s horizon is absorbed at step fraction 1.75/1.9).
3. Collisions: bodies other than black holes and probes; contact when the swept separation comes within R_i + R_j > 0 (points never collide). Broad phase (spec 3.5): loose octree over the swept boxes; a node of size S keeps boxes larger than S/4 and passes the others to the octant of their centre (at most 8 movable boxes per leaf, depth at most 64 and cells above 2^-40 of the coordinate magnitude); loose bounds (cell widened by S/2) contain every box of a subtree, so each box query descends only into overlapping loose bounds. Stable partitions, fixed octant order and a final sort make it deterministic; it equals the exhaustive box test (600 uniform spheres, 504 mixed-scale spheres with coincident and touching boxes). One distant or large box no longer sets the cell size of the others: a regular grid of 512 / 4096 / 32768 bodies of radius 1e-9 AU plus one 1e6 AU sweeper costs 11232 / 110048 / 901361 box tests (about 27 per body), where the former global-cell grid needed N^2/2. Narrow phase: swept sphere-sphere CCD, earliest root of |p0 + tau (p1 - p0)| = R. The miss distance is |p0 x p1| / |p1 - p0|, with each cross-product component formed as a difference of products with fused multiply-add (a few ulps even when the products cancel), and the entry distance is (|p0|^2 - R^2) / (projection + half chord), so neither a near-tangent miss nor a long straight pass loses the perpendicular distance to cancellation (tests: impact parameters 2 and 1.00000001 against R = 1 over 2e9 and 2e8 AU paths miss, impact parameters 0.5 and 0.99999999 hit at the analytic fraction to 1e-15, all under power-of-two scalings 2^-900 to 2^900). Contacts are resolved earliest first (ties by lower id, higher id); after each merge the survivor moves on the merged centre-of-mass path with its new mass and radius, its pending contacts are dropped and it is swept again against all participants over the rest of the step, until no contact remains. Chain collisions and multi-body overlaps therefore merge within one step (test: B hits A at 1.8/22 of the step, the merged A then hits C; three overlapping bodies end as one).
- Merge (collisions and absorption): survivor keeps id, name and kind; m = m1 + m2, R_cm = r1 + f (r2 - r1), V = v1 + f (v2 - v1), f = m2/m. Kind rule: the heavier body wins kind (lower id on equal masses); for horizon crossings the black hole wins. Collision radius: R = (R1^3 + R2^3)^(1/3); a black hole keeps its radius. The relative angular momentum mu (r_rel x v_rel) cannot be carried by a point mass: it is reported as `internal_angular_momentum` in the event, so orbital L after + internal = L before.
- Events (`CollisionEvent`, `DisruptionEvent`, `HorizonCrossingEvent`, `std::variant` `Event`) are appended to an `EventLog` in phase order and, within a phase, in ascending step fraction then id; the step loop calls `drain()`. No callbacks, no threads. Event time = t_start + step_fraction * dt.
- BodyStore gained a radius column (`radius`, `set_radius`, `radii`; every body starts as a point, radius 0) and `set_mass`. The radius decides collisions and tidal disruptions, so it is part of `state_hash` (the golden hashes were re-pinned, see the determinism contract). Tested: states that differ only in one radius, one radius ulp, the sign of a zero radius or the assignment of two radii hash differently; copy, assignment, move, removal and a volume-combining merge keep the expected radius-sensitive hash.

### Approximations (3b)
| ID | Approximation | Where | Validity envelope | Outside the envelope |
|---|---|---|---|---|
| B1 | Paczynski-Wiita potential Phi = -GM/(r - r_s) | `PaczynskiWiitaGravity` | Exact ISCO (3 r_s) and marginally bound orbit (2 r_s); qualitative strong-field orbits | No correct periapsis precession, frame dragging (spin ignored) or gravitational waves |
| B2 | Horizon floor: PW force capped at its value at r = (1 + 1e-3) R_ij | `PaczynskiWiitaGravity` | Exact PW for r >= (1 + 1e-3) R_ij | Inside the shell the trajectory is meaningless; the body is absorbed at the end of the step |
| B3 | Tidal radius r_t = R (M/m)^(1/3), prefactor 1 | `tidal_radius`, tidal phase | Disruption criterion to within a factor of about 2 in radius | Structure and rotation change the prefactor (1 to 2.4); debris not modelled |
| B4 | Linear relative motion within an integration segment (swept tests) | CCD, horizon and tidal phases | Segment (fixed step, or one accepted IAS15 internal step) a small fraction of the encounter time; with IAS15 the entry into a 0.2 AU star at a 0.1 AU pericentre is timed to 1e-3 d | Grazing contacts on strongly curved paths can be missed or mistimed by up to one segment; fixed-step integrators still need dt well below the encounter time |
| B5 | Absorption when the centre crosses r_s | Horizon phase | Bookkeeping of mass and momentum | Finite size, accretion luminosity and radiated energy are not modelled |
| B6 | Perfectly inelastic point merge (equal-density radius) | `merge_bodies` | Low-velocity accretion | Fragmentation and ejecta of high-speed impacts are absent; relative KE is dissipated, relative L reported only |

Fixed-step integrators cannot follow the final plunge, where the PW force diverges: with a constant dt a plunging body can be thrown out again by the discretisation (measured: from 2.9 r_s with dt = T/2000 the probe reached 1.05 r_s and was ejected). The ISCO test therefore limits the step to 1/50 of the time to cover the gap to the horizon. IAS15 shortens its internal steps by itself, and `EventStepper` checks encounters on each accepted internal step, so with IAS15 the horizon crossing is found inside a long output interval too.

### Measured acceptance values (MSVC 19.44 x64 Release, /fp:precise)
| Test | Measured |
|---|---|
| r_s(1 Msun) | 1.974e-8 AU = 2953.25 m |
| PW vs Newton, relative force difference at r = 1e2 / 1e4 / 1e6 r_s | 2.030e-2 / 2.000e-4 / 2.000e-6 (= 2.03 / 2.000 / 2.000 r_s/r) |
| PW without black holes vs NewtonianGravity | bit-identical accelerations and potential |
| Circular PW orbit, r = 10 r_s (M = 4e6 Msun), Yoshida4 4000 steps | period relative error 9.0e-12 vs analytic 0.1153 d (Newtonian period 0.1282 d, ratio 0.9 = (r - r_s)/r); max radius error 8.6e-12 |
| ISCO stable, perturbation 1e-3 (tangential and radial), 30 orbits | r0 = 3.5 / 4 / 6 / 10 r_s: r_min/r0 = 0.979 / 0.988 / 0.993 / 0.995, r_max/r0 = 1.000, none absorbed |
| ISCO unstable, same perturbation | r0 = 2.5 r_s plunges after 1.10 orbits, r0 = 2.9 r_s after 1.57 orbits (HorizonCrossing emitted) |
| No-NaN at r/r_s = 0, 1e-9, 0.5, 1, 1.0005, 1.001, 1.01, 2 | all accelerations and potentials finite; r = 0 gives zero force; cap 7.59e17 AU/day^2 at 1 Msun |
| Merge (moon into planet) | mass 0, momentum 1.3e-16, centre of mass 2.1e-16, L (orbital + internal) 2.1e-16 (relative) |
| Horizon absorption (0.8 Msun star into 1e6 Msun hole) | mass 0, momentum 1.2e-16, centre of mass 1.9e-16, L (orbital + internal) 2.0e-16 (orbital alone 1.7e-5) |
| Radial plunge | angular momentum change 0, momentum change 0 |
| CCD pass-through (bodies swap sides within one step) | detected at step fraction 0.495; momentum change over step + merge 6.7e-17 |
| Event determinism (1e6 Msun hole, 300 bodies, 400 steps, run twice) | 36 collisions, 171 horizon crossings, 183 disruptions (with re-swept survivors), 94 bodies left; identical event lists and state hash; total mass change 5.8e-16 |
| Pericentre plunge with IAS15 (a = 1.05 AU, e = 19/21, 0.2 AU star or tidal radius), one period as 1 / 7 / 64 output intervals | one event each; entry time minus Kepler -9.1e-4 / -3.1e-4 / -8.4e-4 d (collision), -9.0e-4 / -3.1e-4 / -8.3e-4 d (tidal) |
| Loose-octree broad phase, regular grid + 1e6 AU sweeper, 513 / 4097 / 32769 bodies | 11232 / 110048 / 901361 box tests, no pairs |

## Black hole rendering (Phase 5)

Sources: `orrery/core/schwarzschild_geodesic.h` (double-precision reference,
graphics-free), `src/render/shaders/black_hole_trace.comp.glsl` (the GPU ray
marcher), `orrery/render/blackbody_spectrum.h`; rendering design in
`rendering.md` section 17; decisions ADR-003 (Schwarzschild, Kerr deferred)
and ADR-017 (half resolution, step budget); tests:
tests/core/test_schwarzschild_geodesic.cpp, tests/render/test_black_hole.cpp,
tests/render/test_black_hole_render.cpp.

### Null geodesics
- Geometrized units G = c = 1 with lengths in M = G M / c^2 (r_s = 2 M; for
  4.3e6 Msun, M = 6.35e9 m). The image does not depend on the mass in these
  units.
- A photon moves in the plane of its position and direction; with u = 1/r
  and the angle phi in that plane, the Schwarzschild orbit equation is
  u'' = -u + 3 u^2, first integral (u')^2 = 1/b^2 - u^2 (1 - 2u) with the
  impact parameter b = L/E.
- Static observer at r0, ray at angle alpha from the outward radial
  direction: b = r0 sin(alpha) / sqrt(1 - 2/r0), u'(0) = -u0 sqrt(1 - 2 u0)
  cot(alpha).
- Integration: classical RK4 in phi, step min(4 pi / N, 0.25 max(u, 0.01)
  / |u'|) with the step budget N <= 512. Capture at u = 1/2, escape at u = 0
  (escape angle phi + atan2(u, -u') from the end of the last step), cubic
  Hermite dense output for the disk-plane crossings, which lie at
  phi = atan2(e2.N, e1.N) + pi/2 + k pi.
- Critical impact parameter 3 sqrt(3) M (photon sphere r = 3 M): from
  outside the photon sphere an inward ray is captured exactly when b < b_c.
  Shadow angular radius for a static observer:
  sin(psi) = 3 sqrt(3) sqrt(1 - 2/r0) / r0.
- Deflection of a ray from infinity: 4/b + 15 pi/(4 b^2) + 128/(3 b^3) + ...
  (weak field); -ln(b/b_c - 1) + ln(216 (7 - 4 sqrt 3)) - pi near b_c
  (strong-deflection limit, Bozza 2002).

### Thin disk
- Gas on prograde circular geodesics about the disk axis (world +Z),
  Omega = r^(-3/2), from the ISCO 6 M to 30 M.
- Temperature: T(r) = T_peak f(r/r_in) / f(49/36),
  f(x) = x^(-3/4) (1 - x^(-1/2))^(1/4) (Shakura and Sunyaev 1973 with a
  zero-torque inner edge, the Newtonian limit of Page and Thorne 1974);
  T ~ r^(-3/4) for r >> r_in, maximum at r = 49/36 r_in, zero at r_in.
- Frequency ratio for an emitter at r and a static observer at r0:
  g = sqrt(1 - 3/r) / (1 - Omega lambda) / sqrt(1 - 2/r0), with
  lambda = L_z / E of the photon (u^t = 1/sqrt(1 - 3/r) for a circular
  geodesic; the observer's factor is the blueshift of falling to r0).
  Approaching gas has lambda > 0 and g > 1. Example: light emitted along the
  orbit at the ISCO (v = c/2 for a local static observer) reaches infinity
  with g = sqrt(2) forwards and sqrt(2)/3 backwards.
- Beaming: I_nu / nu^3 is invariant along a ray, so I_obs(nu) =
  g^3 I_em(nu/g). For a black body this is exactly a black body at g T, whose
  bolometric intensity is g^4 times the emitted one (Stefan-Boltzmann). The
  renderer evaluates the visible-band radiance of B(g T) (Planck spectrum
  times the CIE 1931 colour matching functions, converted to linear sRGB), so
  neither power is applied as a separate factor; in the visible band the
  boost is steeper than g^4 on the cool (Wien) side, which is why the
  receding side dims and reddens strongly.

### Measured (MSVC 19.44 x64 Release; GPU: AMD Radeon RX 9060 XT)
| Quantity | Reference | Measured |
|---|---|---|
| Deflection, b = 1000 / 2000 / 5000 M | 4/b + 15 pi/(4 b^2), remainder 128/(3 b^3) | within the third-order term plus 2e-8 rad (RK4 at 4 pi / 512) |
| Strong deflection at b = b_c (1 + 1e-2), b_c (1 + 1e-3) | Bozza 2002 | within 0.03 and 0.005 rad |
| Capture, camera at 40 M | b < b_c inwards | b = 0.5 .. 0.999 b_c captured, 1.001 .. 6 b_c escape; outward rays escape |
| g against the emitter four-velocity | p.u contraction | 1e-12 relative, r = 6 .. 60 M, lambda = -12 .. 7 |
| Thin-disk profile | analytic | peak at 49/36 r_in, T(2r)/T(r) = 2^(-3/4) to 1e-3 far out |
| GPU (float) against the CPU reference (double), 3724 rays | same algorithm | escape direction median 6.1e-7 rad (worst 3.5e-5), b 4.9e-6, disk radius 5.0e-5, g 8.6e-6 |
| Shadow radius on screen, 12 / 30 / 80 M | tan(psi) x pixel scale | 122.8 / 48.5 / 18.1 px for 122.9 / 48.5 / 18.4 px |
| Einstein ring of a source behind the hole, 60 M | CPU geodesic root | 143.52 px for 143.52 px |

### Approximations (black hole rendering)
Every item is marked `// APPROXIMATION:` in the code.

| ID | Approximation | Where | Validity envelope | Outside the envelope |
|---|---|---|---|---|
| R1 | Non-rotating hole: Schwarzschild metric, spin stored but ignored (ADR-003, A13) | `lensing_disk`, trace shader | a* = 0 | Kerr holes have an asymmetric, smaller shadow, frame dragging and a closer ISCO |
| R2 | Thin disk: zero thickness, plane normal to world +Z, ISCO to 30 M, circular geodesics, Newtonian-limit temperature profile, optically thick black body without limb darkening or returning radiation | `lensing_disk` (black_hole.cpp), trace shader | Shape, inner edge, colour gradient and Doppler/redshift pattern of a thin, optically thick disk | Thick or radiatively inefficient flows (such as Sgr A*'s actual flow), relativistic Novikov-Thorne factors (tens of percent near the ISCO), absolute luminosity (display-scaled) |
| R3 | Disk opacity and texture: opaque to 0.82 of the outer radius, thinning to the edge; sheared streak pattern (factor 0.4 .. 1.8) on the emission | trace shader | Display only; temperature, Doppler and redshift are not affected | No physical surface structure is implied |
| R4 | Static camera; a ray that exhausts the step budget counts as captured | trace shader | Camera at rest; at 512 steps only rays within about 1e-4 of b_c are cut short | A moving observer would see aberration and an extra Doppler shift |
| R5 | Escape angle from the straight-line solution near u = 0 | `infinity_offset` (core), trace shader | Error second order in the last step's overshoot, below 1e-12 rad | - |
| R6 | One lens; bodies drawn unlensed, hidden by the shadow and disk through depth | `select_lensed_black_hole` | A single dominant hole; the sky at infinity | Bodies near the hole appear at unlensed positions; a second hole does not lens |
| R7 | Edge-aware upsampling of a half-resolution trace; occlusion depth from the nearest sample | composite shader (ADR-017) | Exact at the critical curve (analytic); smooth ray map over two pixels | Near the critical curve the sky is filtered, not resolved; bodies within the disk thickness of the hole may be ordered wrongly by one sample |
| R8 | Lensed stars keep the angular brightness of the reference screen (1080 rows, 40 degrees); in the cube map each star is a Gaussian in the angle from it, drawn on every face within 16 texels of it, so it keeps its flux across edges and corners (within 10 percent, tested) | `sky_cube_stars.vert.glsl` | Window sizes about 1440 x 810 to 1920 x 1080 at 40 degrees (within 10 percent) | Small offscreen images show the lensed stars dimmer than the sprite sky |
| R9 | CIE 1931 colour matching functions from the Wyman-Sloan-Shirley fit | `cie_colour_matching` | A few percent of the CIE table over 380 .. 780 nm, about 1e-3 in black-body xy | - |
| R10 | Negative linear sRGB components clipped (gamut) | `blackbody_radiance_srgb` | Exact above about 1900 K | Black bodies below 1900 K lose a few percent of their luminance in blue |
| R11 | Radiance bound: the light of a trace sample and of a composited pixel is scaled down, hue kept, to at most 1e4 (scene-linear) per component; the look-up of B(g T) stops at 200000 K | trace and composite shaders | Display only: the bound is 4500 times the disk's peak (2.2) and maps to white at any exposure (1/64 .. 64); it is reached only for g >= 2 with the coolest, brightest disk (2000 K, brightness 4), that is for a static camera within a few M of the horizon (g = 13.4 at r0 = 2.01 M) | Without it, B(g T) / B(T) exceeds 10^5 near the horizon and the half-float HDR target (max 65504) stores infinity, NaN after tone mapping. Radiance ratios above the bound are not shown (they would be white anyway) |

## 3a: Barnes-Hut tree gravity, IAS15 and embedded scenarios (orrery_core, Phase 3a)

Headers: `octree.h` (`Octree`), `barnes_hut.h` (`BarnesHutGravity : IForceModel`), `ias15.h` (`IAS15 : IIntegrator`). Scenarios: `assets/scenarios/solar_system_j2000.json`, `figure_eight.json`, `sgr_a_s2.json`. Tests: `tests/core/test_barnes_hut.cpp`, `test_ias15.cpp`, `test_scenarios_phase3a.cpp`. Third-party: oneTBB 2023.2.0 (vcpkg port `tbb`, Apache-2.0), linked PRIVATE to `orrery_core` and allow-listed in `tools/check_deps.py` (`THIRD_PARTY_DEPS`); it has no graphics dependency and the graphics-free checks still inspect it.

### Barnes-Hut (Barnes and Hut 1986)

- Tree build (`Octree::build`): bounding cube of the sources (bodies with positive mass that exert gravity) with side the smallest power of two above the largest extent; 21-bit quantisation per axis; 63-bit Morton key (x in bit 0 of each triple); sort by (key, input position), which is unique, so the order does not depend on the sort algorithm or thread count (oneTBB `parallel_sort`, keys computed with `parallel_for`). Nodes are created top-down in pre-order, splitting a node with more than `leaf_capacity` (default 8) sources into its non-empty octants (maximum depth 21). Each node stores G M, the centre of mass c, the G-scaled second moments Q_ij = sum G m (x - c)_i (x - c)_j (children combined with the parallel-axis theorem), its Morton cell side, the offset |c - cell centre|, and the index of the first node after its subtree (stackless walk).
- Opening criterion (Barnes 1994): a node is used as a multipole when d > side / theta + offset, d = |x - c|. With theta <= 1 a node that contains the receiver is never accepted (the receiver is within offset + side sqrt(3)/2 of c), so the receiver's own mass enters only through direct leaf sums, where it is skipped. theta = 0 accepts nothing (direct summation in tree order). theta must lie in [0, 1].
- Softened multipole field, the exact second-order Taylor expansion of the Plummer kernel f = 1/D, D^2 = |r|^2 + eps^2, r = x - c (the dipole vanishes about c). With grad f = -r D^-3, d_i d_j f = 3 r_i r_j D^-5 - delta_ij D^-3 and d_i d_j d_k f = -15 r_i r_j r_k D^-7 + 3 (delta_ij r_k + delta_ik r_j + delta_jk r_i) D^-5:
  a = M grad f + 1/2 Q : grad grad grad f = -M r / D^3 + (3 Q r + 1.5 tr(Q) r) / D^5 - 7.5 (r.Q.r) r / D^7,
  phi = -M / D - 1.5 (r.Q.r) / D^5 + 0.5 tr(Q) / D^3 (M and Q already scaled by G). Leaves that are not accepted use the Plummer pair kernel of `NewtonianGravity`, G m (r_j - r_i) / (|r_j - r_i|^2 + eps^2)^(3/2).
- Potential energy: U = 1/2 sum over sources of m_i phi_i, with phi_i from the same walk, reduced sequentially in body order. It is consistent with the forces only to the accuracy of the expansion.
- Probes and zero-mass bodies are receivers only (not in the tree): adding probes leaves every other acceleration bit-identical (tested).
- Determinism: every receiver's acceleration comes from its own walk in fixed pre-order; oneTBB `parallel_for` only distributes receivers, and no floating-point value is combined across receivers or threads. Results are bit-identical for any thread count (`BarnesHutOptions::max_threads` limits a `task_arena`). Tested with 1, 2, 3, 4, 8, 12 threads and the default on N = 20000 (acceleration hash 0x2823f5aa0313ae9b); a 20-step leapfrog run on N = 3000 gives the same state hash with 1 thread and with all threads (0x350b54d4ca44954a in the radius-including hash format). The 10000-step golden run (determinism contract) pins 0x904687bcd8a69556 for 1, 2, 4, all and default threads. Only IEEE +, -, *, /, sqrt, floor, frexp and ldexp are used.
- Non-finite source positions or an overflowing bounding box give NaN accelerations, which the integrators reject as `non_finite_state`. Unlike `NewtonianGravity`, the tree code has no scaled path for extreme separations (outside roughly 1e-150 to 1e150 AU).
- APPROXIMATION (A15, Barnes-Hut multipole truncation): groups that satisfy the opening criterion are replaced by monopole + quadrupole; octupole and higher terms are dropped (relative error of order (side/d)^3). Linear momentum is not conserved exactly because the interaction is not pairwise symmetric. Valid for large-N, collisionless systems; the direct sum remains the reference for few-body and planetary dynamics.

Measured (MSVC 19.44 x64 Release, /fp:precise, AMD Ryzen 5 7600, 6 cores / 12 threads):

| Criterion | Setup | Measured | Bound |
|---|---|---|---|
| Acceleration error vs direct sum | seeded Plummer sphere (scale 1 AU, truncated at 10 AU, mt19937_64 seed 20260101), N = 2000, masses (0.5 to 1.5)/N Msun, eps = 1e-3 AU, theta = 0.5, leaf 8 | median 4.70e-4, p99 1.97e-3, max 1.01e-2 | median < 1e-3 (Section 8), max < 2e-2 |
| Same, other theta | theta = 0.3 / 0.7 / 1.0 | median 7.4e-5 / 1.46e-3 / 4.84e-3; max 1.1e-3 / 1.7e-2 / 1.1e-1 | observation |
| theta = 0 vs `NewtonianGravity` | same cluster | max relative difference 2.4e-14 (summation order only) | < 1e-12 |
| Potential energy | theta = 0.5 | relative error 2.2e-6 | < 1e-3 |
| Momentum (observation) | N = 2000 starting at rest, leapfrog 50 x 0.5 d, eps = 1e-2 AU | abs(P) / sum m abs(v) = 3.0e-5 | < 1e-3 |
| Force timing N = 10k (test tagged `[.benchmark]`, not in ctest) | theta = 0.5, eps = 1e-3 AU, Plummer | 11.7 ms (12 threads, leaf 8 or 16), 72 ms single thread | target < 8 ms on 8 cores: missed |
| Force timing N = 100k | same | 191 to 205 ms (12 threads), 1.27 s single thread | target < 120 ms on 8 cores: missed |

Timing deviation: the kernel is scalar double arithmetic with one correctly rounded sqrt and one division per interaction (no fast-math, no approximate reciprocal square roots, no architecture-specific SIMD flags), and the quadrupole term costs about twice a monopole. The parallel speed-up is 6.2 to 6.7 on 6 physical cores, so the work is compute-bound rather than build-bound; scaled to 8 physical cores the estimate is about 9 ms and 150 ms. Tried and rejected: a per-leaf group walk (one interaction list per leaf) was slower (19 to 26 ms, 300 to 450 ms) because the conservative group criterion adds interactions and MSVC does not vectorise the receiver loop under /fp:precise; leaf sizes 8 to 32 change the time by less than 15 percent; a compact 48-byte walk array gave no measurable gain (kept, identical results). Options to reach the target: SIMD intrinsics over receivers (bit-identical per lane, each lane performing the same IEEE operations), a monopole-only mode with a smaller theta, or reusing the tree between substeps. Run with `orrery_test_core "[benchmark]"`.

### IAS15 (Rein and Spiegel 2015, after Everhart 1985)

- Within an internal step h the acceleration is a(t0 + s h) = a0 + b0 s + ... + b6 s^7, fitted at the 8 Gauss-Radau spacings h_0 = 0 and h_1..h_7 = roots of (P7(x) + P8(x)) / (1 + x), x = 2s - 1: 0.0562625605369221, 0.180240691736892, 0.352624717113170, 0.547153626330555, 0.734210177215411, 0.885320946839096, 0.977520613561288. The spacings and the g <-> b conversion matrices (b_k = sum_j c_kj g_j with c_kj the coefficient of s^k in prod_{m<=j} (s - h_m), and d = c^-1) were computed with 60-digit arithmetic and are stored as 26-digit literals.
- Predictor-corrector: positions predicted at each spacing, x = x0 + s h (v0 + s h (a0/2 + s (b0/6 + s (b1/12 + ... + s b6/72)))); accelerations evaluated there; divided differences g_k updated and converted to b. Iteration stops when max abs(delta b6) / max abs(a) < 1e-16, when that ratio stops decreasing (after two iterations), or after 12 iterations.
- Step control (paper, section 3.4): err = max abs(b6) / max abs(a) over all coordinates, h' = h (epsilon / err)^(1/7), epsilon = 1e-9 by default; the step is redone with h' if h' < 0.25 h, and h' is limited to 4 h. The seventh root is a fixed Newton iteration on frexp/ldexp-reduced arguments (no `pow`), so the step sequence is identical on every C runtime.
- Next-step prediction: the converged polynomial is re-expanded about the end of the step for q = h_next / h, e_j = q^(j+1) sum_{k>=j} C(k+1, j+1) b_k, and corrected by the difference between the last converged b and its own prediction; no prediction when q > 20.
- Positions and velocities are advanced with compensated (Kahan) summation.
- Adaptive entry through the fixed-step interface: `step(state, dt, forces)` integrates exactly to t + dt with as many internal steps as needed (when between one and two proposed steps remain, the rest is split into two equal halves; the last step is clipped to the target). The controller step and the b prediction carry over between calls when a call starts from the state this integrator committed (bit-identical positions and velocities); a foreign state, a change of body count or of direction restarts them. A failure leaves the state unchanged. New error `IntegrationError::step_size_underflow`: a step the controller proposes cannot advance the time representation.
- APPROXIMATION (IAS15 step polynomial): the acceleration over a step is a degree-7 polynomial (local error O(h^16)); the step control keeps it near machine precision. Valid while the forces are smooth over a step; impacts and switching force models must be handled at step boundaries by the caller.

| Criterion | Setup | Measured | Bound |
|---|---|---|---|
| Analytic Kepler | a = 1.3 AU, e = 0.5, 10 orbits, output once per orbit | max position error 8.1e-14 AU; 1200 steps, 2 rejected, 25653 force evaluations | 1e-10 AU |
| e = 0.99 vs Leapfrog at equal cost | Sun + 1e-3 Msun, a = 1 AU, start at apocentre, 10 orbits, 68825 force evaluations each | IAS15: max abs(dE/E) 3.95e-14, final 5.4e-15, position error 5.9e-14 AU (3911 steps, 10 rejected). Leapfrog dt = 0.106 d: abs(dE/E) 8.3 (the pericentre passage is unresolved and the orbit is destroyed), position error 162 AU | IAS15 < 1e-12 and 1000 times below Leapfrog |
| Sun + 8 planets (Standish elements) | 100 years, 1-year outputs | max abs(dE/E) 2.6e-15, momentum drift 1.3e-16; 31860 steps (mean 1.15 d) | 1e-12 |
| Time reversal | e = 0.6, 400 d forward then back | 7.7e-15 AU | 1e-12 AU |
| Determinism | golden state, 200 x 10 d, two runs; warm restart vs fresh integrator | identical hashes | equal |
| Golden hash | golden state, eps 1e-4 AU, 10000 x 10 d (92483 accepted steps, 1 rejected) | 0xf9cc3e89244d1214 | pinned (determinism contract) |

### Embedded scenarios

- `solar_system_j2000.json`: Sun, Mercury, Venus, Earth, Moon and the Mars, Jupiter, Saturn, Uranus and Neptune system barycentres at 2000-01-01 12:00 TDB (JD 2451545.0). States copied verbatim from the JPL Horizons API (https://ssd.jpl.nasa.gov/api/horizons.api, retrieved 2026-11-11, DE441) with EPHEM_TYPE=VECTORS, CENTER=500@0 (solar-system barycentre), REF_SYSTEM=ICRF, REF_PLANE=ECLIPTIC (ecliptic and mean equinox of J2000.0), VEC_TABLE=2, VEC_CORR=NONE, OUT_UNITS=AU-D. Masses are GM_body / GM_Sun with every GM from the Horizons physical-data headers (Sun 132712440041.93938 km^3/s^2); system masses add the satellites 501-504, 601-608, 701-705 and 801. The full provenance is in the file's metadata.
- APPROXIMATION (J2000 scenario): ten point masses; asteroids, Pluto, the other satellites, relativity and oblateness are omitted, and masses use the Gauss-constant G (k^2 and the DE441 GM_Sun differ by 7e-12 relative, section 1.1). Valid for planetary positions to about 1e-6 AU over a year. Measured against Horizons one Julian year later (JD 2451910.25, IAS15): Sun 1.9e-9 AU, Mercury 3.9e-7 AU (58 km), Earth 4.1e-7 AU (61 km), Moon 4.7e-7 AU (71 km), Jupiter barycentre 4.1e-9 AU; energy drift 2.6e-15, momentum drift 5.6e-17 (bounds 1e-5 AU per body and 1e-13 for the drifts).
- `figure_eight.json`: the Chenciner and Montgomery (2000) choreography with C. Simo's initial conditions in G = m = 1 units: x1 = -x2 = (0.97000436, -0.24308753), x3 = 0, v3 = (-0.93240737, -0.86473146), v1 = v2 = -v3/2, T = 6.32591398. Scaling to AU / Msun / day: masses 1 Msun, length unit 1 AU, time unit 1/k = 58.132440867 d, so positions are unchanged, velocities are multiplied by k = 0.01720209895 and T = 367.74082 d. v3 = -2 v1 exactly in binary, so the total momentum is exactly zero.
  Measured over 100 periods (return time from a parabola through |X(t) - X(0)|^2 sampled at T/2000 around each k T): IAS15 max abs(dE/E) 2.0e-15, momentum drift 7.1e-16, angular momentum drift 5.1e-16; first period 367.740822 d (6.325914 scaled), relative difference to the literature T 5.1e-9 (limited by the 8 to 9 digits of the published initial conditions); the mean period over 100 returns differs from the first by 1.3e-10; 567159 force evaluations. Yoshida4 with dt = T/2000 (800008 evaluations): max abs(dE/E) 1.1e-13, period drift 7.7e-10. Bounds: energy < 1e-8 (acceptance), period drift < 1e-8 (IAS15) and < 1e-6 (Yoshida4), literature period < 1e-7.
- `sgr_a_s2.json`: Sgr A* (black-hole kind, spin 0, Newtonian gravity) and S2 with the GRAVITY Collaboration 2020 best fit (A&A 636, L5, Table E.1): M = 4.261e6 Msun, R0 = 8246.7 pc, a = 125.058 mas, e = 0.884649, i = 134.567 deg, omega = 66.263 deg, Omega = 228.171 deg, P = 16.0455 yr; a = 0.125058 x 8246.7 = 1031.3158 AU. Starts at apocentre. For comparison, Gillessen et al. 2017 (ApJ 837, 30): a = 0.1255 arcsec, e = 0.884, P = 16.00 yr.
- APPROXIMATION (S2 scenario): Newtonian point masses; S2 is a zero-mass test star (its mass is of order 1e-5 of the hole's), and the Schwarzschild precession and Roemer delay measured by GRAVITY are not modelled. Valid for the orbital period and shape, not for the precession or the redshift.
  Measured: period from successive pericentre passages (sign change of r.v, bisected) 16.045008 Julian years for both orbits; Kepler 2 pi sqrt(a^3 / (k^2 M)) = 16.045008 yr (relative difference 9e-15); GRAVITY fit 16.0455 yr. Acceptance 16.0 +- 0.3 yr.

## Geometry pipeline (orrery_geom, Phase 6a)

Sources: `orrery/geom/primitives.h`, `naca.h`, `mesh_import.h`,
`mesh_repair.h`, `winding_number.h`, `voxel_grid.h`, `panel_mesh.h`,
`reference_quantities.h`; design and formulas in `geometry-import.md`;
decisions ADR-010 (untrusted models) and ADR-018 (kernels without libigl,
validation gate around assimp); tests in `tests/geom`.

### Primitives and analytic references
- Solids of revolution (sphere, cylinder, cone, capsule): exact volume
  (integral of pi r^2 dz) and area (integral of 2 pi r ds) from the
  antiderivatives on each line and circular arc of the meridian profile. The
  tessellation's own volume and area are known in closed form (regular N-gon
  frustums), so every mesh is checked against an exact value; the true shape
  is reached at second order in the resolution.
- Ellipsoid area 4 pi a b c R_G(a^-2, b^-2, c^-2) with Carlson's symmetric
  integrals (Carlson 1995, duplication theorem).
- NACA 4-digit section area: integral of 2 y_t sqrt(1 + y_c'^2) dx by
  Gauss-Legendre quadrature after x = u^2 (thickness applied normal to the
  camber line; Abbott and von Doenhoff, *Theory of Wing Sections*, 1959).

### Measured (MSVC 19.44 x64 Release, /fp:precise)

| Criterion | Setup | Measured | Bound |
|---|---|---|---|
| Primitive volume and area against analytic | sphere, cylinder, cone, box, cube, plate, wedges, Apollo and Orion capsules, ellipsoids (closed forms, spheroid formulas, 2D quadrature); meshes against their polyhedral closed forms at N = 16 .. 128 | worst relative error 6.1e-14 over 65 checks | 1e-9 (acceptance) |
| Capsule antiderivatives against independent quadrature | 20-point Gauss-Legendre, 64 panels per segment | below 1e-12 relative | 1e-12 |
| Carlson integrals | Carlson (1995) table values R_F, R_D, R_G | 14 published digits reproduced | 1e-12 relative |
| Tessellation convergence | N doubled from 16 to 128 | volume error ratio below 0.3 per doubling (second order) | 0.3 |
| NACA 0012 section area | quadrature against closed form | below 1e-13 relative; polygon of 800 stations 1e-5 | 1e-13 / 1e-5 |
| NACA 2412 section area | against Richardson-extrapolated polygons (4000, 8000 stations) | below 1e-9 relative | 1e-9 |
| Broken STL fixture | unit cube with hole, flipped triangle, duplicate, degenerate triangle, fin | reported exactly (14 triangles, 1 degenerate, 1 duplicate, 20 edges, 5 boundary, 1 non-manifold, 3 inconsistent, 1 hole, 1 open chain); repaired: watertight outward unit cube, volume error below 1e-15 | exact counts |
| Sphere voxelization | r = 1, 16128 triangles, 44^3 cells of 0.05, against the true sphere | 85184 of 85184 cells agree (accuracy 1.0); solid volume within 2 percent | 0.999 (acceptance) |
| Hierarchical winding number | beta = 2 against the direct sum, 2000 random points in [-2, 2]^3 | worst abs error 0.024, no inside/outside disagreement | 0.05 |
| Narrow-band SDF | r = 0.8 sphere, 96 x 48 facets, band 3 cells | worst error 8.6e-4 against abs(p) - r (facet sagitta 1.7e-3) | sagitta |
| Hostile input | NaN/inf coordinates, out-of-range indices, STL/PLY/FBX headers declaring billions of elements, 16 GB deflate claim, oversized files, references outside the model directory | all rejected before parsing or before post-processing | rejected |
| Import of 508320 triangles (glb) with watertight report | hidden benchmark `[benchmark]` | 0.96 s | 5 s (project criterion) |

### Approximations (geometry)
Every item is marked `// APPROXIMATION:` in the code. A1 (uniform density
for the centre of mass) applies to `reference_quantities`.

| ID | Approximation | Where | Validity envelope | Outside the envelope |
|---|---|---|---|---|
| G1 | Apollo- and Orion-like capsule presets: published diameter, nose radius, shoulder radius and afterbody angle; rounded apex replaced by a flat top | `CapsuleShape::apollo`, `orion` | Drag and heating studies of the heat shield and shoulder | Afterbody details, RCS fairings and the apex shape of the real vehicles are absent |
| G2 | NACA thickness with the closed-trailing-edge coefficient -0.1036 instead of -0.1015 | `Naca4::half_thickness` | Half thickness differs by 0.0105 t x^4 (0.13 percent of chord at t = 0.12) | Comparisons that depend on the 0.021 t open trailing edge of the original series |
| G3 | Unit of a file without one guessed from its bounding box (e > 500: mm, 50 < e <= 500: cm, else m) | `estimate_unit` | Bodies of 5 cm to 500 m; the caller confirms | Tiny or huge models, inches and feet are never guessed |
| G4 | Holes filled by a fan from the loop centroid | `repair_mesh` | Small, nearly planar holes | Large or curved holes get a flat or self-intersecting patch (changes volume and area) |
| G5 | Winding number far field truncated at first order (dipole), beta = 2 | `MeshTree::winding_number` | Error a few percent at worst, far below the 1/2 decision threshold | Values near 1/2 (open or self-overlapping meshes) may be misclassified where the exact sum is within a few percent of the threshold |
| G6 | Silhouette area by cell-centre raster coverage (1024 cells along the larger side) | `projected_area` | Error below perimeter x cell size (about 0.4 percent for a sphere) | Thin features below a cell are lost; use the bound for convex bodies |
