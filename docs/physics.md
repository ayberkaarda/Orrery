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

## 2. Space dynamics

- Newtonian gravity: a_i = sum_j G m_j (r_j - r_i) / (|r_j - r_i|^2 + eps^2)^(3/2), Plummer softening eps.
- Direct summation O(N^2) as reference; Barnes-Hut octree with opening angle theta for large N.
- Integrators (ADR-002): Leapfrog KDK (second order, symplectic), Yoshida 4th order (composition of leapfrog substeps), IAS15 (adaptive Gauss-Radau, Rein and Spiegel 2015), RK4 for tests only.
- Black hole (ADR-004): Paczynski-Wiita potential Phi = -GM/(r - r_s), r_s = 2GM/c^2; photon sphere 1.5 r_s, ISCO 3 r_s.
- Light bending (ADR-003): null geodesics in Schwarzschild metric, d^2u/dphi^2 + u = 3GM u^2/c^2 with u = 1/r.
- Diagnostics: osculating Kepler elements, Hill sphere, energy and angular momentum drift.

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
