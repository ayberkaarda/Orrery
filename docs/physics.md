# Orrery Physics Reference

Every formula and model implemented in `src/`, with its source, validity
envelope, approximations and the tests that check it. The
[Formula index](#formula-index) below has one row per implemented formula or
model and points to the detailed sections; sections 1 to 6 give the project
overview and the shared approximation table (A1 to A17); the sections after
them, one per phase, give derivations, measured values and the
approximation tables of each solver (L, D, B, R, G, P, V and E IDs). The
display-only approximations of the space renderer are listed in
[rendering.md, section 16](rendering.md#16-approximations).

Method and model are kept apart: lattice Boltzmann is a numerical method
that recovers the Navier-Stokes model in a limit; the panel method and
modified Newtonian theory are models solved directly. No solver here is
validated CFD; each row states the envelope inside which its results are
meant to be used.

## Formula index

Machine-readable twin: [physics_index.json](physics_index.json). The docs
check (`tools/check_docs.py`, ctest `docs.check`) fails when an indexed
formula has no row here, when a row has no index entry, when a named file
or symbol no longer exists, or when a source file with an `APPROXIMATION`
marker belongs to no entry. A source marked "reference to be confirmed" is
the project's own derivation or a standard estimate whose original
publication has not been checked for this document.

Columns: the formula or model; its source; where it is implemented; the
envelope it is meant for; the approximations it carries (IDs of the tables
below, with the measured or stated error bound); the tests that verify it,
with the decisive measured value.

### Space dynamics (orrery_core)

| ID | Formula or model | Source | Implementation | Envelope | Approximation and bound | Verified by |
|---|---|---|---|---|---|---|
| `core.units` | AU = 149597870700 m, day = 86400 s, k = 0.01720209895, G = k^2 AU^3 Msun^-1 day^-2 (`GravitationalConstant`) and GM_sun = k^2 AU^3 day^-2 (`GravitationalParameter`): different dimensions, the same number in these units because Msun is the mass unit; c = 173.1446326742403 AU/day; M_sun [kg] = GM_sun / G | IAU 2012 Resolution B2; IAU 1976 (k); CODATA 2018 (G); IERS Conventions 2010 (GM_sun comparison) | `src/core/include/orrery/core/units.h`, `src/core/include/orrery/core/quantity.h` | all domains (ADR-001) | M_sun in kg uncertain to 2.2e-5 relative through G; dynamics in one unit system unaffected (section 1.1) | `tests/core/test_units.cpp`, `tests/core/test_quantity.cpp`: conversions round trip within a few ulp |
| `core.kepler` | M = E - e sin E and M = e sinh H - H by safeguarded Newton; perifocal state rotated by R3(Omega) R1(i) R3(omega); elements from h, the eccentricity vector and vis-viva | Vallado, *Fundamentals of Astrodynamics and Applications*, 4th ed., sections 2.2 and 2.6; Danby's starting value | `src/core/kepler.cpp` | elliptic and hyperbolic orbits, abs(e - 1) >= 1e-9, non-rectilinear states | osculating two-body conic, exact at the instant; circular and equatorial below e = 1e-13, sin i = 1e-14 (section 2.1) | `tests/core/test_kepler.cpp`: 10296 round trips, worst relative error 8.5e-14 (bound 1e-12) |
| `core.gravity.newton` | a_i = sum_j G m_j (r_j - r_i) / (abs(r_j - r_i)^2 + eps^2)^(3/2); U = -sum G m_i m_j / (r^2 + eps^2)^(1/2); probes receive force only | Newtonian gravity with Plummer softening (standard N-body practice) | `src/core/newtonian_gravity.cpp`, `src/core/include/orrery/core/force_model.h`, `src/core/include/orrery/core/body.h` | reference model for N <= 256; any separation (power-of-two scaled path) | A14 softening: relative force error 1.5 eps^2 / r^2 for r >> eps; point masses (largest neglected planetary term about 3e-8, relativistic); probes as test particles, error of order m_probe / M | `tests/core/test_gravity.cpp`: force = -grad U to 1e-7, probe leaves the others bit-identical |
| `core.integrators.fixed` | Leapfrog KDK; Yoshida 4th order, w1 = 1 / (2 - 2^(1/3)), w0 = -2^(1/3) / (2 - 2^(1/3)); classical RK4 (tests only) | Yoshida 1990, *Phys. Lett. A* 150, 262; ADR-002 | `src/core/integrator.cpp`, `src/core/include/orrery/core/integrator.h` | dt a small fraction of the shortest dynamical time | fixed-step discretisation, global error O(dt^2) / O(dt^4); Sun + 8 planets, 1000 years, Yoshida4 at 0.25 d: max abs(dE/E) 1.4e-10 | `tests/core/test_integrators.cpp`: measured orders 2.000 / 3.998 / 4.13, drift below 1e-9, reversibility 3.4e-13 AU |
| `core.integrators.ias15` | 15th-order Gauss-Radau predictor-corrector on 8 spacings; step h' = h (epsilon / err)^(1/7), epsilon = 1e-9; Kahan-compensated update | Rein and Spiegel 2015, *MNRAS* 446, 1424; Everhart 1985 | `src/core/ias15.cpp`, `src/core/include/orrery/core/ias15.h` | forces smooth over one internal step | acceleration a degree-7 polynomial per step (local error O(h^16)), kept near machine precision | `tests/core/test_ias15.cpp`: Kepler e = 0.5, 10 orbits, 8.1e-14 AU; golden hash pinned |
| `core.barnes_hut` | Octree on 63-bit Morton keys; accept a node when d > side / theta + offset; softened monopole + quadrupole field a = -M r / D^3 + (3 Q r + 1.5 tr(Q) r) / D^5 - 7.5 (r.Q.r) r / D^7 | Barnes and Hut 1986; Barnes 1994 (opening criterion) | `src/core/barnes_hut.cpp`, `src/core/octree.cpp`, `src/core/include/orrery/core/barnes_hut.h` | large-N collisionless systems, theta in [0, 1], positions within about 1e-150 to 1e150 AU | A15 truncation after the quadrupole, error O((side/d)^3); momentum not exactly conserved | `tests/core/test_barnes_hut.cpp`: N = 2000, theta 0.5, median acceleration error 4.70e-4 (bound 1e-3), max 1.01e-2 |
| `core.conservation` | E = sum m v^2 / 2 + U, P = sum m v, L = sum m r x v, drifts against scale sums; 64-bit FNV-1a state hash | FNV-1a hash (Fowler, Noll and Vo) | `src/core/conservation.cpp`, `src/core/state_hash.cpp` | sources of gravity (probes excluded by construction) | none (bookkeeping); momentum drift is a rounding random walk of about DBL_EPSILON sqrt(steps) | `tests/core/test_conservation.cpp`, `tests/core/test_determinism.cpp`: six golden hashes, momentum drift 6.0e-14 over 1000 years |
| `core.black_hole.radii` | r_s = 2 G M / c^2 (2953.25 m per solar mass); photon sphere 1.5 r_s; ISCO 3 r_s; marginally bound orbit 2 r_s | Schwarzschild solution (standard, e.g. Misner, Thorne and Wheeler, *Gravitation*, 1973) | `src/core/include/orrery/core/black_hole.h`, `src/core/black_hole.cpp` | non-rotating holes | spin stored but ignored (R1, ADR-003) | `tests/core/test_black_hole.cpp`: r_s equals 2 GM_sun / c^2 in SI to 1e-14 |
| `core.paczynski_wiita` | Phi = -G M / (r - r_s), pair potential with R_ij the sum of the pair's Schwarzschild radii; circular v^2 = G M r / (r - r_s)^2, T = 2 pi r (r - r_s) / sqrt(G M r) | Paczynski and Wiita 1980; ADR-004 | `src/core/paczynski_wiita_gravity.cpp`, `src/core/include/orrery/core/paczynski_wiita_gravity.h`, `src/core/black_hole.cpp` | exact ISCO (3 r_s) and marginally bound orbit; qualitative strong-field orbits | A12 / B1 no correct periapsis precession (the potential precesses, but not by the Schwarzschild amount), no frame dragging or waves; B2 force capped inside (1 + 1e-3) R_ij | `tests/core/test_black_hole.cpp`: circular period at 10 r_s to 9.0e-12; r0 >= 3.5 r_s stable, 2.5 and 2.9 r_s plunge |
| `core.tidal` | r_t = R (M / m)^(1/3); disruption event when the swept path enters r_t from outside and r_t > r_s | standard order-of-magnitude tidal radius (reference to be confirmed) | `src/core/tidal.cpp`, `src/core/include/orrery/core/black_hole.h` | criterion to within a factor of about 2 in radius | B3 prefactor 1 (structure gives 1 to 2.4); detection only, no debris | `tests/core/test_events.cpp`: pericentre plunge detected for 1, 7 or 64 output intervals |
| `core.horizon` | absorption when a centre comes within r_s (r_s1 + r_s2 for two holes); merged mass, momentum and centre of mass, relative L reported as internal | ADR-004 | `src/core/horizon.cpp`, `src/core/include/orrery/core/horizon.h` | bookkeeping of mass and momentum | B5 absorption at the point-mass horizon; no accretion luminosity | `tests/core/test_black_hole.cpp`, `tests/core/test_events.cpp`: mass 0, momentum 1.2e-16, L (orbital + internal) 2.0e-16 relative |
| `core.collision` | earliest root of abs(p0 + tau (p1 - p0)) = R_i + R_j (cancellation-free miss distance); loose-octree broad phase; inelastic merge m = m1 + m2, R = (R1^3 + R2^3)^(1/3) | spec section 3.5 (project derivation) | `src/core/collision.cpp`, `src/core/include/orrery/core/collision.h`, `src/core/event_step.cpp` | integration segments short against the encounter time | B4 linear relative motion within a segment; B6 perfectly inelastic point merge | `tests/core/test_collisions.cpp`, `tests/core/test_events.cpp`: merge conserves mass and momentum to 1.3e-16; tangent tests hit or miss at 1e-8 of R |
| `core.geodesic` | u'' = -u + 3 u^2 (G = c = 1, lengths in M), RK4 in phi under a 512-step budget; b = r0 sin(alpha) / sqrt(1 - 2/r0); capture for b < 3 sqrt(3) M; weak-field deflection 4/b + 15 pi / (4 b^2) | Schwarzschild null geodesics (Misner, Thorne and Wheeler 1973); Bozza 2002 (strong-deflection limit); ADR-003 | `src/core/schwarzschild_geodesic.cpp` | static observer outside the horizon; non-rotating hole | R5 escape angle from the straight line, below 1e-12 rad; RK4 at 4 pi / 512 adds about 2e-8 rad | `tests/core/test_schwarzschild_geodesic.cpp`: deflection at b = 1000 to 5000 M within the third-order term + 2e-8; capture exactly for b < b_c |
| `core.thin_disk` | T(r) = T_peak f(r / r_in) / f(49/36), f(x) = x^(-3/4) (1 - x^(-1/2))^(1/4); g = sqrt(1 - 3/r) / (1 - Omega lambda) / sqrt(1 - 2/r0), Omega = r^(-3/2) | Shakura and Sunyaev 1973; Page and Thorne 1974 (Newtonian limit) | `src/core/schwarzschild_geodesic.cpp` | thin, optically thick disk from the ISCO (6 M) to 30 M | R2 Newtonian-limit temperature (Novikov-Thorne factors differ by tens of percent near the ISCO); luminosity display-scaled | `tests/core/test_schwarzschild_geodesic.cpp`, `tests/render/test_black_hole.cpp`: g against p.u to 1e-12, peak at 49/36 r_in |
| `core.scenarios` | J2000 state vectors (DE441 through Horizons); figure-eight choreography (Simo's initial conditions); Sgr A* and S2 best fit | JPL Horizons, DE441 (retrieved 2026-11-11); Chenciner and Montgomery 2000; GRAVITY Collaboration 2020, *A&A* 636, L5 | `assets/scenarios/solar_system_j2000.json`, `assets/scenarios/figure_eight.json`, `assets/scenarios/sgr_a_s2.json` | J2000: planetary positions to about 1e-6 AU over a year; S2: period and shape | ten point masses; S2 Newtonian, no Schwarzschild precession | `tests/core/test_scenarios_phase3a.cpp`: Mercury 3.9e-7 AU after one year; figure-eight period 5.1e-9 from the literature; S2 16.045 yr (16.0 +- 0.3) |

### Atmosphere and entry (orrery_atmo)

| ID | Formula or model | Source | Implementation | Envelope | Approximation and bound | Verified by |
|---|---|---|---|---|---|---|
| `atmo.us76` | H = r0 Z / (r0 + Z); TM = TM,b + L (H - Hb); layer pressure by power law or exponential; rho = P M0 / (R* TM); a = sqrt(1.4 R* TM / M0); log-linear rho and P above 86 km | U.S. Standard Atmosphere 1976 (NOAA-S/T 76-1562), Tables 4, 8 and I | `src/atmo/us76.cpp`, `src/atmo/layers.h`, `src/atmo/include/orrery/atmo/atmosphere.h` | 0 to 1000 km geometric altitude | E1 interpolation above 86 km (a few percent between nodes, no solar activity) | `tests/atmo/test_atmosphere.cpp`: Table I values to half a unit of the last printed digit; base pressures to seven figures |
| `atmo.mars` | T = 242.15 K - 0.998 K/km h below 7 km, -2.22 K/km above, isothermal above 50 km; hydrostatic pressure from 699 Pa, R = 192.1 J/(kg K), g0 = 3.711 m/s^2 | NASA Glenn Research Center Mars atmosphere fit (Mars Global Surveyor, 1996) | `src/atmo/atmosphere.cpp`, `src/atmo/include/orrery/atmo/atmosphere.h` | mean conditions | E2 one mean profile: within 10 percent of the fit's pressure below 10 km; season and dust change density by 2x or more | `tests/atmo/test_atmosphere.cpp` |
| `atmo.exponential` | rho = rho0 exp(-h / H), p = rho R T; Mach M = abs(v_rel) / a | Allen and Eggers, NACA Report 1381 (1958) | `src/atmo/atmosphere.cpp`, `src/atmo/include/orrery/atmo/atmosphere.h` | analytic studies | E3 constant scale height, independent temperature | `tests/atmo/test_atmosphere.cpp` |
| `atmo.planet` | spherical planet of mean radius with point-mass gravity and rotation rate; Earth GM 3.986004418e14 m^3/s^2, omega 7.292115e-5 rad/s; Mars GM 4.282837e13 m^3/s^2, R 3389.5 km | Earth GM and rotation rate: WGS 84; mean radii and Mars constants: reference to be confirmed | `src/atmo/planet.cpp`, `src/atmo/include/orrery/atmo/planet.h` | entry dynamics | E4 no J2 (1e-3 of g), geocentric latitude, no terrain | `tests/atmo/test_drag.cpp`, `tests/atmo/test_entry.cpp` |
| `atmo.cd_table` | Cd(M) linear between samples, end values held outside; samples computed by an `IAeroSolver` at chosen Mach numbers | ADR-012, ADR-014 | `src/atmo/aero_table.cpp`, `src/atmo/cd_sampling.cpp`, `src/atmo/include/orrery/atmo/aero_table.h` | inside the sampled Mach range | A11 linear interpolation; E5 end values held (up to about 30 percent off for a capsule below the lowest sample) | `tests/atmo/test_aero_table.cpp`: Apollo table from the Newtonian solver, Mach 5 to 25, within 10 percent of 1.3 |
| `atmo.drag` | F = -1/2 rho abs(v_rel) v_rel Cd(M) A; v_rel = v (inertial air) or v - omega z x r (co-rotating) | ADR-012 | `src/atmo/drag.cpp`, `src/atmo/include/orrery/atmo/drag.h` | ballistic bodies below about 90 km | E6 drag only, Cd of Mach only (no lift, no rarefied regime); E7 air at rest by default (up to 465 m/s airspeed error at the equator) | `tests/atmo/test_drag.cpp` |
| `atmo.entry_split` | v += kick(dt/2); (r, v) = gravity step(dt); v += kick(dt/2); kick: RK4 on dv/dt = F(r, v) / m at fixed r in substeps of at most 0.05 drag times | Strang 1968, *SIAM J. Numer. Anal.* 5, 506; ADR-021 | `src/atmo/entry_impl.cpp`, `src/atmo/include/orrery/atmo/entry.h` | gravity steps well below the deceleration time v / a | E8 second order in dt; dt 0.5 s against 0.1 s changes peak g by at most 1e-4 relative | `tests/atmo/test_entry.cpp`: energy balance 2.1e-5 at dt 0.5 s (bound 1 percent); stiff kick within 1e-4 of the analytic decay |
| `atmo.entry_diagnostics` | g-load abs(F) / (m g0); q_s = k sqrt(rho / R_n) abs(v_rel)^3; heat load by the trapezoidal rule over q_s; E = m (v^2/2 - GM/r) with E(t) - E(0) = integral of F.v dt; event instants by interpolation; Allen-Eggers peak deceleration as test reference | Sutton and Graves, NASA TR R-376 (1971); Allen and Eggers 1958 | `src/atmo/entry_record.cpp`, `src/atmo/entry.cpp` | heating indicative below about 80 km | A8, E10 continuum convective heating only; E9 interpolated event instants (fraction of a step) | `tests/atmo/test_entry.cpp`: Apollo-like peak 6.78 g (6 to 8 g), skip-out reported, Allen-Eggers -0.46 percent (bound 1.5) |

### Aerodynamics (orrery_aero)

| ID | Formula or model | Source | Implementation | Envelope | Approximation and bound | Verified by |
|---|---|---|---|---|---|---|
| `aero.flow_conditions` | a = sqrt(gamma R T), M = V / a, Re = rho V L / mu, q = rho V^2 / 2; Cd, Cl, Cy = F.direction / (q S), Cm = M_y / (q S l) | standard definitions | `src/aero/flow_conditions.cpp`, `src/aero/aero_result.cpp` | every solver | none; air defaults gamma = 1.4, R = 287.05 J/(kg K), US76 sea level | `tests/aero/test_regime.cpp`, `tests/aero/test_newtonian.cpp` |
| `aero.regime` | recommend Newtonian for M >= 5; panel for M < 0.3 and Re >= 1e5; lattice Boltzmann for M < 0.3 and Re < 1e5; nearest solver otherwise; user override with warnings, never blocked | ADR-009 | `src/aero/regime.cpp` | rules of thumb, not sharp limits | P6 boundaries Mach 0.3 and 5, Re 1e5, 10 degrees | `tests/aero/test_regime.cpp`: every warning fires, nothing blocks |
| `aero.panel2d` | constant source per panel plus one vorticity; zero normal velocity at midpoints; Kutta V_t,1 + V_t,N = 0; Cp = 1 - (V_t / V)^2; lift rho V Gamma | Hess and Smith 1967; Katz and Plotkin, *Low-Speed Aerodynamics*, 2001 | `src/aero/panel2d.cpp`, `src/aero/include/orrery/aero/panel_method.h` | attached incompressible flow, thin sections, small angle of attack | A5 potential flow; P5 sideslip ignored | `tests/aero/test_panel.cpp`, `tests/aero_validation/test_naca0012.cpp`: cylinder Cp to 1.1e-14; NACA 0012 slope 1.098 x 2 pi (bound 10 percent) |
| `aero.panel3d` | constant source sigma = -n.V and doublet mu per triangle; internal Dirichlet condition (Katz and Plotkin eq. 12.12); solid-angle doublet and edge-logarithm source influence; Kutta mu_w = mu_upper - mu_lower; Trefftz-plane CL and CDi; dense LU up to 4000 unknowns, GMRES above | Katz and Plotkin 2001; van Oosterom and Strackee 1983 | `src/aero/panel3d.cpp`, `src/aero/panel_influence.cpp`, `src/aero/linear_solve.cpp` | attached flow, thin bodies, small angle of attack | A5 fixed flat wake; P1 far-field multipoles beyond 10 diameters; P2 to P4 wake, Kutta and gradient treatment | `tests/aero/test_panel.cpp`, `tests/aero_validation/test_wing.cpp`: sphere Cp rms 0.026; AR 6 wing CL 0.3915, e = 0.988; GMRES = LU to 3.8e-12 |
| `aero.skin_friction` | Cf = 1.328 / sqrt(Re) for Re <= 5e5, else 0.455 / (log10 Re)^2.58 - 1700 / Re; Cd_f = Cf S_wet / S_ref, reported separately | Schlichting, *Boundary-Layer Theory* | `src/aero/panel2d.cpp` | attached boundary layer at zero pressure gradient | A6 flat-plate correlation, transition fixed at Re 5e5 | `tests/aero/test_panel.cpp` |
| `aero.newtonian` | Cp = Cp_max sin^2(theta) on windward faces, 0 in shadow; Cp_max = 2 / (gamma M^2) (p02/p1 - 1) with the Rayleigh pitot formula (1.839 as M -> infinity for gamma 1.4) | Lees 1955; Anderson, *Hypersonic and High-Temperature Gas Dynamics* | `src/aero/newtonian.cpp`, `src/aero/include/orrery/aero/newtonian.h` | M >= 5, blunt bodies | A7 no shock-layer structure or shadow-side pressure | `tests/aero/test_newtonian.cpp`, `tests/aero_validation/test_apollo.cpp`: sphere Cd within 6.6e-5 of Cp_max / 2; Apollo Cd 1.367 at 20 degrees (1.3 within 10 percent) |
| `aero.sutton_graves` | q_s = k sqrt(rho / R_n) V^3, k = 1.7415e-4 kg^0.5/m (Earth), 1.9027e-4 (Mars) | Sutton and Graves, NASA TR R-376 (1971) | `src/aero/newtonian.cpp` | convective stagnation-point heating of blunt bodies | A8 indicative only: no radiative, catalytic or chemical effects, cold wall | `tests/aero/test_newtonian.cpp` |
| `aero.lbm.units` | dx = L / N, dt = u_lat dx / U, nu_lat = nu dt / dx^2, tau = 3 nu_lat + 1/2; Re identical in both systems; Ma_lat = u_lat sqrt(3) < 0.1 enforced | Kruger et al., *The Lattice Boltzmann Method*, 2017 | `src/aero/lbm/lattice_units.cpp`, `src/aero/lbm/include/orrery/aero/lbm/lattice_units.h` | tau > 1/2, Ma_lat < 0.1 | A3 weak compressibility, density error of order Ma^2 | `tests/aero/lbm/test_lattice_units.cpp`: SI and lattice round trip, refusal at Ma_lat >= 0.1 |
| `aero.lbm.d2q9` | D2Q9 weights 4/9, 1/9, 1/36; f_eq = w rho (1 + 3 c.u + 9/2 (c.u)^2 - 3/2 u.u); BGK; Guo forcing; Smagorinsky tau_eff = (tau + sqrt(tau^2 + 18 sqrt(2) C^2 abs(Pi_neq) / rho)) / 2; GPU populations in binary32 | Qian, d'Humieres and Lallemand 1992; Guo, Zheng and Shi 2002; Hou et al. 1996; Kruger et al. 2017 | `src/aero/lbm/include/orrery/aero/lbm/d2q9.h`, `src/aero/lbm/include/orrery/aero/lbm/lbm2d.h`, `src/aero/lbm/lbm2d_cpu.cpp`, `src/aero/lbm/lbm2d_common.cpp`, `src/aero/lbm/lbm2d_gpu.cpp`, `src/aero/compute/float32_pack.cpp`, `src/render/shaders/lbm/lbm_d2q9_common.glsl` | low lattice Mach; Smagorinsky above Re about 2000 | A2 eddy-viscosity closure; A3; L4 Pi_neq without forcing correction; L5 binary32 on the GPU (L2 1e-4, 7.1e-4 in shedding transients) | `tests/aero/lbm/test_d2q9.cpp`, `tests/aero/lbm/test_lbm2d_flows.cpp`, `tests/aero/lbm/test_lbm_gpu.cpp`: Poiseuille 5e-13, Taylor-Green decay within 0.08 percent |
| `aero.lbm.d2q9_boundaries` | halfway bounce-back f_i = f*_opp(i), moving wall + 6 w_i rho c_i.u_w, specular free slip; regularized Zou-He velocity inlet; convective outlet f(t+1) = (f(t) + U f_upstream(t+1)) / (1 + U) | Zou and He 1997; Latt et al. 2008; Yang 2013 | `src/aero/lbm/lbm2d_internal.h`, `src/aero/lbm/include/orrery/aero/lbm/lbm2d.h`, `src/render/shaders/lbm/lbm_d2q9_common.glsl` | smooth inflow, body 5 or more lengths from the inlet | A4 staircase walls; L1 regularized inlet; L2 convective outlet (0.1 percent mean density drift over 1e5 steps); L3 moving-wall density | `tests/aero/lbm/test_lbm2d_flows.cpp`: Couette 3e-13, mass conserved below 1e-12 |
| `aero.lbm.forces2d` | momentum exchange F = sum over links of 2 c_o f*_o; Cd = F_x / (rho U^2 L / 2); Strouhal number from the dominant oscillation of the lift history; wall Cp | Ladd 1994 | `src/aero/lbm/lbm_aero_solver.cpp`, `src/aero/lbm/lbm2d_cpu.cpp`, `src/aero/lbm/include/orrery/aero/lbm/lbm_aero_solver.h`, `src/aero/lbm/force_history.cpp` | stationary bodies | L6 stationary momentum exchange; L7 wall Cp first order; L8 sideslip ignored | `tests/aero/lbm/test_lbm_aero_solver.cpp`, `tests/aero/lbm/test_force_history.cpp`, `tests/aero_validation/lbm2d/test_cylinder.cpp`: cylinder Re 100 St 0.1674, Cd 1.468 |
| `aero.lbm.solid_mask` | node solid flag from one sample per node (circle, NACA section, polygon, SDF, voxel slice; sphere, box, SDF in 3D) | project derivation | `src/aero/lbm/solid_mask.cpp`, `src/aero/lbm/include/orrery/aero/lbm/solid_mask.h`, `src/aero/lbm3d/solid_mask3d.cpp`, `src/aero/lbm3d/include/orrery/aero/lbm3d/solid_mask3d.h` | features resolved by several cells | A4 staircase wall, first order in the resolution | `tests/aero/lbm/test_solid_mask.cpp`, `tests/aero/lbm3d/test_solid_mask3d.cpp` |
| `aero.lbm.d3q19` | D3Q19 weights 1/3, 1/18, 1/36; equilibrium, BGK, Guo forcing and Smagorinsky as in D2Q9 with abs(Pi_neq) over six components | Qian, d'Humieres and Lallemand 1992; Hou et al. 1996; ADR-008 | `src/aero/lbm3d/include/orrery/aero/lbm3d/d3q19.h`, `src/aero/lbm3d/include/orrery/aero/lbm3d/lbm3d.h`, `src/aero/lbm3d/lbm3d_cpu.cpp`, `src/aero/lbm3d/lbm3d_common.cpp`, `src/aero/lbm3d/lbm3d_gpu.cpp`, `src/render/shaders/lbm/lbm_d3q19_common.glsl` | as D2Q9 | A2, A3, L4, L5 | `tests/aero/lbm3d/test_d3q19.cpp`, `tests/aero/lbm3d/test_lbm3d_flows.cpp`, `tests/aero/lbm3d/test_lbm3d_gpu.cpp`: square duct 5.5e-4, GPU against CPU L2 1.6e-5 |
| `aero.lbm.d3q19_boundaries` | Zou-He inlet rho = (sum_{c_x=0} f + 2 sum_{c_x=-1} f) / (1 - u_x), regularized; interpolated bounce-back with q = d(x) / (d(x) - d(x - c_j)) from the SDF; absorbing layer sigma = s ((T - d) / T)^2; start ramp U (1 - cos(pi t / T)) / 2 | Zou and He 1997; Latt et al. 2008; Bouzidi, Firdaouss and Lallemand 2001; Xu and Sagaut 2013; Hecht and Harting 2010 | `src/aero/lbm3d/lbm3d_internal.h`, `src/render/shaders/lbm/lbm_d3q19_common.glsl` | walls resolved by a few cells, body 2 or more lengths from the inlet and 3 from the layer | D1 regularized inlet; D2 interpolated wall not exactly mass conserving; D3 absorbing layer; D4 gradual start | `tests/aero/lbm3d/test_lbm3d_flows.cpp`, `tests/aero/lbm3d/test_lbm3d_gpu.cpp`: Poiseuille 7.3e-13, mass 1.4e-13 |
| `aero.lbm.forces3d` | momentum exchange F = sum c_o (f*_o(x) + f_j(x, t+1)), moment at x - q c_j, force unit rho_ref dx^4 / dt^2; sphere reference Cd = 24 / Re (1 + 0.15 Re^0.687) | Ladd 1994; Mei, Yu, Shyy and Luo 2002; Schiller and Naumann 1933 | `src/aero/lbm3d/lbm_aero_solver3d.cpp`, `src/aero/lbm3d/include/orrery/aero/lbm3d/lbm_aero_solver3d.h` | stationary bodies, blockage below 5 percent | D5 stationary bodies; D6 finite channel (sphere Re 10 in 6 D: +9.6 percent); D7 wall samples; D8 field only when wind-aligned | `tests/aero/lbm3d/test_lbm_aero_solver3d.cpp`, `tests/aero_validation/lbm3d/test_sphere.cpp`, `tests/aero_validation/lbm3d/test_plate.cpp`: sphere Re 100 ratio 1.086 (bound 15 percent), plate Cd 1.158 |

### Geometry (orrery_geom)

| ID | Formula or model | Source | Implementation | Envelope | Approximation and bound | Verified by |
|---|---|---|---|---|---|---|
| `geom.primitives` | exact volume (integral of pi r^2 dz) and area (integral of 2 pi r ds) of solids of revolution; regular N-gon frustum closed forms; ellipsoid area 4 pi a b c R_G(a^-2, b^-2, c^-2); mesh signed volume and centroid by the divergence theorem | Carlson 1995 (symmetric elliptic integrals) | `src/geom/primitives.cpp`, `src/geom/include/orrery/geom/primitives.h`, `src/geom/triangle_mesh.cpp` | closed, consistently oriented meshes | G1 Apollo- and Orion-like presets (flat apex) | `tests/geom/test_primitives.cpp`: worst 6.1e-14 relative over 65 checks (bound 1e-9) |
| `geom.naca` | y_t = 5 t (0.2969 sqrt(x) - 0.1260 x - 0.3516 x^2 + 0.2843 x^3 - 0.1036 x^4), camber of two parabolas, thickness normal to the camber line; area by Gauss-Legendre after x = u^2 | Abbott and von Doenhoff, *Theory of Wing Sections*, 1959 | `src/geom/naca.cpp`, `src/geom/include/orrery/geom/naca.h` | 4-digit sections | G2 closed trailing edge (-0.1036 for -0.1015, 0.13 percent of chord at t = 0.12) | `tests/geom/test_naca.cpp`: NACA 0012 area to 1e-13 relative |
| `geom.import` | unit from the bounding-box extent e (e > 500: mm, 50 < e <= 500: cm, else m); single-precision assimp values widened to double | ADR-018 | `src/geom/mesh_import.cpp`, `src/geom/include/orrery/geom/mesh_import.h` | bodies of 5 cm to 500 m, detail above 1e-7 of the extent | G3 unit guess (caller confirms); G7 float resolution 6e-8 relative | `tests/geom/test_import.cpp`: hostile inputs (NaN, bad indices, inflated headers, oversized files) rejected before parsing; unit and up-axis estimates |
| `geom.repair` | edge-manifold analysis, orientation propagation, outward orientation by signed volume, fan hole filling | ADR-018 (project kernels) | `src/geom/mesh_repair.cpp`, `src/geom/include/orrery/geom/mesh_repair.h` | small, nearly planar holes | G4 fan from the loop centroid | `tests/geom/test_repair.cpp`: broken STL fixture counted exactly, repaired volume error below 1e-15 |
| `geom.winding_number` | w(p) = sum of signed solid angles / 4 pi; triangle solid angle by the Van Oosterom-Strackee formula; hierarchical far field (dipole, beta = 2) | Jacobson, Kavan and Sorkine-Hornung 2013; Barill et al. 2018; Van Oosterom and Strackee 1983 | `src/geom/winding_number.cpp`, `src/geom/include/orrery/geom/winding_number.h` | closed meshes; values far from 1/2 | G5 first-order far field, error a few percent | `tests/geom/test_voxel.cpp`: worst abs error 0.024 against the direct sum (bound 0.05) |
| `geom.voxel_sdf` | solid flag at cell centres from w >= 1/2; narrow-band signed distance from the closest point on the triangles | ADR-018 | `src/geom/voxel_grid.cpp`, `src/geom/include/orrery/geom/voxel_grid.h` | features larger than a cell | A4 one sample per cell (staircase); SDF exact to rounding within the band | `tests/geom/test_voxel.cpp`: sphere 85184 of 85184 cells (bound 0.999); SDF 8.6e-4 against a 1.7e-3 facet sagitta |
| `geom.panel_mesh` | panels with collocation point, normal and area from triangles; trailing-edge detection; flat wake strips 50 reference lengths long | Katz and Plotkin 2001 | `src/geom/panel_mesh.cpp`, `src/geom/include/orrery/geom/panel_mesh.h` | small angle of attack | A5 / P2 fixed flat wake, no roll-up | `tests/geom/test_panel.cpp` |
| `geom.reference` | frontal, planform or wetted reference area; projected area by raster coverage; centre of mass of a uniform solid | spec section 4.7 | `src/geom/reference_quantities.cpp`, `src/geom/include/orrery/geom/reference_quantities.h` | solid, roughly homogeneous bodies | A1 uniform density; G6 raster error below perimeter x cell (0.4 percent for a sphere) | `tests/geom/test_reference.cpp` |

### Rendering and display (orrery_render, orrery_app)

These rows feed no physics back into a solver; they bound what the pictures
show. Space-view items without a short ID are in
[rendering.md, section 16](rendering.md#16-approximations).

| ID | Formula or model | Source | Implementation | Envelope | Approximation and bound | Verified by |
|---|---|---|---|---|---|---|
| `render.bh_trace` | the `core.geodesic` ray march in binary32 on the GPU (RK4 in phi, 512 steps, half resolution), disk crossings by cubic Hermite dense output | ADR-017 | `src/render/shaders/black_hole_trace.comp.glsl`, `src/render/black_hole_pass.cpp` | static camera; one hole | R3 disk opacity and texture; R4 static camera, budget-exhausted rays captured; R11 radiance bound 1e4 | `tests/render/test_black_hole_render.cpp`: escape direction median 6.1e-7 rad against the double reference |
| `render.bh_emission` | I_obs(nu) = g^3 I_em(nu / g), so the disk shows B(g T); visible-band radiance of B_lambda times the CIE 1931 colour matching functions, converted to linear sRGB | Planck's law; CIE 1931; Wyman, Sloan and Shirley 2013 (*JCGT*, matching-function fit) | `src/render/black_hole.cpp`, `src/render/blackbody_spectrum.cpp`, `src/render/include/orrery/render/blackbody_spectrum.h` | black bodies above about 1900 K for an exact gamut | R1 Schwarzschild only; R2 thin disk; R9 fitted matching functions (1e-3 in xy); R10 gamut clipping | `tests/render/test_black_hole.cpp` |
| `render.bh_composite` | half-resolution trace upsampled edge-aware; occlusion depth from the nearest sample; lensed sky cube with Gaussian stars | ADR-017 | `src/render/shaders/black_hole_composite.frag.glsl`, `src/render/shaders/sky_cube_stars.vert.glsl`, `src/render/black_hole.cpp` | a single dominant hole | R6 one lens, bodies unlensed; R7 upsampling; R8 star brightness within 10 percent | `tests/render/test_black_hole_render.cpp`: shadow radius 122.8 px for 122.9 px; Einstein ring 143.52 px for 143.52 px |
| `render.tone_map` | ACES RRT + ODT fit: input matrix, a = v (v + 0.0245786) - 0.000090537, b = v (0.983729 v + 0.4329510) + 0.238081, a / b, output matrix; sRGB transfer function | Hill's ACES fit (BakingLab `ACES.hlsl`); IEC 61966-2-1 | `src/render/tone_map.cpp`, `src/render/include/orrery/render/tone_map.h`, `src/render/shaders/tonemap.frag.glsl` | scene-linear [0, about 16] | a few percent from the reference CTL transforms; output clamped to [0, 1] | `tests/render/test_tone_map.cpp`: GPU equals the CPU reference within one 8-bit level |
| `render.body_visuals` | main-sequence R ~ M^0.8, L ~ M^3.5, T = T_sun (L / R^2)^(1/4); Planckian-locus cubic fits; irradiance L (1 AU / d)^p; minimum screen radius d m / sqrt(s^2 + m^2); star flux 10^(-0.4 x 0.85 m) | Kim et al. 2002 (Planckian locus fit) | `src/render/body_visuals.cpp`, `src/render/space_renderer.cpp` | display only | rendering.md section 16: stars within about 2x on the main sequence; colour 1e-3 in xy; p = 0.5 compresses brightness ratios | `tests/render/test_body_visuals.cpp` |
| `render.body_shading` | Lambert plus energy-normalised Blinn-Phong with Schlick Fresnel (F0 = 0.04); limb darkening I(mu) = 1 - 0.6 (1 - mu); value-noise albedo | Schlick 1994 (Fresnel approximation) | `src/render/shaders/body.frag.glsl`, `src/render/shaders/test_shape.frag.glsl` | display only | rendering.md section 16: no shadows, eclipses or atmospheres; procedural surfaces | `tests/render/test_space_render.cpp`, `tests/render/test_space_golden.cpp` |
| `render.sky` | star counts N(< m) ~ 10^(0.35 m) over V = -1.5 to 10; spectral mix per class; galactic frame; Milky Way surface brightness model; glare PSF 0.55 / (1 + x^2)^2 with r^-3 wings | IAU galactic coordinate system | `src/render/starfield.cpp`, `src/render/shaders/glow.frag.glsl`, `src/render/shaders/milky_way.frag.glsl` | display only | rendering.md section 16: population statistics, not a catalogue | `tests/render/test_starfield.cpp`, `tests/render/test_space_golden.cpp`: about 4000 stars brighter than V = 6; band twice as bright towards the centre |
| `render.entry_view` | single-colour exponential haze (scale height a quarter of the shell) ray-marched in 16 samples between the shell and the planet with a wrapped sun term; albedo planet with soft terminator and graticule; altitudes exaggerated | project display model | `src/render/shaders/entry/entry_atmosphere.frag.glsl`, `src/render/shaders/entry/entry_planet.frag.glsl`, `src/render/include/orrery/render/entry_view.h` | display only | A16: not Rayleigh or Mie scattering, not radiative transfer | `tests/render/entry/test_entry_view.cpp` |
| `render.camera` | rotation smoothing by a slerp analogue of the exponential blend; Surface-mode latitude and longitude by the midpoint rule per substep of at most 1/240 s | project derivation | `src/render/camera.cpp` | camera placement only | rendering.md section 16: second order in the substep | `tests/render/test_camera.cpp` |
| `render.aero_field` | trilinear (bilinear) interpolation; vorticity and Q = (abs(Omega)^2 - abs(S)^2) / 2 by finite differences; section sampling; lattice Cp from c_s^2 (rho - 1) | Hunt, Wray and Moin 1988 (Q-criterion) | `src/render/aero/aero_field.cpp` | resolved structures of two or more cells | V1, V6, V7, V10 | `tests/render/aero/test_aero_field.cpp` |
| `render.aero_streamlines` | streamlines by RK4 in arc length (0.5 cells); particles by the explicit midpoint rule | project derivation | `src/render/aero/aero_streamlines.cpp`, `src/render/shaders/aero/compute/aero_vis_flow_streamlines.comp.glsl`, `src/render/shaders/aero/compute/aero_vis_flow_particles.comp.glsl` | display only | V2 radius drift 2.7e-8 m over 400 steps on a circular flow; V3 | `tests/render/aero/test_streamlines.cpp` |
| `render.aero_isosurface` | marching cubes with a generated 256-case table and linear edge interpolation; normals from interpolated gradients | Lorensen and Cline 1987 | `src/render/aero/aero_isosurface.cpp`, `src/render/shaders/aero/compute/aero_vis_marching_cubes.comp.glsl` | surfaces of radius 6 cells or more | V4 area within 2 percent; V5 first order at the boundary | `tests/render/aero/test_marching_cubes.cpp` |
| `render.aero_surface` | per-panel Cp averaged to the vertices (angle-weighted); studio shading with a camera-attached key light | project display model | `src/render/aero/aero_surface.cpp`, `src/render/shaders/aero/aero_surface.frag.glsl` | display only, loads unchanged | V8, V11 | `tests/render/aero/test_aero_view.cpp` |
| `render.aero_colormap` | maps linear in CIE L* with hue and chroma in Oklab; display pre-compensation by the inverse ACES fit | Ottosson 2020 (Oklab); CIE 1976 L* | `src/render/aero/aero_colormap.cpp` | colours with a non-negative pre-image | V9 exact to 8-bit rounding | `tests/render/aero/test_aero_colormap.cpp`: Oklab step ratio 1.049 (bound 1.1) |
| `app.soi_primary` | primary of a body: the least massive heavier body whose sphere of influence r (m / M_heaviest)^(2/5) contains it, else the heaviest body | Laplace sphere of influence | `src/render/ui_panels.cpp` | inspector display | sphere radius from the distance to the heaviest body | `tests/render/test_ui.cpp`: Moon around Earth, planets around the Sun |
| `app.framing` | Orbit camera at 72 / 30 degrees around the dominant body, framed radius the median distance of the others; black hole framed for its 30 M disk | project display model | `src/app/space_view.cpp`, `src/app/include/orrery/app/space_view.h` | composition only | framing heuristic, not a physical quantity | `tests/app/test_headless.cpp` |
| `app.entry_summary` | downrange = R times the great-circle angle between the first and the current ground point (spherical law of cosines); ballistic coefficient beta = m / (Cd_max A) with the largest tabulated Cd | standard spherical trigonometry | `src/app/entry/entry_backend.cpp`, `src/app/entry/entry_model.cpp` | display and comparison table | spherical planet (E4); acos loses about 1e-8 rad (0.1 m on Earth) for tiny angles | `tests/app/test_entry_model.cpp` |
| `app.step_budget` | integral nanosecond accumulator; at most `max_steps_per_frame` (default 4096) steps per tick, the rest dropped and reported; the warp governor lowers the credited warp when the physics load exceeds its share | ADR-016 | `src/app/sim/simulation.cpp`, `src/app/sim/warp_governor.cpp` | normal time warp | A17 simulated time lags under extreme warp | `tests/app/test_simulation.cpp`, `tests/app/test_warp_governor.cpp` |

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
- Diagnostics: osculating Kepler elements about the primary chosen by the Laplace sphere of influence (`app.soi_primary`), energy, momentum and angular momentum drift. Hill spheres and ghost trajectories (spec section 3.6) are not implemented in v1.

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

- Atmosphere models: US Standard Atmosphere 1976 (the standard's layered formulation to 86 km, piecewise exponential through its tabulated values above), a simple Mars profile (NASA Glenn temperature fit, hydrostatic pressure), and a generic exponential rho = rho0 exp(-h/H).
- Drag on a probe: F = 1/2 rho v^2 Cd(Mach) A, with Cd(Mach) sampled from the aero solver at several Mach numbers and linearly interpolated, clamped outside the sampled range (ADR-012).
- Stagnation-point heat flux: Sutton-Graves correlation, q proportional to sqrt(rho / R_nose) v^3. Indicative only.
- Entry runs as a chain core state -> atmosphere (density, Mach) -> drag force -> integrator (ADR-014), drag applied as a split-step velocity kick around the gravity step (ADR-021). Validation: mechanical (kinetic plus potential) energy loss equals the work done by drag within 1 percent.
- Outputs: altitude, speed, deceleration (g load), heat flux time series, ground track; outcomes are impact, skip-out or the time limit. Details in docs/atmosphere.md and "Atmosphere and entry (orrery_atmo, Phase 9a)" below.

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
| A13 | Schwarzschild metric only; thin-disk emission | Black hole rendering, ADR-003, ADR-017 (details R1 to R11 in "Black hole rendering") | Non-rotating hole | Spinning holes (Kerr) have different shadow and disk appearance |
| A14 | Plummer softening | N-body force | Used to bound close-encounter forces | Alters dynamics at separations comparable to eps |
| A15 | Barnes-Hut multipole truncation | N-body force | Controlled by opening angle theta | Larger theta increases acceleration error |
| A16 | Entry-view atmosphere: single-colour exponential haze ray-marched in 16 samples; the Rayleigh + Mie scattering of spec section 7.2 is not implemented, and planets in the space view have no atmosphere | Rendering (`render.entry_view`) | Visual cue of the atmosphere shell only | Not a scattering or radiative transfer solution; colours and optical depth are not physical |
| A17 | Step budget per tick (`max_steps_per_frame`, default 4096) and a time-warp governor; simulated time beyond the budget is dropped and reported (ADR-016) | Interactive space simulation (`app.step_budget`) | Normal time warp | Extreme time warp or many IAS15 substeps make simulated time lag wall time (shown as a limited warp); headless runs use the count budget only, so results stay deterministic |

## 6. Validation set

Located in `tests/aero_validation/`. Each case cites its literature reference in the test source.

| Case | Solver | Expected | Tolerance / range |
|---|---|---|---|
| Circular cylinder, Re 100 to 200 | LBM 2D | Strouhal number, drag coefficient | St 0.16 to 0.20; Cd 1.2 to 1.5 |
| Sphere, Re 10 to 1000 | LBM 3D | Cd versus Re follows Schiller-Naumann | within 15 percent |
| NACA 0012, angle of attack 0 to 8 degrees | Panel | Lift slope near 2 pi per radian; embedded reference table from thin-airfoil and thick-section (Joukowski) theory, no XFOIL data | within 10 percent |
| Flat plate normal to flow | LBM | Cd about 1.28 | within 15 percent |
| Apollo-type capsule, Mach 20 | Newtonian | Cd about 1.3 (published capsule data) | within 10 percent |
| User-supplied model | any | No validation | UI shows an "unvalidated geometry" badge |

Related project-level criteria: sphere Cd at Re 100 between 1.0 and 1.2; cylinder Strouhal number at Re 150 between 0.16 and 0.20; NACA 0012 lift slope 2 pi per radian within 10 percent.

Core validation (analytic): two-body analytic orbits, energy drift, Barnes-Hut versus direct, IAS15 figure-eight orbit, ISCO stability, determinism hash, Kepler element round trip.

## Lattice Boltzmann 2D (orrery_aero, Phase 7b)

Method, not model: the D2Q9 BGK lattice Boltzmann scheme recovers the
incompressible Navier-Stokes equations in the low Mach number limit. Full
reference in `lbm.md`; decisions ADR-008, ADR-006 and ADR-019; sources
`src/aero/lbm` and `src/render/shaders/lbm`; tests `tests/aero/lbm` and
`tests/aero_validation/lbm2d`.

### Formulas
- Lattice units dx = dt = 1, rho_0 = 1, c_s^2 = 1/3; D2Q9 weights 4/9, 1/9,
  1/36 (Qian, d'Humieres and Lallemand 1992).
- f_eq,i = w_i rho (1 + 3 c_i.u + 9/2 (c_i.u)^2 - 3/2 u.u); BGK
  f*_i = f_i - (f_i - f_eq,i) / tau; nu = (tau - 1/2) / 3 (Kruger et al.,
  *The Lattice Boltzmann Method*, 2017).
- Guo forcing (Guo, Zheng and Shi 2002): u = (sum c f + F/2) / rho,
  F_i = (1 - 1/(2 tau)) w_i [3 (c_i - u).F + 9 (c_i.u)(c_i.F)].
- Smagorinsky (Hou et al. 1996): tau_eff = (tau + sqrt(tau^2 + 18 sqrt(2)
  C^2 |Pi_neq| / rho)) / 2.
- Halfway bounce-back f_i(x) = f*_opp(i)(x); moving wall adds
  6 w_i rho_w c_i.u_w; free slip by specular reflection.
- Zou-He velocity inlet (Zou and He 1997), regularized (Latt et al. 2008):
  f_i = f_eq,i + w_i / (2 c_s^4) (c_i c_i - c_s^2 I) : Pi_neq.
- Convective outlet (Yang 2013): f(t+1) = (f(t) + U f_upstream(t+1)) /
  (1 + U); or zero-gradient extrapolation.
- Momentum exchange (Ladd 1994): F = sum over fluid-solid links of
  2 c_o f*_o, o pointing into the wall; Cd = F_x / (rho U^2 L / 2).
- Units: dx = L / N, dt = u_lat dx / U, nu_lat = nu dt / dx^2,
  tau = 3 nu_lat + 1/2; Re identical in both systems; lattice Mach
  u_lat sqrt(3) < 0.1 enforced (A3).

### Measured (MSVC 19.44 x64 Release, /fp:precise; GPU AMD Radeon RX 9060 XT)

| Criterion | Setup | Measured | Bound |
|---|---|---|---|
| Cylinder Re 100, CPU | D 20, 26000 steps (lbm.md section 8) | St 0.1674, mean Cd 1.468 | St 0.16 to 0.20, Cd 1.2 to 1.5 |
| Cylinder Re 100, GPU | same | St 0.1674, mean Cd 1.468 | same |
| GPU vs CPU | Poiseuille, Couette, Taylor-Green, cylinder with and without LES | velocity L2 at most 6.1e-5, drag 5.3e-5 | 1e-4 |
| GPU vs CPU, shedding onset | 400 x 200 cylinder, no start perturbation | velocity L2 up to 7.1e-4 at steps 10000 to 18000, 9e-5 after lock-in | not met during the transient (lbm.md, "GPU precision") |
| Poiseuille | tau = 1/2 + sqrt(3/16) / 0.8 / 0.6, H 32 | 5e-13 / 7e-4 / 1.3e-3 relative L2 | 1e-9 / 2e-3 / 2e-3 |
| Couette | moving wall, H 16 | 3e-13 | 1e-9 |
| Taylor-Green decay | 64 x 64, tau 0.65 | rate within 0.08 percent of 4 nu k^2 | 1 percent |
| Mass conservation | periodic, slip, moving no-slip, 1500 steps | below 1e-12 relative | 1e-12 |
| Momentum exchange | channel with a solid row, body force | equals the total body force to 1e-6 | 1e-6 |
| GPU throughput | 2048 x 1024, BGK / with Smagorinsky | 1117 / 1113 steps/s | 200 steps/s |
| CPU throughput | 600 x 400, 12 threads | 108 million updates/s | none |

### Approximations (LBM 2D)
A2 (Smagorinsky), A3 (weak compressibility) and A4 (staircase walls) in
section 5 apply.

| ID | Approximation | Where | Validity envelope | Outside the envelope |
|---|---|---|---|---|
| L1 | Zou-He inlet node regularized (higher-order non-equilibrium dropped) | `zou_he_west`, `lbm_zou_he_west` | Smooth inflow; keeps rho, u and Pi_neq of the Zou-He set | Strong gradients at the inlet are smoothed; keep the body 5 or more lengths downstream |
| L2 | Convective outlet with the inlet speed as convection speed | `Outlet::convective` | Wakes convected at about the free-stream speed | Recirculation at the outlet; the mean density drifts slowly (0.1 percent over 100000 steps at D 40) |
| L3 | Moving no-slip wall: momentum correction scaled by the density of the fluid node next to the wall | `resolve_source` (wall) | Wall-parallel motion, smooth density near the wall | Strong density jumps across the wall-adjacent layer (compressibility error of order Ma^2) |
| L4 | Smagorinsky Pi_neq taken without the forcing correction | `collide`, `lbm_collide` | Weak body forces | Strongly forced LES runs |
| L5 | GPU populations in binary32, shifted by the weights | `lbm2d_gpu.cpp`, `src/render/shaders/lbm` | Agreement with the CPU within 1e-4 on laminar and locked-in periodic flow | Instability transients amplify rounding (up to 7e-4 measured) |
| L6 | Momentum exchange for stationary bodies, force applied at the link midpoints | `add_link_force` | Bodies at rest | Moving bodies need the wall-velocity terms |
| L7 | Wall pressure Cp from the node next to the wall against the mean density of the column next to the inlet; one sample per node with area dx | `LbmAeroSolver2D::update_result` | Qualitative Cp distribution | First-order in the resolution, like A4 |
| L8 | 2D solver ignores sideslip; velocity field reported only at zero angle of attack | `LbmAeroSolver2D` | Plane sections | `lbm.sideslip` warning; field absent when the lattice is wind-aligned |

## Lattice Boltzmann 3D (orrery_aero, Phase 7c)

Method, not model: the D3Q19 BGK scheme recovers the incompressible
Navier-Stokes equations in the low Mach number limit. Full reference in
`lbm.md` (sections 10 to 15); decisions ADR-008, ADR-019 and ADR-020;
sources `src/aero/lbm3d` and `src/render/shaders/lbm/lbm_d3q19_*`; tests
`tests/aero/lbm3d` and `tests/aero_validation/lbm3d`.

### Formulas
- D3Q19 (Qian, d'Humieres and Lallemand 1992): rest, 6 axis and 12 face
  diagonal directions, weights 1/3, 1/18, 1/36, c_s^2 = 1/3; equilibrium,
  BGK, Guo forcing, Smagorinsky relaxation time and lattice units as in
  "Lattice Boltzmann 2D" (|Pi_neq| over six components).
- Velocity inlet: rho = (sum_{c_x=0} f + 2 sum_{c_x=-1} f) / (1 - u_x) (Zou
  and He 1997), unknowns f_i = f_opp(i) + 6 w_i rho c_i.u, then regularized
  (Latt et al. 2008).
- Interpolated bounce-back (Bouzidi, Firdaouss and Lallemand 2001), wall
  fraction q = d(x) / (d(x) - d(x - c_j)) from the signed distance:
  q < 1/2: f_j = 2q f*_o(x) + (1 - 2q) f*_o(x + c_j);
  q >= 1/2: f_j = f*_o(x) / (2q) + (2q - 1)/(2q) f*_j(x).
- Momentum exchange F = sum over links of c_o (f*_o(x) + f_j(x, t+1))
  (Ladd 1994; Mei, Yu, Shyy and Luo 2002), moment at x - q c_j; a lattice
  force unit is rho_ref dx^4 / dt^2.
- Absorbing layer (Xu and Sagaut 2013): f* <- f* - sigma (f* - f_eq(1,
  u_in)), sigma = s ((T - d) / T)^2.
- Gradual start: u_in(t) = U (1 - cos(pi t / T)) / 2 for t < T.
- Sphere reference: Cd = 24 / Re (1 + 0.15 Re^0.687) (Schiller and Naumann
  1933).

### Measured (MSVC 19.44 x64 Release, /fp:precise; GPU AMD Radeon RX 9060 XT)

| Criterion | Setup | Measured | Bound |
|---|---|---|---|
| Sphere Re 10, GPU | D 12, channel 10 D wide (lbm.md section 14) | Cd 4.415 (correlation 4.151, ratio 1.064) | within 15 percent |
| Sphere Re 100, CPU | D 12, 121 x 61 x 61, 6000 steps | Cd 1.186 (1.092, ratio 1.086), steady, 27 s | within 15 percent |
| Sphere Re 100, GPU | D 16 | Cd 1.140 (ratio 1.044), steady | within 15 percent |
| Sphere Re 300, GPU | D 16 | Cd 0.693 (0.684, ratio 1.013) | within 15 percent |
| Sphere Re 1000, GPU | D 48 | Cd 0.482 (0.438, ratio 1.099) | within 15 percent |
| Square plate normal to the flow, Re 1000, GPU | 24 cells per side | Cd 1.158 (ratio 0.905 to 1.28) | within 15 percent of 1.28 |
| GPU vs CPU | Poiseuille, duct, Couette, sphere with BGK, Smagorinsky, interpolated wall, extrapolation outlet | velocity L2 at most 1.6e-5, drag at most 1.3e-5 | 1e-4 |
| Poiseuille | tau = 1/2 + sqrt(3/16) / 0.8, H 24 | 7.3e-13 / 1.2e-3 relative L2 | 1e-9 / 2e-3 |
| Square duct | 20 x 20, series solution | 5.5e-4 relative L2 | 1e-3 |
| Couette | moving wall, H 16 | 1.0e-13 | 1e-9 |
| Mass conservation | periodic, slip, moving no-slip, sphere, 1500 steps | 1.4e-13 relative | 1e-12 |
| Momentum exchange | solid layer, body force | equals the total body force to 1e-6 | 1e-6 |
| GPU throughput | 128^3 sphere channel, BGK / with Smagorinsky | 652 / 648 steps/s | 60 steps/s (GTX 1660 class, not measured) |
| CPU throughput | 121 x 61 x 61, 12 threads | about 100 million updates/s | none |

### Approximations (LBM 3D)
A2 (Smagorinsky), A3 (weak compressibility) and A4 (staircase walls, for
`SolidWall::halfway` and for the node classification) in section 5 apply,
and L2, L4 and L5 of "Lattice Boltzmann 2D" hold unchanged in 3D.

| ID | Approximation | Where | Validity envelope | Outside the envelope |
|---|---|---|---|---|
| D1 | Inlet node regularized; no tangential Zou-He corrections (Hecht and Harting 2010) | `velocity_inlet_west`, `lbm3_inlet_west` | Smooth inflow; density and velocity are exact after regularization | Strong gradients at the inlet are smoothed; keep the body 2 or more lengths downstream |
| D2 | Interpolated wall from the linear interpolation of the signed distance along each link; q < 1/2 without a second fluid node falls back to halfway; not exactly mass conserving | `link_fraction`, `reflected`, `lbm3_reflected` | Walls resolved by a few cells (the distance is exact within three cells) | Thin gaps and sharp corners revert to the staircase; mass drifts slowly at walls |
| D3 | Absorbing layer relaxing to the free-stream equilibrium before the outlet (2 lengths, strength 0.1) | `Sponge`, `sponge_sigma`, `lbm3_sponge` | Wake and pressure waves leaving through the outlet; fixes the pressure level | Bodies close to the layer feel it; keep 3 or more lengths between them |
| D4 | Gradual start from rest over 3 convective times | `LbmSolver3DOptions::start_ramp` | Steady or statistically stationary results | The start-up transient differs from an impulsive start |
| D5 | Momentum exchange for stationary bodies | `add_link_force` | Bodies at rest | Moving bodies need the wall-velocity terms |
| D6 | Channel with free-slip or periodic sides of finite width | `LbmAeroSolver3D` | Blockage below 5 percent (`lbm.blockage`) | Confinement raises the drag (sphere Re 10 in a 6 D channel: 9.6 percent above the correlation) |
| D7 | Wall samples: one per fluid node next to the body, an equal share of the mesh area, Cp against the plane next to the inlet | `LbmAeroSolver3D::update_result` | Qualitative Cp distribution | First order in the resolution |
| D8 | Velocity field reported only at zero angle of attack and sideslip | `LbmAeroSolver3D` | Wind-aligned lattice equal to body axes | Field absent when the lattice is rotated |

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

- The first four hashes are pinned by tests/core/test_determinism.cpp, the IAS15 hash by test_ias15.cpp and the Barnes-Hut hash by test_barnes_hut.cpp (every thread count must give the same pinned value). Adding the radius word (Phase 3b; radius 0 for every golden body) changed every value; the trajectories did not change, since neither the integrators nor the force models of the golden runs were modified (the kind-tag format before it gave 0xaf61005ef03001b2, 0xe0fa7b48f95f973e, 0x9edf045b7f8169ad, 0xec2fbc333855be6d). Each new golden test was checked to fail under a deliberate deterministic mutation: reordering one Barnes-Hut multipole accumulation (`ax += radial * rx + tensor * qrx` evaluated as `(ax + tensor * qrx) + radial * rx`) and truncating the last IAS15 Gauss-Radau spacing (h_7 to 0.977520613561287); the other IAS15 and Barnes-Hut tests, including the run-to-run and thread-count comparisons, still pass under these mutations. The golden uses 10-day outputs because with 1-day outputs every internal step is clipped to the output and the step controller never acts. Two other truncations left the hash unchanged (below rounding in this scenario, not a coverage gap of the hash): `d_coef[0][3]`, which only seeds the corrector iteration, and `c_coef[0][2]` (3e-15 relative).
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
- `sgr_a_s2.json`: Sgr A* (black-hole kind, spin 0) and S2 with the GRAVITY Collaboration 2020 best fit (A&A 636, L5, Table E.1): M = 4.261e6 Msun, R0 = 8246.7 pc, a = 125.058 mas, e = 0.884649, i = 134.567 deg, omega = 66.263 deg, Omega = 228.171 deg, P = 16.0455 yr; a = 0.125058 x 8246.7 = 1031.3158 AU. Starts at apocentre. For comparison, Gillessen et al. 2017 (ApJ 837, 30): a = 0.1255 arcsec, e = 0.884, P = 16.00 yr.
- Force model: the scenario file is titled "Sgr A* and the star S2 (Newtonian)" and its metadata describe the Newtonian point-mass model its orbit elements and the measurement below use. The bundled experiment `assets/experiments/sgr_a_s2.json` runs it with `"force": "auto"`, which selects Paczynski-Wiita because the scene holds a black hole (the simulation reports `paczynski_wiita`, checked by `tests/app/test_simulation.cpp`). At S2's pericentre (about 119 AU; r_s = 0.084 AU) the PW force exceeds the Newtonian one by about 2 r_s / r = 1.4e-3, and the potential advances the pericentre by about 4 pi G M / (c^2 a (1 - e^2)) per orbit to leading order in r_s / r: two thirds of the Schwarzschild value, not the GRAVITY measurement. To run the Newtonian model, set `"force": "newtonian"` in a copy of the experiment.
- APPROXIMATION (S2 scenario): point masses (Newtonian in the scenario's model and the period test, Paczynski-Wiita in the bundled experiment); S2 is a zero-mass test star (its mass is of order 1e-5 of the hole's), and the Schwarzschild precession and Roemer delay measured by GRAVITY are not modelled. Valid for the orbital period and shape, not for the precession or the redshift.
  Measured (Newtonian force, `tests/core/test_scenarios_phase3a.cpp`): period from successive pericentre passages (sign change of r.v, bisected) 16.045008 Julian years for both orbits; Kepler 2 pi sqrt(a^3 / (k^2 M)) = 16.045008 yr (relative difference 9e-15); GRAVITY fit 16.0455 yr. Acceptance 16.0 +- 0.3 yr.

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
| Hostile input | NaN/inf coordinates, out-of-range indices, STL/PLY/FBX headers declaring billions of elements, a PLY comment naming `end_header` before the counts, 16 GB deflate claim, oversized files, referenced files over the byte budget, references outside the model directory | all rejected before parsing or before post-processing | rejected |
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
| G7 | Imported coordinates and node transforms parsed in single precision (the vcpkg assimp port has no `ASSIMP_DOUBLE_PRECISION` build), widened to double afterwards | `import_mesh_file`, `import_mesh_memory` | Detail above about 1e-7 of the model extent (float resolution 6e-8 relative); every later stage is double | Coordinates closer than float resolution (`1.0` and `1.000000001`) collapse to one vertex; a double-precision assimp build is needed for such models |

## Aerodynamic solvers (orrery_aero, Phase 7a)

Sources: `orrery/aero/aero_solver.h` (interface), `flow_conditions.h`,
`aero_result.h`, `regime.h`, `panel_method.h`, `newtonian.h`; overview and
validity envelopes in `aero.md`; decision ADR-009; tests in `tests/aero`
and `tests/aero_validation`.

### Flow conditions and coefficients
- Speed of sound a = sqrt(gamma R T), Mach = V / a, Re = rho V L / mu,
  q = rho V^2 / 2; air defaults gamma = 1.4, R = 287.05 J/(kg K), sea-level
  rho, T and mu of the US Standard Atmosphere 1976.
- Body axes x downstream, y span, z up. Free-stream (drag) direction
  (cos a cos b, sin b, sin a cos b), lift direction normalize(drag x y) =
  (-sin a, 0, cos a) at b = 0, side direction lift x drag.
- Cd, Cl, Cy = force . direction / (q S); Cm = M_y / (q S l) about the
  moment centre, nose up positive.

### Panel method (Hess and Smith 1967; Katz and Plotkin, *Low-Speed Aerodynamics*, 2001)
- 2D: constant source sigma_j on each panel and one vorticity gamma on all
  panels. In panel coordinates (xi along the panel, eta normal) a unit source
  induces u = ln(r1/r2) / (2 pi), v = beta / (2 pi) and a unit vortex
  u = -beta / (2 pi), v = ln(r1/r2) / (2 pi), beta the angle the panel
  subtends. Unknowns from zero normal velocity at the panel midpoints plus the
  Kutta condition V_t,1 + V_t,N = 0 at the trailing edge; Cp = 1 - (V_t/V)^2.
  Circulation Gamma = -gamma * perimeter * V gives the Kutta-Joukowski lift
  rho V Gamma, which the tests compare with the pressure integral.
- 3D: on each triangle a constant source sigma = -n . V and an unknown
  constant doublet mu; the perturbation potential inside the body is zero
  (Dirichlet condition, Katz and Plotkin eq. 12.12):
  sum_j mu_j C_ij + sum_w mu_w C_iw + sum_j sigma_j B_ij = 0, C_ii = -1/2.
  Doublet influence -Omega / (4 pi) from the solid angle (van Oosterom and
  Strackee 1983); source influence -(1 / 4 pi) [sum_k d_k ln((r_k + r_k+1 +
  l_k) / (r_k + r_k+1 - l_k)) - |h| |Omega|] (edge-logarithm form), both
  checked against brute-force quadrature. mu is the surface perturbation
  potential, so the surface velocity is V_tangential + grad mu, with grad mu
  from weighted least squares over two rings of neighbours that do not
  cross the trailing edge.
- Kutta condition: mu_w = mu_upper - mu_lower at the trailing edge (upper
  and lower values: mean of the panels at each trailing-edge vertex,
  extrapolated to the vertex with their gradients).
- Trefftz plane: wake trace with point vortices -mu, +mu at each segment's
  ends; CL_T = 2 sum mu_k ds_k / S and CDi = -sum mu_k w_k ds_k / S (w the
  normal velocity at the segment midpoints, per unit free-stream speed).
- Linear solve: dense partial-pivot LU up to 4000 unknowns, restarted GMRES
  (Eigen) above; the relative residual is reported.
- Flat-plate friction (Schlichting, *Boundary-Layer Theory*): Cf = 1.328 /
  sqrt(Re) for Re <= 5e5, else 0.455 / (log10 Re)^2.58 - 1700 / Re;
  Cd_friction = Cf S_wetted / S_ref, reported separately from Cd.

### Modified Newtonian theory (Lees 1955; Anderson, *Hypersonic and High-Temperature Gas Dynamics*)
- Cp = Cp_max sin^2(theta), sin theta = -n . V_hat on windward faces, 0 in the
  shadow; force -sum Cp q A n.
- Cp_max = 2 / (gamma M^2) (p02/p1 - 1) with the Rayleigh pitot formula
  p02/p1 = [(g+1)^2 M^2 / (4 g M^2 - 2 (g-1))]^(g/(g-1)) (1 - g + 2 g M^2) / (g+1)
  for M >= 1 (isentropic stagnation below); 1.839 for gamma = 1.4 as M -> inf.
- Sutton-Graves (NASA TR R-376, 1971): q_s = k sqrt(rho / R_n) V^3,
  k = 1.7415e-4 kg^0.5/m for Earth air (configurable).

### Measured (MSVC 19.44 x64 Release, /fp:precise)

| Criterion | Setup | Measured | Bound |
|---|---|---|---|
| NACA 0012 lift slope, 2D Hess-Smith | 200 panels, alpha 0, 2, 4, 6, 8 degrees, least-squares slope through the origin | 6.897 per rad = 1.098 x 2 pi; 1.005 x the Joukowski thick-section slope 2 pi (1 + 0.77 t/c) | 2 pi within 10 percent (acceptance); thick-section slope within 2 percent |
| NACA 0012 Cl table | as above, against the embedded thin-airfoil and thick-section values | Cl 0.2413, 0.4823, 0.7227, 0.9622 at 2, 4, 6, 8 degrees; at most 0.7 percent from the thick-section values; inviscid pressure Cd 1.4e-4 to 2.1e-4 (discretisation error) | 3 percent |
| Symmetric section at zero incidence | NACA 0012, 160 panels | Cl 1e-16, Cm 5e-17; Cl at 4 degrees equals Kutta-Joukowski rho V Gamma within 1 percent | 1e-10 |
| Circular cylinder, 2D | 128 panels | worst error of Cp against 1 - 4 sin^2(theta) 1.1e-14 at the panel midpoints | 1e-10 |
| Sphere, 3D | 480 and 1984 triangles | Cp against 1 - 9/4 sin^2(theta): rms 0.052 and 0.026 (first order), worst 0.066; Cd, Cl below 0.01 | rms 0.03, worst 0.08 |
| Rectangular NACA 0012 wing, AR 6, 5 degrees | 16 stations, 24 strips, 1596 panels | CL 0.3915 (Helmbold with the 2D slope 0.421, 7 percent lower); Trefftz CL 0.3920; CDi 0.00823, e = 0.988; at 0 degrees CL 6e-6, CDi 2.5e-4 (P3) | CL within 10 percent, Trefftz within 2 percent, e 0.85 to 1.05 |
| GMRES against dense LU | sphere, 480 panels, 3 degrees | doublet strengths agree to 3.8e-12 | 1e-8 |
| Triangle influence against quadrature | 5 points around a triangle, 16384-cell centroid rule | below 1e-4 relative (quadrature error) | 1e-4 |
| 3D panel solve time | sphere, 2048 panels, dense LU | 0.69 s (hidden `[benchmark]`) | 3 s (project criterion) |
| Newtonian sphere | 40k triangles, Mach 10 | Cd 0.915775 against Cp_max / 2 = 0.915835 (6.6e-5 relative) | 2e-3 |
| Newtonian flat plate | Mach 8, alpha 10, 30, 90 degrees | Cd, Cl equal Cp_max sin^2(a) (sin a, cos a) to 1e-8 | 1e-8 |
| Apollo command module, Mach 20 | capsule primitive (`CapsuleShape::apollo`), 128 x 0.01 rad, heat shield forward | Cd 1.600, 1.539, 1.367, 1.250 at 0, 10, 20, 25 degrees; at the 20 degree trim attitude Cd 1.367 (5 percent above 1.3) and L/D -0.303; Sutton-Graves q_s 35 W/cm^2 at 60 km | Cd 1.3 within 10 percent (acceptance); abs(L/D) 0.3 within 0.05 |
| Newtonian time | sphere, 100352 triangles | 3.4 ms (hidden `[benchmark]`) | 100 ms (project criterion) |
| Validity warnings | panel 2D at Mach 0.59, 14 degrees, sideslip 2 degrees; Newtonian at Mach 3; selector override to Newtonian at Mach 0.15 | `panel.mach`, `panel.angle_of_attack`, `panel.sideslip_ignored`; `newtonian.mach`; `newtonian.mach` with the result still computed | every warning fires, nothing blocks |

### Approximations (aero solvers)
Every item is marked `// APPROXIMATION:` in the code. A5 (potential flow,
fixed wake), A6 (flat-plate friction), A7 (Newtonian impact) and A8
(Sutton-Graves heating) in section 5 apply.

| ID | Approximation | Where | Validity envelope | Outside the envelope |
|---|---|---|---|---|
| P1 | Panels beyond 10 diameters act as a point source and point doublet | `PanelSolver3D::step` | Neglected terms of relative order 1e-2 of influences that are themselves 1e-2 of the near field | Not used closer than 10 diameters |
| P2 | Wake strips flat and straight along the free stream, 50 reference lengths long | `PanelSolver3D::setup` (A5) | Small angle of attack, wake roll-up negligible for the loads | High lift or close wake-body interaction |
| P3 | Kutta jump from the mean of the panels at each trailing-edge vertex, extrapolated linearly | `kutta_jumps` | Triangulations symmetric between upper and lower surface give the exact discrete jump; the diagonal split of `naca_wing` leaves CDi = 2.5e-4 at zero incidence (AR 6, 16 stations) | Very thin trailing-edge panels on an unsymmetric triangulation: spurious antisymmetric circulation at the tips (CDi = 2e-3 at 48 stations) |
| P4 | Surface velocity from a least-squares doublet gradient over two rings of neighbours, sharp edges (normals over 45 degrees apart) used only as a fallback | `gradient_weights` | Smooth surfaces; first-order accurate | Cp next to sharp edges (wing tips, capsule shoulders) is smeared over one panel |
| P5 | 2D panel method ignores sideslip | `PanelSolver2D::step` | Sections in a plane flow | Swept or yawed flow; a `panel.sideslip_ignored` warning is attached |
| P6 | Regime boundaries Mach 0.3 (incompressible), Mach 5 (hypersonic), Re 1e5 (thin attached boundary layer), 10 degrees (attached flow) | `validity_envelope`, `select_solver` | Engineering rules of thumb, not sharp limits | Results near a boundary deserve the same caution as those just outside |

## Aero visualisation (orrery_render, Phase 8a)

Display of the aerodynamic results (docs/aero-visualisation.md). Nothing
here feeds back into a solver; the approximations bound what the pictures
show, not the computed loads.

### Approximations (aero visualisation)
Every item is marked `// APPROXIMATION:` in the code.

| ID | Approximation | Where | Validity envelope | Outside the envelope |
|---|---|---|---|---|
| V1 | Trilinear (2D: bilinear) interpolation of nodal fields | `sample` (aero_field.cpp), `flow_velocity` (aero_vis_flow_common.glsl), section texture filtering | Exact for linear fields, second order otherwise | Structures below two grid spacings are smoothed away |
| V2 | Streamlines by classical RK4 in arc length, fixed step (0.5 grid spacings by default); stop below 1e-3 of the peak speed | `trace_streamlines`, `aero_vis_flow_streamlines` | Fourth order in the step; 2.7e-8 m radius drift over 400 steps on a circular flow | Near stagnation points and through regions thinner than a step the line ends early; binary32 on the GPU (agrees with the CPU to 1e-4 m) |
| V3 | Particles by the explicit midpoint rule in time on the field of the current frame, sub-stepped to half a grid spacing | `advect_particle`, `aero_vis_flow_particles` | Second order; pathlines while the field changes little between frames | In fast unsteady flow the traces are a visual aid, not pathlines |
| V4 | Marching cubes with linear edge interpolation; face ambiguity by separating inside corners, interior ambiguity ignored | `marching_cubes`, `aero_vis_marching_cubes` | Closed, consistently oriented surfaces; area within 2 percent for a sphere of radius 6 cells or more | Topology near saddle points can differ from the trilinear interpolant's |
| V5 | Iso-surface normals from interpolated central-difference gradients | `vertex_normal` (aero_isosurface.cpp) | Smooth shading of resolved surfaces | First order at the grid boundary |
| V6 | Vorticity and Q by finite differences, central inside the fluid, one-sided at grid edges and walls | `vorticity`, `q_criterion` (aero_field.cpp) | Second order inside, first order next to walls | Wall vorticity is smeared over one cell |
| V7 | Section planes sampled at grid-node resolution in the plane, linearly along the normal; solid flag of the nearer node | `sample_section` | As sharp as the grid | Lattice bodies show their staircase unless a mesh covers them |
| V8 | Per-panel Cp averaged to the vertices (angle-weighted) for smooth colouring | `vertex_values` (aero_surface.cpp) | Display only; integrated loads unchanged | Across sharp edges the two sides share their vertices' averaged values |
| V9 | Display pre-compensation by the inverse of the ACES fit | `scene_linear_for_display` (aero_colormap.cpp) | Colours with a non-negative pre-image (all map colours): exact to 8-bit rounding | More saturated colours cannot be shown through the pipeline and are not used |
| V10 | LBM pressure as Cp from the weakly compressible lattice pressure c_s^2 (rho - 1), rho_ref = 1 | `flow_field_from` (aero_field.cpp) | Lattice Mach below 0.1 (A3) | Acoustic noise of the solver appears in the pressure |
| V11 | Studio shading: camera-attached key light, wrapped Lambert with a 0.42 floor, weak highlight and rim; no shadows | aero_surface.frag.glsl | Colours equal the legend where the surface faces the key light | Shaded parts are darker than the legend colour |

## Atmosphere and entry (orrery_atmo, Phase 9a)

Atmosphere models, the Cd(Mach) bridge from the aero solvers, drag as a core
force model and the entry experiment (docs/atmosphere.md, ADR-012, ADR-014,
ADR-021). SI throughout.

### Formulas

| Quantity | Formula | Source |
|---|---|---|
| Geopotential altitude (US76) | H = r0 Z / (r0 + Z), r0 = 6 356 766 m | U.S. Standard Atmosphere 1976 (NOAA-S/T 76-1562) |
| US76 layer temperature | TM = TM,b + L (H - Hb); L = -6.5, 0, 1.0, 2.8, 0, -2.8, -2.0 K/km' from 0, 11, 20, 32, 47, 51, 71 km' | US76 Table 4 |
| US76 layer pressure | P = Pb (TM,b / TM)^(g0' M0 / (R* L)), or Pb exp(-g0' M0 (H - Hb) / (R* TM,b)) for L = 0; g0' = 9.80665, M0 = 28.9644 kg/kmol, R* = 8314.32 J/(kmol K) | US76, hydrostatic equation per layer |
| US76 density, sound speed, kinetic temperature | rho = P M0 / (R* TM), a = sqrt(1.4 R* TM / M0), T = TM M / M0 (M/M0 from Table 8 between 80 and 86 km) | US76 |
| US76 above 86 km | log-linear rho and P, linear T between Table I nodes (86 to 1000 km) | US76 Table I |
| Mars temperature | T = 242.15 K - 0.998 K/km h (h < 7 km), -2.22 K/km above, isothermal above 50 km | NASA Glenn Research Center Mars atmosphere fit (Mars Global Surveyor, 1996), joined continuously |
| Mars pressure | hydrostatic through the temperature layers from 699 Pa, R = 192.1 J/(kg K), g0 = 3.711 m/s^2, gamma = 1.29 | surface pressure and gas constant of the same fit |
| Exponential atmosphere | rho = rho0 exp(-h / H), p = rho R T | Allen and Eggers, NACA Report 1381 (1958) |
| Mach number | M = abs(v_rel) / a | |
| Relative wind | v_rel = v (inertial atmosphere) or v - omega z x r (co-rotating) | |
| Drag | F = -1/2 rho abs(v_rel) v_rel Cd(M) A | ADR-012 |
| Cd(Mach) | linear between samples, end values held outside | ADR-012 |
| g-load | abs(F) / (m g0), g0 = 9.80665 m/s^2 | |
| Stagnation heat flux | q = k sqrt(rho / R_n) abs(v_rel)^3, k = 1.7415e-4 (Earth), 1.9027e-4 (Mars) kg^0.5/m | Sutton and Graves, NASA TR R-376 (1971) |
| Split step | v += kick(dt/2); (r, v) = gravity step(dt); v += kick(dt/2); kick: RK4 on dv/dt = F(r, v) / m at fixed r | Strang (1968), ADR-021 |
| Energy balance | E = m (v^2 / 2 - GM / r); E(t) - E(0) = integral of F . v dt | work-energy theorem |
| Allen-Eggers peak (test reference) | n_max = V_E^2 sin(gamma) / (2 e H g0) at V = V_E exp(-1/2), rho = beta sin(gamma) / H | Allen and Eggers (1958) |

### Measured (MSVC 19.44 x64 Release, /fp:precise)

| Quantity | Setup | Measured | Criterion |
|---|---|---|---|
| US76 at the layer boundaries (and 5 km') | `sample_geopotential` against Table I | TM, P, rho, a equal to the printed values (half a unit of the last digit) | published precision |
| US76 base pressures | computed from the constants | 101325, 22632.06, 5474.889, 868.0187, 110.9063, 66.93887, 3.956420, 0.3733836 Pa | Table 4, seven figures |
| Apollo-like ballistic entry | 11.0 km/s, -5.7 degrees, Cd 1.3, 5560 kg, 12.02 m^2 | peak 6.78 g at 224.5 s, impact at 438.1 s, 1.47 MW/m^2 peak heat flux | 6 to 8 g |
| Energy balance | same | relative error 2.1e-5 (dt 0.5 s), 8.5e-7 (dt 0.1 s) | below 1 percent |
| Skip-out | -5.0 degrees | 1.07 g pass to 71.6 km, exits at 225 s, bound | leaves the atmosphere, event reported |
| Allen-Eggers | exponential, no gravity, 10 km/s, -60 degrees | 224.6 g against 225.6 g (-0.46 percent) | within 1.5 percent |
| Cd table from the solver | Apollo capsule, Newtonian, 20 degrees, Mach 5 to 25 | every point within 10 percent of 1.3, non-decreasing | Cd 1.3 within 10 percent |

### Approximations (atmosphere and entry)
Every item is marked `// APPROXIMATION:` in the code. A8 (Sutton-Graves), A9
(constant mass), A10 (no attitude dynamics) and A11 (tabulated Cd) in section
5 apply.

| ID | Approximation | Where | Validity envelope | Outside the envelope |
|---|---|---|---|---|
| E1 | US76 above 86 km by piecewise exponential interpolation of the tabulated values | `UsStandardAtmosphere1976::sample` | Exact at the Table I nodes (86, 90, 95, 100, 110, ... 1000 km); between nodes a smooth profile is reproduced to a few percent | Not the standard's diffusive-equilibrium solution; no solar-activity dependence (the real thermosphere varies by an order of magnitude) |
| E2 | Mars: one mean profile (Glenn temperature fit, isothermal above 50 km, hydrostatic pressure from 699 Pa) | `MarsAtmosphere` | Mean conditions; within 10 percent of the fit's pressure below 10 km | Season, latitude, local time and dust storms (density varies by a factor of 2 or more); use Mars-GRAM for mission work |
| E3 | Isothermal exponential atmosphere with an independent temperature | `ExponentialAtmosphere` | Analytic studies (Allen-Eggers); H = R T / g for hydrostatic consistency | Real profiles have a varying scale height |
| E4 | Spherical planet (mean radius), point-mass gravity, geocentric latitude | `Planet`, `EntryExperiment` | Entry dynamics (J2 is 1e-3 of g) | Altitudes differ from geodetic heights by up to about 14 km between equator and pole on Earth; no terrain |
| E5 | Cd held at the end values outside the sampled Mach range | `AeroCoefficientTable::lookup` | Within the sampled range; capsules whose Cd varies little below the lowest sample | A capsule table sampled at Mach 5 to 25 also drives the supersonic and subsonic tail, where real Cd differs by up to about 30 percent |
| E6 | Drag only, Cd a function of Mach only (no Reynolds or Knudsen dependence; continuum Cd in the rarefied upper atmosphere) | `drag_force` | Ballistic bodies below about 90 km, where the heating and deceleration pulses happen | No lift (Apollo's L/D 0.3 corridor control is absent), no free-molecular drag rise above about 100 km |
| E7 | Atmosphere at rest in the inertial frame by default; co-rotating option without winds | `AtmosphereFrame`, `air_relative_velocity` | Default: up to omega R = 465 m/s airspeed error at Earth's equator (4 percent at 11 km/s, about 6 percent in peak g) | Use `co_rotating` for ground-relative studies; winds are never modelled |
| E8 | Strang splitting of drag (RK4 kick at fixed position, in substeps of at most 0.05 of the drag time v_air / a_drag) and gravity | `EntryExperiment` (ADR-021) | Second order in dt; dt 0.5 s changes peak g by at most 1e-4 relative against dt 0.1 s; a stiff kick (quadratic drag at 1/60 s drag time over 0.25 s) matches v0 / (1 + k v0 t) within 1e-4; a non-finite drag stage fails the run | Gravity steps comparable to the deceleration time v / a (about 100 s at 7 g from 7 km/s; seconds in a low-altitude terminal descent): the kick stays exact for drag alone, but the drag-gravity coupling is then resolved only at the step |
| E9 | Event instants, the impact row and the final state at impact by linear interpolation within the step; a step whose ends lie on the same side of a level while both osculating two-body orbits put the extremum between them beyond it (a pass through the interface or the ground inside one step) is redone in halves down to 1 ms until an end shows the crossing | `detect_events`, `crossed_unseen` | Steps of 0.5 s (altitude curvature over a step is centimetres); a periapsis 7 km below the interface inside a 416 s step is found (entry and skip-out reported) | Long coast steps (up to `max_coast_step`) and the halved steps around such a pass: instants to a fraction of the step; a crossing hidden by drag that both osculating orbits miss, or shallower than the 1 ms halving resolves, is not reported |
| E10 | Heat flux from the airspeed and the continuum density at every altitude | `measure` (Sutton-Graves, A8) | Indicative peak values below about 80 km | Rarefied flow; radiative heating at lunar-return speeds (a large share of the stagnation heating there) |
