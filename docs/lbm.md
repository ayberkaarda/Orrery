# Lattice Boltzmann Solver (2D D2Q9, 3D D3Q19)

Reference for the Phase 7b lattice Boltzmann solver of `orrery_aero` and the
compute backend it runs on; the Phase 7c three-dimensional solver follows in
sections 10 to 15. Formulas and approximations are summarised in
`physics.md` ("Lattice Boltzmann 2D"); the decisions are ADR-008 (LBM, BGK,
Smagorinsky), ADR-006 (GLSL compute behind `IComputeBackend`) and ADR-019
(the backend's design). LBM is a numerical *method* that recovers the
incompressible Navier-Stokes *model* in the low Mach number limit; the
documentation keeps the two apart.

| Piece | Header | Source |
|---|---|---|
| Compute backend interface, `ScopedBuffer` | `orrery/aero/compute/compute_backend.h` | `src/aero/compute` |
| Host-memory backend for tests | `orrery/aero/compute/null_compute_backend.h` | `src/aero/compute` |
| binary32 packing (GPU boundary) | `orrery/aero/compute/float32_pack.h` | `src/aero/compute/float32_pack.cpp` |
| D2Q9 lattice | `orrery/aero/lbm/d2q9.h` | |
| Lattice <-> SI units, Mach guard | `orrery/aero/lbm/lattice_units.h` | `src/aero/lbm` |
| Solid masks | `orrery/aero/lbm/solid_mask.h` | `src/aero/lbm` |
| Solver (CPU and GPU) | `orrery/aero/lbm/lbm2d.h` | `lbm2d_cpu.cpp`, `lbm2d_gpu.cpp` |
| Force history, Strouhal number | `orrery/aero/lbm/force_history.h` | `src/aero/lbm` |
| `IAeroSolver` adapter | `orrery/aero/lbm/lbm_aero_solver.h` | `src/aero/lbm` |
| OpenGL backend | `orrery/render/gl_compute_backend.h` | `src/render/compute` |
| Kernels (GLSL 460) | | `src/render/shaders/lbm` |

## 1. Lattice and collision

Lattice units: dx = dt = 1, reference density 1. Directions 0 rest, 1 E,
2 N, 3 W, 4 S, 5 NE, 6 NW, 7 SW, 8 SE; weights 4/9, 1/9 (axes), 1/36
(diagonals); c_s^2 = 1/3.

- Equilibrium: f_eq,i = w_i rho (1 + 3 c_i.u + 9/2 (c_i.u)^2 - 3/2 u.u).
- BGK: f*_i = f_i - (f_i - f_eq,i) / tau, viscosity nu = (tau - 1/2) / 3.
- Body force (Guo, Zheng and Shi 2002): u = (sum c_i f_i + F/2) / rho and
  F_i = (1 - 1/(2 tau)) w_i [3 (c_i - u).F + 9 (c_i.u)(c_i.F)].
- Smagorinsky closure (Hou, Sterling, Chen and Doolen 1996), switched on by
  `Lbm2DConfig::smagorinsky`: Pi_neq = sum c_i c_i (f_i - f_eq,i),
  Q = sqrt(Pi_neq : Pi_neq), tau_eff = (tau + sqrt(tau^2 + 18 sqrt(2) C^2 Q /
  rho)) / 2, i.e. nu_eff = nu + (C dx)^2 |S|. Default C = 0.1.
  APPROXIMATION A2: an eddy-viscosity model, not resolved turbulence; the
  `IAeroSolver` adapter enables it above Re 2000.

One fused kernel per time step: every fluid node pulls its post-streaming
populations from its neighbours (applying the boundary rules below while it
gathers), collides them and writes the result into the other of two
population buffers. Each node is written by one task and reads only the
previous state, so the CPU result does not depend on the thread count.

## 2. Boundaries

The pull step asks, for population i of node x, where it comes from:

| Source of x - c_i | Rule |
|---|---|
| fluid node | plain streaming |
| solid node (mask) | halfway bounce-back: f_i(x) = f*_opp(i)(x); wall halfway between the nodes |
| below y = 0 / above y = ny - 1, periodic | wrap around |
| same, free slip | specular reflection: f_i(x) = f*_mirror(i)(x - c_ix, y) |
| same, no-slip wall (at y = -1/2 or ny - 1/2) | bounce-back plus the moving-wall term 6 w_i rho_w c_i.u_w (Ladd 1994), rho_w the density of the wall node at the previous step |
| x < 0, inflow | unknown; Zou-He velocity inlet (below) |
| x > nx - 1, outflow | outlet rule (below) |

- **Zou-He velocity inlet** (Zou and He 1997) on x = 0 with the prescribed
  (u_x, u_y): rho = (f0 + f2 + f4 + 2 (f3 + f6 + f7)) / (1 - u_x),
  f1 = f3 + 2/3 rho u_x, f5 = f7 - (f2 - f4)/2 + rho u_x / 6 + rho u_y / 2,
  f8 = f6 + (f2 - f4)/2 + rho u_x / 6 - rho u_y / 2. The node is then
  **regularized** (Latt, Chopard, Malaspinas, Deville and Michler 2008):
  f_i = f_eq,i + w_i / (2 c_s^4) (c_i c_i - c_s^2 I) : Pi_neq. Measured: the
  plain Zou-He node made a uniform 360 x 192 stream at u = 0.05 blow up within
  1300 steps for tau below about 0.54 (an instability travelling along the
  inlet column); the regularized node is stable down to tau = 0.505.
- **Outlet** on x = nx - 1, for the three populations entering from
  x = nx (3, 6, 7): `Outlet::extrapolation` copies them from the outlet
  column (zero gradient, the value f*_i(nx - 1, y - c_iy));
  `Outlet::convective` (default, Yang 2013) solves df/dt + U df/dx = 0
  implicitly, f(t+1) = (f(t) + U f_zero-gradient(t+1)) / (1 + U), U the inlet
  speed, with the previous values kept in a small history (3 ny values).
  Measured: with the zero-gradient outlet the wake of a D = 40 cylinder fed
  back once the vortices reached the outlet (Cl amplitude growing past 1.4
  at Re 100); with the convective outlet the same run settles at Cl 0.43,
  St 0.170.
- Periodic x requires nothing else; y is periodic on both sides or neither.

## 3. Forces

Momentum exchange (Ladd 1994): a boundary link is a fluid node x and a
direction j whose source x - c_j is solid (`boundary_links`). The population
leaving towards the wall, o = opposite(j), is reflected in the next step, so
the wall receives F = sum over links 2 c_o f*_o(x) per step, applied at the
wall point x - c_j / 2 for the moment. Stationary bodies only. Coefficients
per unit span: C_d = F_x / (rho U^2 L / 2), C_l = F_y / (rho U^2 L / 2) in
lattice units. `CoefficientHistory` keeps the instantaneous values and a
moving average over a window; `dominant_oscillation` finds the lift period
from upward mean crossings (linear interpolation, hysteresis of a quarter of
the peak-to-peak range), St = f L / U. `settled_oscillation` accepts that
period only for settled shedding (consistent periods, the same lift
amplitude and mean drag over the first and the last period; a full period
averages the periodic part of the drag out wherever it starts), and
`steady_since` tests a steady history (section 7).

## 4. Units and the Mach guard

`LatticeUnits::create(SiFlow, LatticeResolution)`: with N cells per
characteristic length L and the free stream u_lat in lattice units,
dx = L / N, dt = u_lat dx / U, nu_lat = nu dt / dx^2, tau = 3 nu_lat + 1/2.
The Reynolds number is the same in both systems. Conversions: velocity
dx/dt, time dt, pressure c_s^2 (rho - 1) rho_ref (dx/dt)^2, vorticity 1/dt,
force per span rho_ref dx^3 / dt^2 (force coefficients are unchanged).
APPROXIMATION A3: the scheme is weakly compressible; `create` refuses a
lattice Mach number u_lat / c_s of 0.1 or more, and `validate` applies the
same bound to the inlet and wall speeds. Local speeds near a body are higher
(the cylinder runs reach lattice Mach 0.12); the adapter warns above 0.2.

## 5. Solid masks

Node (x, y) is solid when the signed distance at the node is <= 0
(`mask_from_sdf`, `circle_mask`), when it lies inside a polygon
(`polygon_mask`, `naca_mask` from `geom::Naca4::section` placed by leading
edge, chord and angle of attack), or when the voxel of an
`orrery_geom::VoxelGrid` (generalized winding number, Phase 6) is solid
(`mask_from_voxels`, an xy or xz slice). APPROXIMATION A4: one sample per
node gives a staircase wall; the halfway wall of such a mask encloses the
true area on average (circle of radius 12: 2 percent) but drag converges
only to first order in the resolution (section 8).

## 6. CPU and GPU solvers

`make_cpu_solver(config, mask)` runs the step in double precision with
oneTBB over rows; nodes away from walls and solids use plain streaming, the
others resolve every link. 600 x 400 cylinder channel: 108 million lattice
updates per second (12 threads, MSVC Release; hidden `[benchmark]` of
`orrery_test_aero_lbm`).

`make_gpu_solver(backend, config, mask)` runs the same step through an
`IComputeBackend` with three kernels:

| Kernel | Bindings | Work items |
|---|---|---|
| `lbm_d2q9_step` | 0 source populations, 1 solid flags, 2 destination, 6 outlet history | nx x ny |
| `lbm_d2q9_moments` | 0 populations, 1 solid flags, 3 rho - 1, u_x, u_y | nx x ny |
| `lbm_d2q9_force` | 0 populations, 4 links (cell, j), 5 per-link force | links |

Uniforms carry the lattice size, boundary kinds, inlet and wall speeds, tau,
C^2, the body force and the outlet mode. Populations are stored as
f_i - w_i in binary32; the per-link forces are read back and summed (with
the weights added back) in double on the host.

### GPU precision

The deviations from the weights carry the flow (u = 0.05 changes f by about
w_i 3 c.u, 2e-2 of w), so storing f - w keeps the O(1) weights out of the
rounding error (Skordos 1993; Lehmann, Krause, Amati, Sega, Harting and
Gekle 2022). Measured on the RX 9060 XT (tests in
`tests/aero/lbm/test_lbm_gpu.cpp`, relative L2 differences GPU vs CPU):

| Case | Steps | Velocity L2 | Populations L2 | Drag |
|---|---|---|---|---|
| Poiseuille 8 x 64, tau 0.8, body force | 4000 | 4.3e-5 | 1.0e-7 | |
| Couette 8 x 32, moving wall | 4000 | 4.7e-6 | 8.1e-8 | |
| Taylor-Green 64 x 64, tau 0.6 | 2000 | 6.1e-5 | 3.6e-7 | |
| Cylinder 200 x 100, Re 100, D 10 | 4000 | 4.0e-5 | 3.5e-5 | 4.2e-5 |
| same with the Smagorinsky closure | 4000 | 4.0e-5 | 3.5e-5 | 5.3e-5 |
| Cylinder through `IAeroSolver`, Re 100 | 2000 | | | 1e-6 (Cd 1.35104 both) |

The acceptance bound (1e-4) holds on these shared cases. It does not hold
throughout every run: in a 400 x 200 cylinder run started without a
perturbation (`[.measure]` test) the velocity L2 grows from 4e-5 to 7.1e-4
between steps 10000 and 18000, while the wake breaks its symmetry, and falls
back to 9e-5 once periodic shedding has locked in (steps 50000 to 60000;
lift coefficients then agree to 3 digits). The symmetry breaking is a
physical instability that amplifies any difference in the initial noise,
here the binary32 rounding, by about 1e4 (binary64 rounding differences of
1e-16 would grow to about 1e-12 by the same factor). If that transient must
match, the remedy is a binary64 kernel variant (memory traffic doubles,
binary64 arithmetic runs at 1/16 of the binary32 rate on this GPU class);
binary32 stays the default for speed.

The kernels see tau, the inlet and wall speeds, C^2 and the body force
rounded to binary32. `make_gpu_solver` refuses, before allocating, a
configuration for which that rounding changes the physics class: a value
outside the binary32 range (it would become infinite), or a tau that rounds
to exactly 1/2 (zero viscosity on the GPU while the CPU keeps a positive
one; tau - 1/2 must exceed 2^-25, about 3e-8). Tau values that close to 1/2
are far outside the stable range of BGK anyway (section 7 warns below 0.51).

The OpenGL backend converts every `Float32Uniform` through the same guarded
binary32 rounding as the buffer packing (`round_to_float32`) and refuses a
finite value that would overflow it.

### Throughput

2048 x 1024 cylinder channel (D = 64, Re 1000), hidden `[benchmark]` of
`orrery_test_aero_lbm_gl`: 1117 steps/s (2343 million lattice updates per
second) with BGK, 1113 steps/s with the Smagorinsky closure, on an AMD
Radeon RX 9060 XT (target: 200 steps/s).

## 7. `IAeroSolver` adapter

`LbmAeroSolver2D` takes `AeroGeometry::section` (x-z plane, metres) and its
`ReferenceValues`, and `FlowConditions` (speed, density, dynamic viscosity,
angle of attack). The section is rotated by the angle of attack so the free
stream runs along the lattice x axis; the channel has free-slip sides, a
regularized Zou-He inlet `upstream` reference lengths ahead and a convective
outlet `downstream` behind. `LbmSolverOptions` sets the resolution (cells per
reference length, lattice velocity), domain, LES mode, duration and
averaging window (convective times L / U), steps per `step()` call, the
force sampling interval and, optionally, a compute backend (GPU) instead of
the CPU.

The force is sampled at the lattice steps that are multiples of
`sample_every`, whatever `steps_per_call` is: a call that ends between two
sample points takes no sample, so the samples, the history and the averages
are the same for any split of the run into calls (a run of 800 steps gives
identical samples with 1, 7, 25 or 800 steps per call). The averaging window
is a number of samples, `average_window` L / U divided by `sample_every`
steps.

Result: Cd, Cl and Cm from the force and moment averaged over the window
(per unit span, `set_coefficients` with the caller's reference area); the
force and pitching moment in body axes; wall samples (`surface`: one per
fluid node next to the body, normal from its links, area dx per metre of
span, Cp against the mean density of the column next to the inlet); the
velocity field on the lattice when the angle of attack is zero (the lattice
is wind-aligned otherwise); residuals (relative change of the averaged Cd
per call, reported only); `converged` when the run finished and the force
history over the last half of the run (or the averaging window, if longer)
is settled, either steady (`steady_since`: drag and lift ranges within 1e-3
of the mean drag) or periodic shedding (`settled_oscillation`: lift period
spread within 0.05, the bound of the cylinder validation, and lift
amplitude and mean drag within 2 percent between the first and the last
period). A Strouhal estimate alone is not accepted: `dominant_oscillation`
returns a period after three mean crossings, which the round-off lift of a
symmetric start-up wake also produces (measured: St 1.86 after 10 L / U of a
cylinder at Re 100, while the drag was still falling). Instantaneous coefficients, the history and the Strouhal
number are accessors of the adapter; the instantaneous and history
coefficients go through the same conversion as the result (SI force per
unit span, `set_coefficients` with the reference area), so their window
average equals the result's Cd and Cl also when the area is not the
reference length times 1 m.

Warnings (never blocking): the envelope of `check_validity`
(`lbm.mach`, `lbm.reynolds`) plus `lbm.sideslip` (2D ignores it),
`lbm.resolution` (fewer than 20 cells per length), `lbm.tau` (tau below
0.51), `lbm.les` (closure active), `lbm.les_off` (Re above 2000 without it),
`lbm.blockage` (body above 10 percent of the channel height),
`lbm.local_mach` (lattice Mach above 0.2 in the flow), `lbm.unconverged`
and `lbm.diverged`.

## 8. Validation

`tests/aero_validation/lbm2d/test_cylinder.cpp`: circular cylinder at
Re = 100 in a channel with free-slip sides, started from the free stream with
a transverse bump of 0.1 U one diameter behind the cylinder (so the wake
leaves its unstable symmetric state early), forces sampled every 10 steps,
the last half of the run analysed. Besides the literature ranges the run
must pass `settled_oscillation`, the convergence test of the adapter.

Literature at Re = 100 (unconfined): St 0.164 (Williamson 1996, Annu. Rev.
Fluid Mech. 28:477), 0.165 (Park, Kwon and Choi 1998, KSME Int. J. 12:1200),
0.16 (Braza, Chassaing and Ha Minh 1986, J. Fluid Mech. 165:79); mean Cd
1.33 (Park et al. 1998), 1.35 (Henderson 1995, Phys. Fluids 7:2102), 1.36
(Braza et al. 1986). Re = 200: St 0.197 (Williamson 1996), Cd 1.34
(Henderson 1995). Acceptance: St 0.16 to 0.20 and mean Cd 1.2 to 1.5.

CI case (D = 20 cells, tau 0.53, inlet 8 D ahead, outlet 12 D behind, channel
20 D wide, 26000 steps, about 5 analysed periods):

| Solver | St | Mean Cd | Cl amplitude | Time |
|---|---|---|---|---|
| CPU (double, oneTBB, 12 threads) | 0.1674 | 1.468 | 0.44 | 33.5 s |
| GPU (binary32) | 0.1674 | 1.468 | 0.44 | 1.6 s |

The CPU case runs in optimised builds only (it is skipped in Debug and
sanitizer builds, which CI runs separately).

Resolution and Reynolds number study (hidden `[.study]`, GPU):

| Re | D (cells) | Channel (D) | Grid | Steps | St | Mean Cd | Cl amplitude | GPU time |
|---|---|---|---|---|---|---|---|---|
| 100 | 12 | 8 up, 22 down, 16 wide | 360 x 192 | 30000 | 0.165 | 1.604 | 0.52 | 2.1 s |
| 100 | 20 | 8 up, 12 down, 20 wide | 400 x 400 | 50000 | 0.168 | 1.474 | 0.42 | 3.3 s |
| 100 | 20 | 8 up, 22 down, 16 wide | 600 x 320 | 50000 | 0.169 | 1.481 | 0.43 | 6.3 s |
| 100 (Smagorinsky C 0.1) | 20 | same | 600 x 320 | 50000 | 0.168 | 1.485 | 0.43 | 5.4 s |
| 100 | 40 | 8 up, 22 down, 30 wide | 1200 x 1200 | 100000 | 0.170 | 1.423 | 0.43 | 63 s |
| 150 | 20 | 8 up, 22 down, 16 wide | 600 x 320 | 50000 | 0.186 | 1.485 | 0.68 | 2.9 s |
| 150 | 40 | 8 up, 22 down, 30 wide | 1200 x 1200 | 100000 | 0.188 | 1.410 | 0.63 | 58 s |
| 200 | 20 | 8 up, 22 down, 16 wide | 600 x 320 | 50000 | 0.196 | 1.518 | 0.86 | 2.8 s |
| 200 | 40 | 8 up, 22 down, 30 wide | 1200 x 1200 | 100000 | 0.199 | 1.423 | 0.77 | 59 s |

Literature St at Re 150 is about 0.183 (Williamson 1996); the lift
amplitude at Re 100 is 0.33 (Park et al. 1998), so the channel runs (5 to
6 percent blockage) overpredict it by about 30 percent. D = 12 cells is outside the drag range (1.60) and
D = 20 at Re 200 just outside (1.52); both are under-resolved for the
halfway bounce-back wall.

Drag converges to first order in the resolution (staircase wall with
halfway bounce-back, A4) and is raised by confinement; St is within the
range at every resolution. The CI case sits at Cd 1.47, 2 percent below the
upper acceptance bound; D = 40 in a wider channel gives 1.42.

## 9. Tests

- `orrery_test_aero_lbm` (graphics-free, `[aero][lbm]`): Null backend,
  binary32 packing, lattice moments and isotropy, unit conversion and the
  Mach guard, solid masks (circle, NACA, voxel slice), force history,
  Poiseuille (tau = 1/2 + sqrt(3/16): 5e-13; tau 0.8: 7e-4; tau 0.6: 1.3e-3
  relative L2, the BGK wall slip), Couette (3e-13), momentum exchange
  against the body force (1e-6), mass conservation (1e-12 over 1500 steps
  with periodic, slip and moving walls), Taylor-Green decay rate (within
  0.1 percent of 4 nu k^2), uniform translation between co-moving walls at
  densities 0.7, 1 and 2 (1e-12), determinism, the GPU driver on the Null
  backend (including the binary32 parameter checks), settled and steady
  force histories, and the `IAeroSolver` adapter (step-count guards, call
  size independence, reference-area normalisation, a start-up transient
  that must not count as converged).
- `orrery_test_aero_lbm_gl` (`[gl]`, OpenGL 4.6, hidden window): backend
  buffers, kernels and uniform type checks, embedded shaders, GLSL tables
  against the C++ lattice, GPU against CPU on every boundary type, uniform
  translation between co-moving walls on the GPU (5e-6 at density 2), the
  refusal of a uniform outside the binary32 range, the adapter on the GPU;
  hidden `[benchmark]` and `[.measure]`.
- `orrery_test_lbm2d_validation`: the cylinder on CPU and GPU, hidden
  `[.study]`.

## 10. Three dimensions: D3Q19 (Phase 7c)

| Piece | Header | Source |
|---|---|---|
| D3Q19 lattice | `orrery/aero/lbm3d/d3q19.h` | |
| Solid masks, wall distances, voxel placement | `orrery/aero/lbm3d/solid_mask3d.h` | `src/aero/lbm3d/solid_mask3d.cpp` |
| Solver (CPU and GPU) | `orrery/aero/lbm3d/lbm3d.h` | `lbm3d_cpu.cpp`, `lbm3d_gpu.cpp`, `lbm3d_common.cpp` |
| `IAeroSolver` adapter | `orrery/aero/lbm3d/lbm_aero_solver3d.h` | `src/aero/lbm3d` |
| Kernels (GLSL 460) | | `src/render/shaders/lbm/lbm_d3q19_*.glsl` |

The 3D solver is the 2D scheme (sections 1 to 4) with one more axis; units
(`LatticeUnits`), force histories and the Smagorinsky closure are shared.
Directions: 0 rest, 1..6 the axes (+x, -x, +y, -y, +z, -z), 7..18 the twelve
face diagonals in opposite pairs; weights 1/3, 1/18, 1/36; c_s^2 = 1/3
(Qian, d'Humieres and Lallemand 1992). The equilibrium, BGK, Guo forcing
and the Smagorinsky relaxation time (the factor 18 sqrt(2) holds in 3D as in
2D; |Pi_neq| now has six components) are those of section 1. Memory layout
f[i * cells + x + nx (y + ny z)], the index of `geom::GridSpec`.

## 11. 3D boundaries

| Source of x - c_i | Rule |
|---|---|
| fluid node | plain streaming |
| solid node, `SolidWall::halfway` | halfway bounce-back f_i(x) = f*_opp(i)(x) |
| solid node, `SolidWall::interpolated` | Bouzidi linear interpolation (below) |
| outside y or z, periodic / free slip / no-slip | as in 2D, per side (`y_low`, `y_high`, `z_low`, `z_high`); free slip mirrors c_y (c_z); a no-slip wall may move along x with the density-scaled term 6 w_i rho_w c_i.u_w |
| x < 0, inflow | velocity inlet (below) |
| x > nx - 1, outflow | convective or zero-gradient outlet, the five populations with c_x = -1 |

The y sides are resolved before the z sides, so a population crossing an
edge between two slip walls is reflected by both.

- **Velocity inlet** on x = 0 (Zou and He 1997, regularized after Latt,
  Chopard, Malaspinas, Deville and Michler 2008): rho = (sum_{c_x=0} f +
  2 sum_{c_x=-1} f) / (1 - u_x); the five unknown populations by bounce-back
  of their non-equilibrium part, f_i = f_opp(i) + 6 w_i rho c_i.u; then the
  node is regularized, f_i = f_eq,i + w_i / (2 c_s^4) (c_i c_i - c_s^2 I) :
  Pi_neq. Density and all three velocity components are then exact, so the
  tangential corrections of the D3Q19 Zou-He scheme (Hecht and Harting 2010)
  are not needed; tested to 1e-13 next to a sphere with a skewed inflow.
- **Interpolated bounce-back** (Bouzidi, Firdaouss and Lallemand 2001): the
  wall sits at the fraction q of the link from the fluid node x towards its
  solid neighbour, q = d(x) / (d(x) - d(x - c_j)) from the signed distance d
  of the mask (linear interpolation along the link, exact for a plane wall),
  and
  q < 1/2: f_j(x, t+1) = 2q f*_o(x) + (1 - 2q) f*_o(x + c_j),
  q >= 1/2: f_j(x, t+1) = f*_o(x) / (2q) + (2q - 1)/(2q) f*_j(x),
  o = opposite(j). A link with q < 1/2 whose second node is not fluid falls
  back to q = 1/2, and so does a link whose distances place no wall on it:
  a non-finite distance (validation refuses only NaN), a negative distance
  at the fluid node or a positive one at the solid node, or zero at both.
  q is computed without overflow for any finite pair (the halves are
  subtracted when d(x) - d(x - c_j) overflows), and both solvers read the
  same q (the GPU gets the host's fractions in binary32). The distance comes from the analytic masks (`sphere_mask`,
  `box_mask`, `mask_from_sdf_3d`) or, for meshes, from
  `geom::signed_distance_band` (exact within three cells of the surface,
  signed by the voxel classification; `place_distance`). Measured: the
  sphere drag at D = 16 falls from 9.6 to 4.4 percent above
  Schiller-Naumann at Re 100 (section 14).
- **Absorbing layers** (`Sponge`; Xu and Sagaut 2013): within `thickness`
  nodes of the inlet, the outlet or a non-periodic side the post-collision
  populations relax towards the free-stream equilibrium,
  f* <- f* - sigma (f* - f_eq(1, u_in)), sigma = strength ((T - d) / T)^2.
  The adapter uses a layer of 2 reference lengths before the outlet with
  strength 0.1: it pins the pressure level of the channel (without it the
  mean density drifted with the inflow, and the sphere drag at Re 100 came
  out 15 percent high) and takes out the pressure waves of the start.
- **Gradual start** (`set_inlet_velocity`): the adapter starts from rest and
  raises the inlet speed as U (1 - cos(pi t / T)) / 2 over T = 3 L / U,
  constant over each sampling interval. An impulsive start from the uniform
  stream launched pressure waves that rang between the inlet and the slip
  sides for tens of convective times (drag ripple of 3.4 percent at Re 100
  after 15 L / U, so the steady test of section 13 never passed); with the
  gradual start and the outlet layer the ripple is 0.07 percent and the run
  is steady by the 1e-3 test.

## 12. 3D forces, GPU and throughput

Momentum exchange per link: F = sum c_o (f*_o(x) + f_j(x, t+1)), which is
2 c_o f*_o for the halfway wall (Ladd 1994) and uses the interpolated
reflected population otherwise (Mei, Yu, Shyy and Luo 2002); the moment of
each link acts at the wall point x - q c_j.

`make_cpu_solver(Lbm3DConfig, SolidMask3D)` (double, oneTBB over rows of
constant y and z) and `make_gpu_solver(backend, config, mask)` (binary32,
populations shifted by their weights) share `lbm3d_internal.h`:

| Kernel | Bindings | Work items |
|---|---|---|
| `lbm_d3q19_step` | 0 source, 1 node classes, 2 destination, 6 outlet history, 7 wall fractions | nx x ny x nz |
| `lbm_d3q19_moments` | 0 populations, 1 node classes, 3 rho - 1, u_x, u_y, u_z | nx x ny x nz |
| `lbm_d3q19_force` | 0 populations, 1 node classes, 4 links, 5 per-link f*_o + f_j, 7 wall fractions | links |

A node class is 0 solid, 1 bulk fluid (plain streaming without any check), 2
boundary fluid, or 3 + s for a node with wall fractions in slot s (19 per
slot). Before allocating, `make_gpu_solver` refuses what binary32 cannot
carry (as in 2D), a population buffer above the largest binding and a
lattice above the free device memory, when the backend reports them
(ADR-020; the RX 9060 XT reports 2048 MiB per binding and its free memory
through `GL_ATI_meminfo`).

GPU throughput, 128^3 sphere channel (hidden `[benchmark]` of
`orrery_test_aero_lbm3d_gl`, AMD Radeon RX 9060 XT): 652 steps/s (1367
million lattice updates per second) with BGK, 648 steps/s with the
Smagorinsky closure (target: 60 steps/s on a GTX 1660-class GPU; not
measured on such a GPU). CPU: about 100 million lattice updates per second (12 threads, MSVC Release;
the CI sphere case below, 121 x 61 x 61 nodes, 6000 steps in 27 s).

GPU against CPU (relative L2, `test_lbm3d_gpu.cpp`):

| Case | Steps | Velocity L2 | Populations L2 | Drag |
|---|---|---|---|---|
| Poiseuille 4 x 32 x 4, tau 0.8 | 4000 | 1.6e-5 | 1.3e-8 | |
| Square duct 4 x 20 x 20 | 4000 | 4.7e-7 | 9.0e-11 | |
| Couette 4 x 4 x 16, moving wall | 4000 | 8.0e-6 | 8.3e-8 | |
| Sphere 64 x 32 x 32, D 8, Re 20 | 800 | 7.6e-6 | 6.4e-6 | 1.0e-5 |
| same with Smagorinsky | 800 | 7.7e-6 | 6.4e-6 | 1.0e-5 |
| same, interpolated wall | 800 | 7.6e-6 | 6.4e-6 | 1.1e-5 |
| same, extrapolation outlet, periodic y | 800 | 2.7e-6 | 1.8e-6 | 1.3e-5 |
| Sphere 48 x 24 x 24, D 8, interpolated wall, unusable distances on half the body | 400 | 5.6e-6 | 2.8e-6 | 5.5e-6 |
| Sphere through `IAeroSolver`, Re 50, D 10 | 1000 | | | 1.1e-5 (Cd 1.85151 / 1.85149) |

The 1e-4 bound holds on these laminar cases; the binary32 caveat of
section 6 applies unchanged (an instability transient amplifies the
rounding difference).

## 13. 3D `IAeroSolver` adapter

`LbmAeroSolver3D` takes `AeroGeometry::mesh` (validated with
`geom::validate_mesh`) and its reference values. The lattice is
wind-aligned: x along `drag_direction()`, y along `side_direction()`, z
along `lift_direction()` (a right-handed set), so angle of attack and
sideslip rotate the body. The body is voxelized over its bounding box only
(plus one node), shifted a quarter node off the lattice planes (no node on
an axis-aligned face, where the winding number would sit at the threshold),
and placed `upstream` reference lengths behind the inlet in a square
channel `width` lengths wide with free-slip or periodic sides. Options as in
2D plus `wall` (interpolated by default), the absorbing layers, `start_ramp`
and `max_cells`.

Grid sizing: the cell budget is `max_cells`, else the free device memory the
backend reports divided by 172 bytes per node (and the largest binding
divided by 76), else `default_max_cells_3d` = 256^3 (also for the CPU, about
5.1 GB). A larger lattice is refused before anything is allocated, with the
lattice size, the budget and its reason, and the largest cells per
reference length that would fit. Step counts are checked below 2^53 before
conversion; samples follow the absolute schedule (multiples of
`sample_every`, the start ramp constant over each interval), so samples and
averages do not depend on the call size (tested bitwise for 1, 7, 25 and
400 steps per call). History, instantaneous and result coefficients share
one normalisation (`apply_force`, `set_coefficients` with the caller's
reference area).

Result: Cd, Cl, Cy and Cm of the window-averaged force and moment
(lattice force unit rho dx^4 / dt^2, body axes); wall samples (one per
fluid node next to the body, normal from its links, an equal share of the
mesh area each, Cp against the mean density of the plane next to the inlet);
the velocity field when both angles are zero; `converged` when the force
history of the last half of the run (or the window, if longer) is steady in
drag, lift and side force (ranges within 1e-3 of the mean drag) or settled
periodic in the lift, the side force or the drag (section 7). A wake that
is neither (the sphere at Re 300 sheds irregularly at these resolutions, the
wakes at Re 1000 are chaotic) is reported `lbm.unconverged`; its mean
coefficients are still the window average.

Warnings (never blocking): `lbm.mach`, `lbm.reynolds` (envelope),
`lbm.resolution` (fewer than 16 cells per reference length, or fewer than 4
across the thinnest body extent), `lbm.tau` (below 0.51), `lbm.les`,
`lbm.les_off` (Re above 2000 without the closure), `lbm.blockage` (frontal
area of the voxelized body above 5 percent of the cross-section),
`lbm.local_mach` (above 0.2), `lbm.unconverged`, `lbm.diverged`. Every one
is triggered in `test_lbm_aero_solver3d.cpp`.

## 14. 3D validation

`tests/aero_validation/lbm3d`: the sphere (orrery_geom tessellation, 64
vertices per ring) and a square plate through the adapter, free-slip sides,
interpolated wall, gradual start, outlet layer; forces every 10 steps.

Sphere reference: Schiller and Naumann (1933), Cd = 24 / Re (1 + 0.15
Re^0.687), within a few percent of the measured curve below Re about 800
(Clift, Grace and Weber 1978); Johnson and Patel (1999, J. Fluid Mech.
378:19) computed Cd 0.656 at Re 300. Acceptance: within 15 percent.

| Solver | Re | D (cells) | Channel (D: up, down, wide) | Grid | Steps | Cd | Schiller-Naumann | Ratio | Converged | Time |
|---|---|---|---|---|---|---|---|---|---|---|
| CPU (CI) | 100 | 12 | 3, 6, 5 | 121 x 61 x 61 | 6000 | 1.186 | 1.092 | 1.086 | steady | 27.4 s |
| GPU | 10 | 12 | 6, 10, 10 | 205 x 121 x 121 | 7200 | 4.415 | 4.151 | 1.064 | steady | 18.7 s |
| GPU | 100 | 16 | 4, 8, 6 | 209 x 97 x 97 | 9600 | 1.140 | 1.092 | 1.044 | steady | 16.4 s |
| GPU | 300 | 16 | 4, 8, 6 | 209 x 97 x 97 | 9600 | 0.693 | 0.684 | 1.013 | no | 16.4 s |
| GPU | 1000 | 48 | 2, 4, 4 | 337 x 193 x 193 | 19200 | 0.482 | 0.438 | 1.099 | no | 202 s |

Every case is within the 15 percent; the drag lies above the correlation
throughout (resolution and confinement both raise it). Cd 0.693 at Re 300 is
5.6 percent above the direct simulation of Johnson and Patel (0.656).

The CPU case runs in optimised builds only, like the cylinder. Re 10 needs a
wide channel (the disturbance of a slow sphere decays slowly: in a 6 D
channel the drag was 9.6 percent high); Re 1000 needs D 48, where BGK is
stable without the closure (tau 0.507): with the Smagorinsky closure the
drag at D 32 was 22 percent high, without it 16 percent; the boundary layer
(thickness of order D / sqrt(Re)) sets the resolution.

Study (hidden `[.study]`, GPU; same setup unless noted):

| Body | Re | Cells per length | Channel (up, down, wide) | Wall | Cd | Ratio | GPU time |
|---|---|---|---|---|---|---|---|
| sphere | 10 | 16 | 4, 8, 6 | interpolated | 4.550 | 1.096 | 17 s |
| sphere | 10 | 16 | 6, 10, 10 | interpolated | 4.397 | 1.059 | 56 s |
| sphere | 100 | 16 | 4, 8, 6 | halfway | 1.197 | 1.096 | 16 s |
| sphere | 100 | 24 | 4, 8, 6 | halfway | 1.161 | 1.063 | 77 s |
| sphere | 100 | 10 | 3, 6, 5 | interpolated | 1.219 | 1.117 | 0.8 s |
| sphere | 100 | 24 | 4, 8, 6 | interpolated | 1.127 | 1.033 | 77 s |
| sphere | 300 | 24 | 3, 6, 5 | interpolated | 0.679 | 0.992 | 43 s |
| sphere | 1000 | 32 | 2, 4, 4 | interpolated, Smagorinsky | 0.534 | 1.219 | 48 s |
| sphere | 1000 | 32 | 2, 4, 4 | interpolated | 0.509 | 1.161 | 48 s |
| sphere | 1000 | 40 | 2, 4, 4 | interpolated | 0.511 | 1.167 | 95 s |
| plate | 1000 | 16 | 4, 8, 6 | interpolated | 1.045 | 0.817 (of 1.28) | 20 s |
| plate | 1000 | 32 | 3, 6, 5 | interpolated | 1.206 | 0.942 | 117 s |
| plate | 5000 | 24 | 3, 6, 5 | interpolated, Smagorinsky | 1.185 | 0.926 | 39 s |

The halfway wall converges at first order (9.6 and 6.3 percent above the
correlation at D 16 and 24); the interpolated wall halves the error at the
same resolution (4.4 and 3.3 percent). At Re 1000 the error is set by the
boundary layer, not the wall model; the Smagorinsky closure only adds
dissipation there. The plate below 24 cells per side (one-cell thickness)
is outside the band.

Plate reference: Cd 1.28 for a flat plate normal to the flow (NASA Glenn
Research Center, Beginner's Guide to Aeronautics, "Shape Effects on Drag":
frontal area, low-speed wind tunnel; the page gives no aspect ratio). A
sharp-edged plate's drag depends on its aspect ratio rather than on Re:
about 1.2 for a square plate, rising towards 2 for a two-dimensional one
(Hoerner, Fluid-Dynamic Drag, 1965, chapter 3). The case is a square plate,
thickness 1/16 of the side; acceptance within 15 percent of 1.28.

| Re | Cells per side | Channel (sides: up, down, wide) | Grid | Steps | Cd | Ratio to 1.28 | Time |
|---|---|---|---|---|---|---|---|
| 1000 | 24 | 3, 6, 5 | 219 x 121 x 121 | 19200 | 1.158 | 0.905 | 51 s |

The drag is averaged over the last 25 convective times (the wake is
chaotic and the 5-length windows vary by about 7 percent); 1.158 is within 2
percent of the square-plate value of about 1.2 and 9.5 percent below the
1.28 of the reference.

## 15. 3D tests

- `orrery_test_aero_lbm3d` (graphics-free, `[aero][lbm3d]`): lattice
  isotropy to fourth order, symmetry tables, equilibrium moments; checked
  cell counts and voxel placement (a 4 x 4 x (2^60 + 1) grid that wraps to
  16 cells is refused); plane Poiseuille (tau = 1/2 + sqrt(3/16): 7.3e-13; tau 0.8: 1.2e-3 relative L2), square duct against the
  series solution (5.5e-4), Couette (1.0e-13), uniform translation between
  co-moving walls at densities 0.7, 1, 2 (1e-12), mass conservation with
  periodic, slip and moving walls and a sphere (1.4e-13 over 1500 steps),
  momentum exchange against the body force (1e-6), a uniform stream through
  inlet and outlet, the inlet velocity next to a sphere, determinism,
  validation, memory estimates; the GPU driver on the Null backend (layout,
  ping-pong, binary32 and memory refusals); the adapter (sphere, rotated wind
  frame, every warning, refusals, cell budget, step-count guards, call-size
  independence, reference area, convergence).
- `orrery_test_aero_lbm3d_gl` (`[gl]`): GLSL tables, kernel compilation and
  the memory query, GPU against CPU (section 12), co-moving walls on the GPU
  (8e-6 at density 2), the adapter on the GPU; hidden `[benchmark]`.
- `orrery_test_lbm3d_validation`: section 14; hidden `[.study]`.
