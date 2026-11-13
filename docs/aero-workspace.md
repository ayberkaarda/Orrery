# Aero Workspace

The Aero workspace of `orrery_app` (spec 8, Phase 8b) puts the geometry
pipeline (Phase 6), the aerodynamic solvers (Phase 7) and the aero view
(Phase 8a, `orrery/render/aero_view.h`) behind one user
interface, and records every run as an experiment. Sources: the
graphics-free side in `src/app/aero/*.cpp` compiled into `orrery_sim`
(headers `orrery/sim/aero_*.h`), the user interface and the view adapter in
`orrery_app_core` (headers `orrery/app/aero_*.h`).

## 1. Layout

| Area | Content |
|---|---|
| Main bar | word mark, `Aero` tab, Run/Pause, Step, Stop, pacing (steps per frame or run to convergence), the solver that runs, an `UNVALIDATED` chip for user models, the run status (lattice steps, throughput) and the frame rate |
| Workspace strip | Space, Aero, Entry (disabled until Phase 9); Ctrl+1 / Ctrl+2 |
| Left, top | **Geometry**: Primitive / File / URL tabs, the body report, reference quantities |
| Left, bottom | **Flow Conditions**: free stream, derived numbers, solver choice and warnings, lattice settings |
| Right | **Aero Results**: state, reference area selector, coefficients, breakdown, lattice progress, convergence plots, warnings |
| Bottom | **Experiments** (save, saved list, comparison table, CSV export) and **View** (layers, camera) as tabs |
| Centre | the aero view of the current body and run |

The dock layout is built in code like the space workspace's; Space keeps
its own layout. Drag in the scene orbits the turntable camera, the wheel
zooms, F or Home reframes, Space runs or pauses, `.` or Right steps one
frame.

## 2. Geometry

**Primitives** (`orrery/geom/primitives.h`, `naca.h`), built on a background
thread whenever a value is committed. Bodies are centred on their bounds;
solids of revolution face the free stream:

| Shape | Dimensions | Placement |
|---|---|---|
| Sphere | diameter | centred |
| Cube | side | centred |
| Cylinder (cross-flow) | diameter, span | axis along y |
| Cone | diameter, length | apex upstream (-x) |
| Ellipsoid | length, width, height | semi-axes along x, y, z |
| Flat plate | chord, span, thickness | chord along x |
| Wedge | length, half angle, span | leading edge upstream |
| Entry capsule | maximum diameter; Apollo or Orion proportions | heat shield upstream |
| NACA wing | 4 digits, chord, span | chord along x, span along y; trailing edge detected for the wake |
| NACA section (2D) | 4 digits, chord | x-z plane, per unit span |

Tessellation: 48 vertices around, arcs in 0.08 rad steps, 48 NACA stations
(96 section points), 12 span strips; small enough for the panel method's
dense solve. Moments are taken about the quarter chord of NACA bodies and
the centre of volume of anything else.

**User models** come from a file (OBJ, STL, glTF, GLB, PLY, FBX through
`geom::import_mesh_file`) or an https URL (`geom::net::fetch_model`, ADR-010:
https only, 200 MB, content sniffing, SHA-256 cache, one zip level), both on
a background thread with the current stage shown. The import is then held
for **confirmation of units and orientation**: the importer's estimate
(format unit, or the bounding-box guess with the largest extent in file
units) preselects metres, centimetres, millimetres, inches, feet or a custom
scale, and the up axis (format or convention) Z or Y; the resulting size is
shown before "Use this model". The confirmed model is scaled, turned to Z up,
centred and repaired (`geom::repair_mesh`), again in the background.

The **body report** shows triangles, watertight and outward, and for user
models what repair changed (boundary, non-manifold and inconsistent edges
before and after, components, welded vertices, removed and flipped
triangles, filled holes) with warnings for an orientation conflict or a body
still open after repair. User models carry the **UNVALIDATED GEOMETRY**
badge (spec 5.4: no validation case exists for them) and a **licence /
source note** that is stored with the experiment and never checked.
Reference quantities: frontal, planform and wetted area, length, span,
volume.

Every failure - an unreadable file, a refused URL, a model that fails
validation, a primitive the generators reject - opens the **problem modal**
with a one-line title and the full detail (copyable). Nothing is dropped
silently.

## 3. Flow conditions and solver choice

Inputs: speed [m/s], density [kg/m3], dynamic viscosity [Pa s],
temperature [K], angle of attack and sideslip [deg] (sliders over +-25 and
+-20 deg, Ctrl+click types any value), and a sea-level air (ISA) preset.
Derived live (`sim::summarize_flow`): Reynolds number on the reference
length, Mach number, dynamic pressure, speed of sound, kinematic viscosity.

The solver is the regime recommendation of `aero::select_solver` (ADR-009:
Newtonian from Mach 5, panel method below Mach 0.3 at Re >= 1e5, lattice
Boltzmann below Re 1e5) or the user's override; the rationale is shown.
Newtonian theory needs a surface mesh, so on a 2D section it is replaced by
the panel method with a note (`workspace.solver_unavailable`). Conditions
outside the chosen solver's validity envelope are listed as warnings and
**never block a run**; only values no solver can use (non-positive density,
viscosity, temperature, speed or reference length, gamma <= 1, non-finite
angles) disable Run, with the reason next to the inputs.

Lattice Boltzmann settings: resolution in cells per reference length
(default 40 in 2D, 16 in 3D) and duration in convective times L/U (default
60 and 30).

## 4. Solvers and the solver thread

| Body | Panel method | Newtonian | Lattice Boltzmann |
|---|---|---|---|
| 2D section | `PanelSolver2D` | - | `LbmAeroSolver2D` (CPU) |
| 3D mesh | `PanelSolver3D` | `NewtonianSolver` (Sutton-Graves heat flux with the sphere or capsule nose radius) | `LbmAeroSolver3D` (CPU, at most 4.2 M nodes) |

`sim::AeroRunner` owns the solver on its own thread, like the physics
thread of the space workspace (ADR-011): the UI sends `AeroCommand`s
(start, pause, steps per frame, run to convergence, step one frame, stop)
through a lock-free queue, wakes it once per frame, and reads immutable
progress from a triple buffer. No frame waits for a solve.

**Pacing** (spec 8, "steps per frame" and "run to convergence"): a
time-marching run advances `steps_per_frame` lattice steps for every frame
that woke the thread (in calls of 50 lattice steps in 2D, 20 in 3D; frames
that arrive while a batch runs merge), so the view animates with the frame
rate; run to convergence steps back to back until the solver's duration is
reached, polling commands between calls. Direct solvers finish in their
first step either way. `converged` is the solver's own test (steady or
settled periodic force history for the lattice solvers).

**Published progress**: state, solver, wall time and throughput of the last
call, the latest `AeroResult` and diagnostics (force history, lattice size,
planned steps, Strouhal number, blockage), and the **view data** - Cp per
drawn element and the lattice flow field - refreshed at most every 0.25 s
while running and on every pause, step and finish (reading the lattice
moments costs a pass over every node).

The workspace's lattice Boltzmann runs use the CPU solver: the GPU solver
needs an OpenGL context current on the solver thread, and `GlContext` is
single-threaded. The headless path uses the CPU as well, which keeps its
images deterministic.

**Reference area**: the Results panel chooses frontal, planform or wetted
area (2D: chord x 1 m). The run normalises by the area chosen when it
started; a different choice afterwards renormalises the shown coefficients
(C x A_run / A_new, and A L for Cm) and says so.

## 5. View frame

Both lattice solvers run in the wind frame, so the view does too
(`sim::view_frame`): the free stream always runs along +x and the body shows
its angle of attack, as in a wind tunnel.

- 3D: view axes are the drag, side and lift directions. The lattice origin
  repeats the solver's placement rule (quarter-node shift); a test checks it
  against the solver's own velocity field at zero angles.
- 2D: the same frame with zero sideslip. The section lies in the x-z plane
  and is extruded one chord along y (`render::extrude_section`), so the run
  looks like a short wing; the 2D lattice becomes a light sheet at y = 0
  (two identical node layers at y = +-dx/2, pressure as Cp), seen nearly
  square-on with the wing piercing it. `render::flow_field_from` places a
  2D lattice in the x-y plane instead, which would not meet the extruded
  section, so the workspace builds this field itself.

Cp on the body: copied for solvers with one sample per element (panel,
Newtonian); the lattice solvers' wall samples are assigned to the nearest
triangle or section panel. The Cp colour range is symmetric about zero and
reaches the 98th percentile of |Cp| (at least [-1, 1]), so the singular
panels at wing tips and trailing edges do not wash the map out. The camera
looks from upstream, to the side and above (yaw -2.0 rad, pitch 0.45 rad), so
windward faces and suction sides show while the stream runs left to right;
with a flow field it also frames the near wake. Layers (View panel): Cp on the body, force arrows
(scaled with the body), a section plane (speed, vorticity, vorticity
magnitude, pressure coefficient or Q; 3D: position across the span),
streamlines from a rake upstream of the body, particles, and the Q-criterion
iso-surface (3D). Flow layers need a lattice Boltzmann run.

## 6. Experiments

A run's record is an experiment of domain `aero` in the schema of
`orrery/scene/experiment.h`, version 1 (`orrery/scene/aero_experiment.h`):

```json
{
  "version": 1,
  "domain": "aero",
  "inputs": { "name": "NACA 0012 at 4 deg", "geometry": "primitive",
              "shape": "naca_section", "naca": "0012", "chord_m": 1 },
  "conditions": { "speed_m_s": 30, "density_kg_m3": 1.225,
                  "viscosity_pa_s": 1.789e-05, "temperature_k": 288.15,
                  "angle_of_attack_deg": 4, "sideslip_deg": 0,
                  "gamma": 1.4, "gas_constant_j_kg_k": 287.05 },
  "solver": { "name": "panel", "version": "1",
              "settings": { "reference_area": "planform", "steps_per_frame": 100 } },
  "result": { "solver": "panel", "cd": 0.0012, "cl": 0.4812, "cm": -0.0031,
              "reynolds": 2.05e6, "mach": 0.0882, "reference_area_m2": 1,
              "reference_length_m": 1, "iterations": 1, "converged": 1,
              "warnings": "" },
  "series": { "residual": [3.1e-15] }
}
```

- `inputs`: `geometry` primitive | file | url. Primitives store only their
  own dimensions (`diameter_m`, `length_m`, `width_m`, `height_m`, `side_m`,
  `chord_m`, `span_m`, `thickness_m`, `half_angle_deg`) plus `naca` or
  `capsule`; user models store `path` (relative paths resolve against the
  experiment file), `unit_scale`, `up_axis`, `license`, and for URLs `url`
  and `sha256` (a re-download that hashes differently is reported).
- `solver.name`: `auto` (the recommendation), `panel`, `newtonian`, `lbm`;
  settings `reference_area`, `reference_length_m`, `lbm_cells_per_length`,
  `lbm_duration`, `steps_per_frame`.
- `result` and `series` hold what the run produced; `series` is the
  optional schema addition of this phase (named arrays of finite numbers:
  `residual`, `history_step`, `history_cd`, `history_cl`). Files without it,
  every space experiment among them, load unchanged, and a record without
  series is written exactly as before; no version change was needed.

Unknown keys, wrong types and values no solver accepts are rejected with
the key in the message (`conditions.speed_m_s: must be positive and
finite`); angles stay in degrees in the file so a record round-trips
exactly (tested, together with the bundled space experiments).

**Experiments panel**: a name, Save (inputs and, when the last finished run
or the loaded record used exactly these inputs, its result: a loaded result
is dropped once a condition, a solver setting or the geometry changes, and
comes back when they are restored; written to a temporary file and renamed
into place), the saved list of the experiments folder (unreadable
aero files are listed as problems, other domains skipped), Load, and the
**comparison table**: runs grouped by identical geometry, sorted by angle of
attack, with Cd, Cl, Cm, their differences to the lowest angle, L/D, and the
least-squares lift slope per degree and per radian (2 pi for thin-airfoil
theory). Export CSV writes the table next to the experiments
(`<geometry>_comparison.csv`, RFC 4180, full precision).

## 7. Headless

`orrery_app --headless --experiment <aero experiment> --out <file.png>
[--frame N] [--size WxH]` reads the domain first and renders aero
experiments through `app::render_aero_headless`: the geometry is prepared as
in the workspace, the solver runs on the calling thread (a time-marching
solver for N frames of the experiment's steps_per_frame lattice steps, or
to completion when N is 0; a direct solver always to completion), and the
aero view draws it with the workspace's default layers and framing. The
summary line names the body, solver, angle of attack, Reynolds and Mach
numbers, coefficients, lattice steps, convergence and each warning.
`--benchmark` remains a space-view measurement. Bundled:

| Name | Body | Solver | Note |
|---|---|---|---|
| `aero_naca0012_panel` | NACA 0012 section, chord 1 m | 2D panel | 30 m/s, 4 deg, Re 2.1e6 |
| `aero_naca0012_lbm` | NACA 0012 section, chord 0.1 m | 2D lattice Boltzmann | Re 400, 4 deg, 200 steps per frame |
| `aero_naca2412_wing` | NACA 2412 wing, 1 m x 6 m | recommended (3D panel) | 50 m/s, 5 deg |
| `aero_apollo_capsule` | Apollo-type capsule, 3.912 m | recommended (Newtonian) | Mach 20, trim 20 deg: Cd 1.37, L/D -0.30 |

## 8. Approximations and limits

- The view's 3D lattice origin repeats a private placement rule of
  `LbmAeroSolver3D` (tested at zero angles; the solver has no accessor).
- Lattice Boltzmann wall Cp is mapped to the nearest drawn element (visual
  only; the coefficients come from the solver's momentum exchange).
- The workspace runs the CPU lattice solvers (see section 4); large 3D
  lattices are slow there, and the 3D node budget is 4.2 M.
- User models are repaired automatically after the unit confirmation; the
  report says what changed, and the result stays marked unvalidated.
