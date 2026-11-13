# Aero Visualisation

The aero view of `orrery_render` (Phase 8a, spec sections 7.1, 7.4, 7.6 and
7.7) draws the results of the aerodynamic solvers: the body coloured by its
surface pressure coefficient, section planes through a flow field,
streamlines and particle traces, the Q-criterion iso-surface and the force
vectors, with colour legends in the overlay.

The renderer takes plain data and draws; it owns no user-interface code.
The Aero workspace of the application decides what to show and passes it in
`AeroViewOptions`; it can draw the legends itself from `AeroView::legends()`
or let `draw_overlay()` draw them.

| Header | Contents |
|---|---|
| `aero_view.h` | `AeroView` (create, `set_surface`, `set_flow`, `set_forces`, `advance`, `draw`, `draw_overlay`, `legends`), the layer options, `frame_bounds`, `aero_hdr_settings` |
| `aero_surface.h` | `AeroSurface` (mesh + Cp per triangle) from 3D and 2D solver results, `AeroForces` |
| `aero_field.h` | regular grids, `FlowField`, the solver adapters, derived quantities, interpolation, section sampling |
| `aero_colormap.h` | colour maps, scales, legend ticks, the display pre-compensation |
| `aero_streamlines.h` | streamlines and particles, CPU reference and GPU path |
| `aero_isosurface.h` | marching cubes, CPU reference and GPU path, mesh topology check |
| `aero_compute.h` | the GPU kernels and the compute backend that runs them |
| `camera.h` | `TurntableMode` |

## 1. Inputs and units

Everything is in body axes (x downstream, y along the span, z up; see
`aero::FlowConditions`), metres and SI units, double precision on the host.

- **Surface.** `AeroSurface` is an `orrery_geom` triangle mesh and,
  optionally, one Cp per triangle. `surface_from_result()` pairs a 3D panel or
  Newtonian result with the mesh it was computed on (one sample per
  triangle); `extrude_section()` turns a 2D panel result (one sample per
  panel of the section) into a short extruded wing with neutral end caps.
- **Flow.** `FlowField` holds the velocity on a regular grid (node
  `(i, j, k)` at `origin + spacing (i, j, k)`, index `i + nx (j + ny k)`),
  optionally a pressure on the same grid and a solid flag per node. A 2D
  field has `nz = 1` and lies in the plane `z = origin.z`. Adapters:
  `aero::VelocityField` (volume solvers), `lbm::MacroFields2D` and
  `MacroFields3D` with a `LatticeMapping` from lattice to world units (node
  spacing in metres, metres per second per lattice speed). With a reference
  speed the lattice pressure is reported as Cp = c_s^2 (rho - 1) / (U^2 / 2).
- **Forces.** `forces_from_result()` projects the solver force on the drag,
  lift and side directions of the flow conditions and keeps the coefficients.

`set_flow` validates the field (grid size at most 2^24 nodes, finite values,
matching counts) and the view derives what it needs lazily: speed, the
vorticity vector (component along the section normal, signed; z for a 2D
field), its magnitude and Q = (|Omega|^2 - |S|^2) / 2. Derivatives are
central differences, one-sided at the grid edges and next to solid nodes
(V6).

## 2. Colour maps

Three maps, all with CIE L* linear along the map (along each arm of a
diverging map) and hue and chroma set in Oklab (Ottosson 2020):

| Map | Use | Definition |
|---|---|---|
| `diverging` | Cp on bodies | blue (hue 255 deg) - light neutral grey - red-orange (38 deg); L* 45 at both ends, 95 at the centre; chroma 0 at the centre to 0.15 |
| `diverging_dark` | signed fields on section planes (vorticity, pressure) | blue (250 deg) - near black - orange (55 deg); L* 14 at the centre, 76 at both ends; chroma 0 to 0.088 |
| `sequential` | magnitudes (speed, abs(omega), Q), streamlines | blue-violet (290 deg) - teal - pale yellow (105 deg); L* 22 to 94 |

Rationale:

- **Perceptual order.** Lightness carries the magnitude, so the order of
  values survives greyscale printing and every colour-vision deficiency.
  Each arm of the diverging maps is a straight line in Oklab, so equal steps
  in the value are equal perceived steps (the measured Oklab step ratio along
  an arm of `diverging` is 1.05; tests allow 10 percent).
- **Colour-vision safety.** The two arms differ along the blue-yellow axis,
  which protanopes and deuteranopes keep; red against green is never used to
  carry a sign. The tests simulate both deficiencies (Machado, Oliveira and
  Fernandes 2009) and require the two arms to stay apart at every distance
  from the centre.
- **Symmetry.** Both arms reach the same lightness and chroma at the same
  distance from the centre, so neither sign looks stronger. Negative values
  (suction) are blue.
- **Light or dark centre.** On a lit body the light-centred map reads best
  (shading stays visible on near-zero Cp). On a section plane in the dark
  workspace the dark-centred map lets the undisturbed flow recede and the
  vortices stand out.

**Scales.** A `ColorScale` is `automatic` (data range), `symmetric`
(`[-m, m]`, zero at the neutral centre; the default for Cp and forced for
signed section quantities unless fixed) or `fixed`. Automatic and symmetric
ranges are widened to the next multiple of a fifth of the legend tick step,
so the ends are round without wasting much of the map. Legend ticks use a
1-2-2.5-5 sequence, the same number of decimals on every tick, and a common
exponent outside [1e-3, 1e5).

**Display pre-compensation.** The HDR pipeline tone maps the scene with the
ACES fit (rendering.md, section 4). Colour-mapped layers are drawn with
`scene_linear_for_display(colour)`, the inverse of that transform, so a
section plane, and a surface facing the key light, show exactly the legend
colour (within 8-bit rounding, tested for every map entry). The maps are
confined to colours the inverse can reach (the ACES fit desaturates the most
saturated sRGB colours); shaded parts of a body are darker than the legend
by design (V9, V11).

## 3. Surface pressure

The body is drawn with its Cp per triangle, or, by default, interpolated to
the vertices (angle-weighted mean of the surrounding triangles, V8) for
smooth gradients. Normals are smooth except across creases of more than 45
degrees (trailing edges, wing tips, caps). Thin iso-lines of Cp at half the
legend tick step help reading values. Triangles without a finite Cp (the
caps of an extruded section) take the neutral body colour.

Lighting (V11) is a key light attached to the camera (up and to the left of
the view direction, so whatever side the turntable shows is lit), a wrapped
Lambert term that never falls below 0.42, a weak highlight and a faint cool
rim against the backdrop. Bloom is off in the aero view (spec 7.6): nothing
adds glow, and `aero_hdr_settings()` gives exposure 1 and 4x MSAA.

## 4. Section planes

A section plane is an axis-aligned cut through the field at a world
coordinate, sampled at the grid nodes in the plane and linearly along the
normal (`sample_section`, V7), uploaded as a float texture and filtered
bilinearly on the GPU. Solid nodes take the body colour with an
anti-aliased edge. The colour range of a quantity comes from the whole
field, not the plane, so moving a plane does not change its colours.
Several planes may be shown at once; translucent planes are drawn after
the opaque layers.

## 5. Streamlines and particles

Streamlines solve dx/ds = u / abs(u) with classical RK4 in arc length (step
0.5 grid spacings by default) on the trilinear velocity (V1, V2), forward,
backward or both from each seed. A line stops when it leaves the grid,
reaches a solid node, falls below 1e-3 of the peak speed or reaches
`max_points`. The default seeds are a 12 x 6 rake on the upstream face (24
seeds along the inlet edge of a 2D field). Lines are anti-aliased
screen-space ribbons coloured by speed, tapered at their ends.

Particles move in time, dx/dt = u, with the midpoint rule (V3), sub-stepped
so that no step crosses more than half a grid spacing. Each keeps a ring of
its last positions; the trace is drawn straight from the kernel's buffer
(vertex pulling, no copy to the host), fading towards the tail and over the
start and end of the particle's life. A particle that leaves the grid, hits
a body or outlives `lifetime` is released again on a seed with a
deterministic jitter. On the first `advance()` the particles are spread over
the whole fluid with random ages, so the view is full at once.

Both run on the GPU (kernels `aero_vis_flow_streamlines` and
`aero_vis_flow_particles`, binary32, positions in grid index units); the
double-precision CPU functions are their references in the tests.

## 6. Q-criterion iso-surface

The surface Q = level (a fraction of the largest Q by default) is extracted
by marching cubes on the GPU (`aero_vis_marching_cubes`, an atomic append of
triangles that the host welds and orders) and drawn translucent, back faces
first. Vertices sit on cell edges, linearly interpolated (V4); normals are
minus the interpolated value gradient (V5).

The case table is not typed in: `marching_cubes_table()` generates all 256
cases from one rule. Per cube face, walking its corners counter-clockwise
seen from outside, every outside-to-inside crossing is joined to the next
inside-to-outside crossing; on an ambiguous face this keeps the two inside
corners apart. Two cells that share a face see the same four corners and
make the same choice, so surfaces that do not touch the grid boundary are
closed and consistently oriented. The face segments form loops, which are
triangulated by dynamic programming so that no diagonal joins two points of
the same face (the neighbouring cell could use the same diagonal, and an
edge in four triangles would not be manifold).

## 7. Force vectors

Lift, drag and (optionally) side force are drawn as lit 3D arrows from the
application point, `length_per_coefficient` metres per unit coefficient, in
the theme accent (lift) and the cool secondary accent (drag). An arrow
shorter than four shaft radii is not drawn (the drag of a wing at small
incidence); the overlay read-out lists the coefficients either way.

## 8. Turntable camera

`TurntableMode` (camera.h) orbits a fixed focus with world +Z up, turns on
its own at `spin_rate`, zooms in log distance and pans: move.x along the
horizontal right axis, move.y along +Z, at `distance` per second. The pan
along the turning right axis is integrated in closed form, so the time-step
contract of the camera holds (tested against 240 updates per second).
`frame_bounds()` places the camera so that the bounding sphere of the scene
fills the narrower field of view.

## 9. Overlay

The legends are rasterised on the CPU into one texture and blended into the
sRGB target after tone mapping: rounded panels in the theme's window colour
with a border, the title, a vertical colour bar with tick marks and labels.
Text uses a small built-in stroke font (monoline glyphs of line segments and
elliptic arcs, drawn as anti-aliased capsules), so the renderer needs no
font library. Metrics scale with the frame height (reference 720 rows). The
texture is rebuilt only when a legend changes.

## 10. GPU paths

The kernels live in `src/render/shaders/aero/compute` and run through
`aero::compute::IComputeBackend` like the LBM kernels: `GlComputeBackend`
accepts extra kernel sources (`create(std::span<const ShaderSource>)`) and
`make_aero_compute_backend()` builds one with the LBM and the aero kernels.
`GlComputeBackend::buffer_name()` exposes the OpenGL name of a buffer so the
particle traces are drawn without a copy. The drawing shaders live in
`src/render/shaders/aero` and are embedded like the others.

## 11. Tests and measured values

Measured on MSVC 19.44 x64 Release (/fp:precise) with an AMD Radeon RX 9060
XT (OpenGL 4.6); `orrery_test_render "[aero]"`.

| Criterion | Setup | Measured | Bound |
|---|---|---|---|
| Diverging map | 513 samples | L* monotone on each arm, mirror-symmetric within 0.5, equal ends; Oklab step ratio 1.049 | monotone, 1.1 |
| Sequential map | 513 samples | L* monotone; step ratio 1.63 (hue turn and gamut limit) | monotone, 1.75 |
| Colour vision | protanopia and deuteranopia simulation | arms of both diverging maps apart at every distance from the centre | Oklab distance > 0.12 d |
| Display pre-compensation | 65 entries of every map, exposure 1 and 0.5 | ACES of the pre-image within half an 8-bit step | 0.5 / 255 |
| Marching cubes, sphere | radius 6, 10.5, 15.25 cells | area error -0.87, -0.28, -0.13 percent (1376, 4156, 8724 triangles); closed, Euler characteristic 2, outward normals | 2 percent |
| Marching cubes, closed random fields | 6 fields, 3 levels, 20^3 | every surface closed and consistently oriented | 0 boundary, non-manifold or misoriented edges |
| Marching cubes GPU vs CPU | 24^3 random fields | identical vertices and triangles (26544 and 26400), positions within 1.3e-7 m | 1e-5 m |
| Streamlines, circular flow | RK4, step 0.05 m | radius drift 2.7e-8 m (r = 1 m) and 3.6e-9 m (r = 1.5 m) over 400 steps, back at the seed after one turn | 1e-7 m |
| Streamlines GPU vs CPU | 16 seeds, 300 points, solid disc | positions within 1.2e-6 m over 1699 points | 1e-4 m |
| Particles GPU vs CPU | 201 particles, one step | within 3e-7 m | 1e-5 m |
| NACA 0012 golden | 3D panel method, AR 4, 4 deg, 2012 panels | SSIM 1.0 against the golden; CL 0.269, CD 0.0061 | SSIM > 0.98 |
| Vortex street goldens | 2D LBM, D = 16 cells, Re 150, 16000 + 2 x 300 steps, CPU | SSIM 1.0 for all three frames; 4.8 s including the LBM run | SSIM > 0.98 |

Golden images: `tests/render/golden/aero/naca0012_cp_aoa4.png`,
`cylinder_vortex_street_0.png`, `_1.png`, `_2.png`; regenerate with
`ORRERY_UPDATE_GOLDEN=1 orrery_test_render "[golden]"` and review them.

## 12. Limits

- The view draws what the solvers give: the panel method has no wake
  field, so a panel result shows the body, its Cp and the forces only.
- Interpolation is trilinear on the grid (V1): features below two grid
  spacings are not resolved, and bodies on a lattice show their staircase on
  a section plane unless a mesh covers it (the cylinder test draws one).
- Particles are advected in the field of the current frame (V3); in an
  unsteady flow their traces approximate pathlines only as well as the frame
  rate follows the flow.
- Marching cubes resolves face ambiguities by one fixed rule and ignores
  interior ambiguities (V4); the topology near saddle points may differ from
  that of the trilinear interpolant, though the surface is always closed.
- Grids are limited to 2^24 nodes; marching cubes to 2^21 triangles by
  default.
- The goldens contain thin particle traces; drivers whose binary32
  arithmetic differs may move them by a pixel, which the SSIM threshold
  absorbs but an exact comparison would not.
