# Orrery Rendering Reference

Conventions and design of `orrery_render`. Phase 4a provides the foundation:
camera, OpenGL context and object wrappers, shader embedding, the HDR chain,
headless snapshots, PNG output and snapshot comparison. Phase 4b adds the
space scene (sections 10 to 12: bodies, starfield, trails), the immutable
`SimSnapshot` the renderer reads (section 9) and the Dear ImGui / ImPlot
layer (section 13). The physics thread, the triple buffer and the
application loop come in Phase 4c (ADR-005, ADR-011). Phase 5 adds the
black-hole lensing pass (section 17: Schwarzschild ray marching in a compute
shader, an environment cube map of the sky, a thin accretion disk); bloom,
rings and atmospheres come later.

## 1. Precision and units

| Space | Type | Unit | Origin and axes |
|---|---|---|---|
| Physics (core) | `double` | AU, day (ADR-001) | barycentric, ecliptic J2000, +Z north |
| World (render input) | `double` (`glm::dvec3`) | metre | same origin and axes as physics |
| Render space | `float` (`glm::vec3`) | metre | camera eye at the origin, world axes |
| View space | `float` | metre | camera looks along -Z, +Y up |

Choice: render space is **metres, camera-relative, float**. Positions are
converted from AU to metres in double at the render boundary (1 AU =
149597870700 m, `core::constants::astronomical_unit`), the camera position is
subtracted in double (floating origin), and only the camera-relative result
is cast to float. Float then carries about 7 significant digits relative to
the camera distance, which is all a perspective view needs. No float value in
the render path ever holds an absolute world coordinate.

Metres rather than AU keep near-camera geometry (surface views, probes,
meshes in the aero workspace) in natural units; the log depth buffer and the
float exponent range (to 3.4e38 m) cover interstellar distances.

### Floating origin

`Camera` stores the eye as an **anchor** plus an **offset**:

- Orbit, Chase, Surface: the anchor is the target body position, the offset
  the camera position relative to it. A body's render position is
  `(body - anchor) - offset`; for the target itself this is exactly `-offset`,
  and for nearby bodies the subtraction is exact (Sterbenz lemma). Precision is
  therefore independent of the distance from the world origin.
- Free: the anchor is the eye itself. The eye position is an absolute double,
  whose spacing is about 0.25 m at 1e4 AU and 2e-5 m at 1 AU. Use Orbit or
  Chase for close work near distant bodies.

Test (`tests/render/test_camera.cpp`): a body at 1e4 AU, camera 1 km away. The
render position has a relative error below 1e-6 (only the final float
rounding remains) for a Free camera against the exact difference of the stored
doubles, and for a Chase camera against the requested offset. Millimetre
camera steps at 1e4 AU move the render position by millimetres. The same
positions converted to float before subtraction collapse onto the eye.

## 2. Camera

All camera math is double precision and independent of OpenGL
(`include/orrery/render/camera.h`).

| Mode | State | Behaviour |
|---|---|---|
| Free | position, orientation, speed | 6-DoF flight; input moves along camera axes, yaw/pitch/roll turn about camera axes, zoom scales speed |
| Orbit | target body or fixed focus, distance, yaw, pitch, view shift | eye at `focus + distance (cos p cos y, cos p sin y, sin p)`, looks at the focus, world +Z up; a view shift (half view heights right and up) slides eye and look point in the view plane, so the focus is drawn off centre at any yaw, pitch and distance |
| Chase | target body, distance, yaw, pitch | offset in the body's travel frame (forward = velocity, up = world +Z made orthogonal); yaw = pitch = 0 is directly behind; the frame turns smoothly with the velocity |
| Surface | body, latitude, longitude, altitude, heading, pitch, speed | eye at `radius + altitude` on the body sphere in the body-fixed frame (`BodyFrame::orientation`), looks along a heading from local north; input walks along the heading as arc length on the sphere |

Bodies are resolved each update through a `BodyFrameLookup` callback
(position, velocity, orientation, radius); if a target disappears, `update`
returns false and the view stays where it was.

Input is a set of rates (`CameraInput`: move in [-1, 1], yaw/pitch/roll in
rad/s, zoom in e-folds per second) applied by `update(dt, input, lookup)`.
Mode parameters first move the goal; the view follows with exponential
smoothing (time constant `smoothing_time`, default 0.15 s; 0 disables it).
Distances are smoothed in log space. Switching modes snaps.

Time-step contract: all rates of one update are held constant and
integrated together, so the same input over the same time gives the same
camera however the time is split into updates.

- Goals are integrated in closed form: Free flight turns about the fixed
  camera axis (pitch, yaw, -roll) while its speed decays as
  `exp(-zoom t)`, and the position follows that turning, decelerating path
  exactly; Orbit/Chase yaw, pitch and log distance are linear; Surface
  heading, pitch, speed and altitude are exact, and ground motion along the
  turning heading is exact in the local tangent plane.
- A clamp (pitch limit, 1 mm minimum distance, zero altitude) splits the
  update at the moment it is reached, so the smoothing sees the true
  piecewise-linear goal.
- The smoothing uses the exact solution of `dc/dt = (g - c) / tau` for a goal
  that moves linearly within a piece. Free and Surface goals move along
  curves, so they are advanced in internal substeps of at most 1/240 s (at
  most 10000 per update) and smoothed per substep; Surface converts
  tangent-plane motion to latitude and longitude per substep with the
  midpoint rule. Splits into multiples of 1/240 s reproduce the same
  substeps; other splits differ by the second-order error of the substeps.

Tested (`test_camera.cpp`, `[timestep]`): Free move.z = 1 with zoom = 1 for
1 s travels 1 - e^-1 = 0.632120559 m within 1e-12 m for 1, 2, 30 and 240
updates (previously 0.368, 0.487, 0.622 and 0.631 m); forward flight while
yawing follows the exact circular arc within 1e-12 m; combined
move/turn/roll/zoom input and Surface diagonal movement (including a descent
to zero altitude and the pitch limit) at 1, 30, 60 and 240 updates per
second agree within 1e-9 after a 5 s settling period; Orbit and Chase pitch
and zoom saturation agree within 1e-10 with the closed-form lag of the
saturating ramp (Orbit pitch rate 2 rad/s after 1 s: 1.498666 rad for every
split; previously 1.334627 with one update).

Limits: pitch is clamped to +/- (90 deg - 1e-3 rad) so the view never aligns
with the up axis; surface latitude to +/- (90 deg - 1e-6 rad); distances to
at least 1 mm; altitude to at least 0.

Matrices (float, glm):

- view = rotation only (`conjugate(orientation)`); the eye is the render origin;
- projection = `glm::infinitePerspective(fov_y, aspect, near)` (OpenGL clip
  convention, z in [-w, w]); no far plane clips anything;
- `inverse_view_projection_matrix` and `Camera::to_world` invert both for
  picking; the product is formed in double and cast once.

Lens and aspect validity: `Camera::set_lens` accepts a lens only with
`vertical_fov` in [1e-6, pi - 1e-6] rad, `near_plane` in [1e-9, 1e12] m and a
finite `log_depth_far` greater than `near_plane`; anything else (zero or
negative near plane, FOV 0 or pi, NaN, log far 0, ...) returns an error
naming the value and keeps the previous lens (`Camera::validate_lens` checks
without applying). The projection accessors replace an aspect ratio that is
not a positive finite number (a minimised window has width 0) by 1 and clamp
positive aspects to [1e-6, 1e6] (`Camera::usable_aspect`). With these limits
every projection matrix and its inverse are finite in float.

## 3. Logarithmic depth

`depth = log2(1 + w) / log2(1 + far)`, with `w` the clip-space w (view
distance along the axis, metres) and `far = 1e20 m` by default. The vertex
stage passes `w`, the fragment stage writes `gl_FragDepth` (snippet
`shaders/log_depth.glsl`, CPU twin `include/orrery/render/log_depth.h`), and
the depth attachment is `GL_DEPTH_COMPONENT32F`. Per-fragment evaluation keeps
large triangles that span many orders of magnitude in `w` correct, at the cost
of early depth testing.

The smallest resolvable relative separation at distance `w` is about
`eps * depth * ln 2 * log2(1 + far) * (1 + w) / w` with `eps = 2^-23`: below
1e-5 from 0.1 m to 1e4 AU (test: two surfaces 1e-5 apart always get
different float depths), so planets, moons and their rings do not z-fight.

## 4. HDR chain

```
scene pass       RGBA16F colour + DEPTH32F, 4x MSAA by default, scene-linear radiance
   | resolve     multisample blit to a single-sample RGBA16F texture
   v
tone map pass    exposure -> ACES fitted (RRT + ODT) -> clamp [0, 1]
   |             written to SRGB8_ALPHA8 with GL_FRAMEBUFFER_SRGB:
   v             the hardware applies the sRGB transfer function on store
sRGB target      read back as 8-bit RGBA (glGetTextureImage: stored bytes, no conversion)
```

- Scene shaders output scene-linear radiance in linear sRGB primaries. Values
  above 1 are expected (highlights, emissive surfaces).
- `HdrSettings::exposure` multiplies the radiance before tone mapping.
- Tone mapping: Stephen Hill's fit of the ACES reference rendering and sRGB
  output transforms (input matrix, rational curve, output matrix); see the
  approximation list. The CPU reference `tone_map_aces` / `resolve_hdr_pixel`
  is checked against the GPU result for constant HDR colours from 0 to 100 at
  several exposures: identical within one 8-bit level on the reference
  machine.
- Bloom (dual filter, spec 7.6) is not part of Phase 4a.

## 5. OpenGL layer

- `GlContext` creates a GLFW window (visible, or hidden for offscreen and
  headless use) with an OpenGL 4.6 core, forward-compatible, debug context and
  loads entry points with glad. KHR_debug messages are routed to
  `log_message` (synchronous output; notifications are disabled); messages of
  type ERROR or severity HIGH are counted and `OffscreenRenderer::render`
  fails a frame that produced any.
- GLFW and glad are private dependencies: no public header includes them, and
  OpenGL names cross the API as `std::uint32_t`. glm is public (camera API).
- The vcpkg glad port (0.1.36) generates a compatibility-profile loader; it is
  a superset of the core entry points and is used with a core context.
- RAII wrappers (`gl_resources.h`): `Shader`, `Program`, `Buffer`,
  `VertexArray`, `Texture`, `Framebuffer`. Move-only, direct state access,
  delete on destruction; use only while the creating context is current.
  Framebuffer completeness errors name the status
  (`GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT`, ...).
- Contexts do not share objects, so an object name means something only in
  the context that created it. Each wrapper records that context and deletes
  its object only while it is current; otherwise the delete is skipped (the
  object is released with its context), logged as an error and counted in
  `detail::gl_handle_stats()`, because the same name may denote another
  object in the current context.
- `Buffer::create`, `Buffer::create_uninitialized` and `Texture::create_2d`
  return `Result`: empty buffers, sizes beyond `GLsizeiptr`, texture sizes
  outside [1, `GL_MAX_TEXTURE_SIZE`] and sample counts outside [1, the
  format's sample limit] are rejected before any GL call; after the storage
  call the GL error and the stored size are checked, and on failure the
  object is deleted and an error returned. Drivers commonly accept even
  absurd sizes and commit memory lazily, so the failure path is tested with
  `detail::SimulatedAllocationFailure`, which makes allocations on the
  calling thread fail after the storage call.
- Logging (`log.h`) is a minimal sink-based logger with no external
  dependency; the default sink writes to stderr. The sink is called without
  any logger lock held, so it may replace or remove itself and may log again
  (for example from a synchronous GL debug callback); a sink replaced while a
  call is running stays alive until that call returns. Calls are not
  serialised: a sink must be thread-safe.
- Contexts are single-threaded: create, use and destroy them on one thread.

## 6. Shaders

- Sources live in `src/render/shaders/*.glsl`. At build time
  `src/render/cmake/embed_shaders.cmake` embeds every file into the generated
  header `orrery/render/embedded_shaders.h` (`shaders::<stem>` with the file
  name and source, plus `shaders::all`); nothing is read from disk at run
  time. Line endings are normalised to LF. The source is emitted as adjacent
  raw string literals of at most 8000 bytes (MSVC limits one literal to
  16380 bytes): pieces end at line boundaries, and a longer line is cut into
  pieces of its own, never inside a multi-byte UTF-8 character; the pieces
  concatenate to the source bytes exactly. `test_shader_embedding.cpp`
  compiles fixtures with a 20,000-byte line, a 30,003-byte line of 3-byte
  UTF-8 characters and 1500 short lines, and checks every piece and the
  byte-for-byte reconstruction (the script takes `-DNAMESPACE=` for this).
- A stage is a main file starting with `#version 460 core` plus optional
  snippets (for example `log_depth.glsl`), composed by `compose_shader`: each
  piece is its own source string introduced by `#line 1 <n>`. Compiler logs
  are rewritten from `n:line` / `n(line)` to `file.glsl:line`
  (`annotate_shader_log`), so errors read like
  `ERROR: test_shape.frag.glsl:37: 'foo' : undeclared identifier`.
- Uniform naming: `u_` uniforms, `a_` attributes, `v_` varyings, `o_` outputs.

| Shader | Purpose |
|---|---|
| `fullscreen.vert.glsl` | fullscreen triangle from `gl_VertexID` |
| `tonemap.frag.glsl` | exposure + ACES; linear output into the sRGB target |
| `log_depth.glsl` | log depth snippet |
| `background.frag.glsl` | test scene backdrop gradient |
| `test_shape.vert.glsl`, `test_shape.frag.glsl` | test shape pair: lit torus with log depth |
| `noise.glsl` | snippet: PCG3D integer hash, value noise, fbm (driver-independent) |
| `body.vert.glsl`, `body.frag.glsl` | instanced icosphere bodies: procedural surfaces, PBR-lite, emissive stars, log depth |
| `glow.vert.glsl`, `glow.frag.glsl` | screen-space star glare and black-hole ring sprites |
| `stars.vert.glsl`, `stars.frag.glsl` | starfield point sprites at infinity |
| `milky_way.frag.glsl` | Milky Way band (with `fullscreen.vert.glsl`) |
| `trail.vert.glsl`, `trail.frag.glsl` | anti-aliased screen-space trail segments, near-plane clipped |
| `black_hole_trace.comp.glsl` | compute: Schwarzschild null geodesics per half-resolution sample, thin disk (section 17) |
| `black_hole_composite.frag.glsl` | edge-aware upsampling of the trace, lensed sky lookup, shadow edge, depth (with `fullscreen.vert.glsl`) |
| `sky_cube_stars.vert.glsl`, `sky_cube_stars.frag.glsl` | starfield into the faces of the environment cube map: Gaussian in the angle from each star |

## 7. Headless snapshots

`render_snapshot(width, height, scene, settings)` (or a reusable
`OffscreenRenderer`) creates a hidden-window context, runs the scene callback
with the HDR framebuffer bound and cleared, resolves, tone maps and returns an
`ImageRgba8` (sRGB, rows top to bottom). Creation fails with a message when no
window system or no OpenGL 4.6 driver is available.

Creation has two stages that tests keep apart: the context
(`OffscreenRenderer::context_options()` with `GlContext::create`) and the
pipeline built in it (`OffscreenRenderer::create(context, ...)`). Tests tagged
`[gl]` are skipped only when the context cannot be created, unless
`ORRERY_REQUIRE_GL=1` makes that a failure; any later failure (shader
compilation, texture allocation, framebuffer completeness) always fails the
test. `HdrPipelineOverrides` lets tests inject a broken tone map shader or an
unusable output format to check this.

Lifetime: a renderer releases its GL objects with its own context current.
Destruction and move assignment make that context current, delete the
pipeline, destroy the context and make the previously current context
current again (nothing is current afterwards if it was the renderer's own).
`render` makes its context current again after the scene callback, and
`render_snapshot` restores the caller's context, so a snapshot nested in
another renderer's frame leaves that renderer working.
The `--headless` application flags (spec 7.7) are described in `app.md`
(Phase 4c); they use this offscreen path with a hidden GLFW window. EGL/Mesa
contexts for display-less CI are not part of V1 Phase 4: there the `[gl]`
tests skip unless `ORRERY_REQUIRE_GL=1`.

PNG output (`png.h`) is dependency-free: 8-bit RGBA, non-interlaced, CRC-32
per chunk, Adler-32 over the data. The writer picks a row filter per row
(None, Sub, Up, Average, Paeth: smallest sum of absolute filtered values) and
compresses with its own deflate: LZ77 over a 32 KiB window (hash chains of
3-byte prefixes, one step of lazy matching), blocks of up to 32768 tokens
written as stored, fixed-Huffman or dynamic-Huffman blocks (length-limited
canonical codes), whichever is smallest. Output is deterministic and decodes
with standard decoders (the `render.png_python_zlib` test decodes written
files with Python's zlib and compares every pixel). The reader inflates all
three block types and rejects malformed input: over-subscribed or incomplete
Huffman codes (except the single one-bit code deflate allows), invalid
symbols, distances before the start of the output, repeats beyond the code
count, truncation, trailing data, preset dictionaries and checksum
mismatches. Decompression is bounded by the image: the output may not exceed
the `(4 * width + 1) * height` scanline bytes of the header (at most 2^28
pixels), so a small file cannot make the reader inflate or allocate more than
the image it announces (tested with a stream that would expand to 64 MiB).
The rendered references compress to 3 to 19 percent of their raw size.

## 8. Snapshot comparison and golden images

`ssim(a, b)` implements SSIM (Wang et al. 2004) per R, G, B channel on the
8-bit values with an 11x11 Gaussian window (sigma 1.5), C1 = (0.01 * 255)^2,
C2 = (0.03 * 255)^2, averaged over the valid window positions. The score is
the minimum of the three channel means. Images are checked before anything is
allocated: `ImageRgba8` computes `width * height * 4` with overflow checks (a
size that does not fit in `size_t` is never `valid()`, and `filled` returns
the empty image for it), and `ssim` refuses images above 2^24 pixels, whose
double-precision work buffers would take more than 1.5 GB.

Golden test: `tests/render/test_golden.cpp` renders the pipeline test scene
(`test_scene.h`: a lit torus 1e4 AU from the world origin, Orbit camera at
5.4 m, 320x240, 4x MSAA, exposure 1) and requires SSIM > 0.98 against
`tests/render/golden/test_shape.png`. Every run writes the actual image to
`<build>/tests/render_output/test_shape_actual.png`. The reference is only
written on request:

```
ORRERY_UPDATE_GOLDEN=1 <build>/tests/orrery_test_render "[golden]"
```

The test then reports that it wrote the reference; review the image before
committing it. A missing reference fails with this instruction. The
reference was generated on an AMD Radeon RX 9060 XT (OpenGL 4.6 core); other
drivers differ by a few levels at edges and in highlights, which SSIM > 0.98
tolerates.

The test scene: off-white albedo, warm key light from the upper left at an
irradiance that pushes highlights above 1, a cool rim light from behind on
the right, a two-colour hemispheric ambient, and a dark blue backdrop
gradient. Independent of the reference image, the test also checks that the
backdrop corners equal the CPU tone map of the gradient ends and that the
torus is lit well above the backdrop without clipping to white.

## 9. Simulation snapshot

`core::SimSnapshot` (`src/core/include/orrery/core/sim_snapshot.h`) is the
only physics data the renderer and the panels read (architecture rule:
`std::shared_ptr<const SimSnapshot>`). `SimSnapshot::capture(state,
sequence)` copies time, ids, names, kinds (with black-hole spin and probe
aero refs), masses, radii, positions and velocities in space units (AU,
solar mass, day; double), in BodyStore order, so the id column is sorted and
`index_of` is a binary search. There are no mutators; the private
constructor makes `capture` the only way to build one, and copy and move
construction and assignment are deleted (compile-time tested), so no caller
can hold a writable `SimSnapshot` behind a published const pointer; sharing
is by copying the `shared_ptr`. Capturing reads the state once, so the source
`SimState` must not change during `capture`, and spans returned by the
accessors are valid while their snapshot is alive. The type lives in
core and depends only on core headers (no GL; `tools/check_deps.py` still
reports core graphics-free and float-free). The physics thread hands
snapshots to the render thread through a lock-free triple buffer
(`app.md`, "Physics thread").

## 10. Bodies

`SpaceRenderer::draw(view, frame)` draws a snapshot into the bound HDR scene
target in four passes:

1. Milky Way and starfield, at infinity, additive, no depth (section 11);
2. bodies, instanced icospheres with log depth, back faces culled;
3. trails, alpha-blended, depth-tested, no depth write (section 12);
4. glow sprites (star glare, black-hole ring), additive, depth-tested
   against the centre's log depth so that bodies in front hide them.

The CPU part of the frame (`prepare_bodies`) is separate from OpenGL and
unit-tested: visual radius, LOD, frustum culling, lighting, instance data and
the projected screen position of every body (`BodyScreenInfo`, used for
labels).

**Meshes.** `make_icosphere(n)` subdivides the icosahedron n times
(10 * 4^n + 2 vertices, 20 * 4^n triangles, counter-clockwise from outside).
Midpoints are formed and normalised in double and cast to float once, so the
vertices lie on the unit sphere to float rounding (< 2e-7) and double as
normals. All seven levels live in one vertex and one index buffer; each
frame draws one `glDrawElementsInstancedBaseVertexBaseInstance` per used LOD.

**Screen bounds.** A sphere seen off the optical axis projects to an ellipse
stretched towards the image edge. `project_sphere` derives its bounds from
the camera-space centre (distance d, depth z along the view axis, radius r,
s = pixels per unit tangent):

- `radius_px`, the largest screen extent: the semi-major axis
  `r sqrt(d^2 - r^2) / (z^2 - r^2) * s`. It equals the on-axis apparent radius
  `r / sqrt(d^2 - r^2) * s` at the image centre and grows towards the edges
  (at 90 degrees FOV, a sphere of 1.5 m at (90, 0, -100) m on a 1080-row image
  spans 10.90 px, not the 6.02 px of the distance-only formula). Labels and
  glare sprites use it (`BodyScreenInfo::radius_px`).
- `lod_scale_px = r (d + r) / (z - r)^2 * s`, the largest magnification of the
  projection over the sphere: a point displaced by e on the sphere moves at
  most `e / r * lod_scale_px` pixels on screen. It is at least `radius_px`
  and equal to it for small spheres.

Both are infinite when the sphere reaches the plane of the eye.

**LOD.** The silhouette error of a mesh is bounded by measured geometry, not
by an assumed edge length: every point of an inscribed polyhedron lies
between the sphere and the sphere through its closest face plane, and a face
seen edge-on at the silhouette pulls the outline in by that full face depth
`1 - h`. Subdivision does not halve every edge (the new central edges are
longer than half their parent), and the face depth exceeds the sagitta
`1 - cos(theta / 2)` of the longest edge by about a third, so both are
measured from the generated meshes (`make_icosphere`, float vertices) and the
larger one is used. `select_lod(lod_scale_px, bias)` takes the coarsest level
with `(1 - h) * lod_scale_px <= 0.25` px:

| LOD (subdivision) | 0 | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|---|
| vertices | 12 | 42 | 162 | 642 | 2562 | 10242 | 40962 |
| longest edge [rad] | 1.1071 | 0.6283 | 0.3264 | 0.1648 | 0.0826 | 0.0413 | 0.0207 |
| face depth 1 - h | 0.2053 | 0.0658 | 0.0178 | 0.0045 | 0.0011 | 2.8e-4 | 7.1e-5 |
| up to lod_scale_px [px] | 1.22 | 3.80 | 14.1 | 55.2 | 220 | 878 | any |

Tested: every level is measured again (longest edge, deepest face), the
bound holds at each threshold, and the projected outline of the actual mesh,
in the worst orientation (deepest face at the silhouette) and in arbitrary
ones, stays within 0.25 px of the exact projected sphere, on the axis and for
off-axis spheres with 60 to 120 degree fields of view and several aspect
ratios. The Render Settings bias shifts the level by -3 .. +3.

**Visual radius policy.** Physical radii are tiny at solar-system scale: at
12 AU the Sun covers about 0.6 px and the Earth 0.006 px on a 1080-row
image. Two modes (`ScaleMode`):

- `true_scale`: every body is drawn at its physical radius. Honest, and
  most bodies vanish below a pixel.
- `clamped` (default): the physical radius, enlarged where necessary so that
  the body covers at least a per-kind minimum screen radius, times the
  "Min. size" factor (0.25 .. 8):

  | star | planet | moon | asteroid | black hole | probe |
  |---|---|---|---|---|---|
  | 5 px | 3 px | 2 px | 1.25 px | 4 px | 2.5 px |

  The enlarged radius is `d m / sqrt(s^2 + m^2)`: angular radius m pixels at
  distance d (s = pixels per unit tangent), which is the projected radius at
  the image centre. The policy is deliberately angular: a marker keeps its
  size wherever it is, so off the axis its drawn ellipse (`radius_px`) is
  somewhat larger than m. Once a body is resolved
  the physical radius wins, so close-ups are never distorted (tested: the
  Earth disc is identical in both modes in the Earth-Moon golden image). A
  body drawn above its true size gets a marker weight `1 - true_px / min_px`
  that keeps its night side at 30 percent of the lit albedo, so unresolved
  bodies never read as black holes.

The physical radius is the snapshot radius when positive; otherwise the
nominal radius from a built-in table of solar-system bodies (IAU/NASA mean
radii; matched on the first word of the name, so "Jupiter system" is
Jupiter) or, for unknown bodies, from mass and a bulk density per kind. A
black hole uses its shadow radius `sqrt(27)/2 r_s`. Stars other than the Sun
get radius, luminosity and temperature from main-sequence mass relations,
except S2 near Sgr A* (T_eff = 28,500 K, R = 5.53 R_sun; Habibi et al.
2017): the bundled scenario enters it as a zero-mass test star, which the
relations would draw as a cool red dwarf instead of the hot B star it is.

**Materials and lighting.** Each kind and known body has a primary and a
secondary albedo, a procedural surface style (rocky, cloudy, terrestrial,
desert, banded, ice giant) evaluated from value noise in the body frame
(spin axis from the IAU pole), and a specular weight. Lighting is PBR-lite:
Lambert diffuse plus an energy-normalised Blinn-Phong lobe with Schlick
Fresnel, from one directional light per body (the star that gives it the
highest irradiance, direction computed in double at the body centre), plus a
0.25 percent ambient. Irradiance is `6 * L (1 AU / d)^p` (scene-linear, L in
solar luminosities); p defaults to 0.5 so that Neptune stays visible next to
Mercury, and the Render Settings "Falloff" slider reaches the physical p = 2.
Without any star a soft headlight from the camera is used. A star at the
body's own position (coincident bodies, which the body store allows) has no
direction and is not a light candidate; the direction to any other star is
normalised after scaling by its largest component, so it stays finite even
when the squared distance underflows (tested: coincident and 1e-200 AU
pairs give finite instance data and an evenly lit disc).

**Stars** are emissive: photosphere radiance 14 times the black-body colour
of their effective temperature (unit luminance, linear sRGB from the
Planckian locus), limb darkening `1 - 0.6 (1 - mu)`, faint granulation. They
are by far the brightest objects, so the future bloom pass has HDR values to
work with. Until then an analytic glare sprite marks them: a point spread
function sized from the disc radius d, `0.55 / (1 + x^2)^2` with
`x = r / 1.6d` (core) plus power-law wings `0.09 / (1 + (r / 3d)^2)^1.5`
and a faint veil `0.012 / (1 + (r / 9d)^2)^1.5`, both falling as r^-3, times
the photosphere colour at 0.11. The sprite reaches 24 d (64 .. 480 px) and
its window only fades the last, already faint, part of the wings from 30
percent of that extent, so the glare has no visible rim (an earlier flat
skirt read as a grey disc around every star). **Black holes** are drawn by
the lensing pass (section 17); the black disc of the shadow radius with a
faint warm ring sprite remains as the marker of a hole whose shadow is below
its minimum screen size, and of every hole when lensing is off. **Probes**
are small emissive cyan markers.

## 11. Starfield and Milky Way

`generate_starfield(params)` is deterministic per seed on every platform
(own SplitMix64 generator, no standard-library distributions). Defaults:
1e5 stars, V = -1.5 .. 10, cumulative counts `N(<m) ~ 10^(0.35 m)` (about
4000 stars brighter than V = 6, as on the real sky; tested), spectral
classes O..M with the mix of a magnitude-limited sample and temperatures per
class, colours from the black-body locus, and a disc population whose share
grows towards faint magnitudes, concentrated in galactic latitude and towards
the galactic centre (tested: 40+ percent of V > 8 stars within 10 degrees of
the plane, against 17 percent for an isotropic sky). The galactic frame
follows the IAU definition (pole and centre tested against their ecliptic
coordinates).

Stars are point sprites with a Gaussian profile at infinity: the vertex stage
multiplies the direction (w = 0) by the view-projection matrix, so only the
camera rotation matters. Peak radiance `2.4 * 10^(-0.4 * 0.85 m) / sigma^2`,
sigma 0.42 .. 1.02 px growing for bright stars, scaled with the viewport
height. The Milky Way is a full-screen pass in galactic coordinates: a
Gaussian disc that thickens towards the centre, a bulge, star clouds and a
gentle finer texture from value noise, and a meandering dust lane broken into
filaments that dims the band by up to 70 percent. The noise is evaluated on
galactic axes stretched 1.5 times across the plane and domain-warped along
it, so clouds and lanes run along the band and meander instead of forming
round blobs (no seam: the coordinates are the galactic direction, not
longitude). The tint is a desaturated cool white along the disc and warmer
towards the bulge. Intensity scale 0.03 scene-linear (the band itself stays
below about 0.1 even at the bulge), which the ACES toe keeps a backdrop: the
default framing camera looks about 30 degrees from the galactic centre, so
the band is behind every opening view and must not compete with the trails.
Tested: the band is more than twice as bright towards the centre as towards
the anticentre, and dark at the galactic pole.

## 12. Trails

`TrailHistory` keeps one fixed-capacity `TrailRing` of world positions
(metres, double) per body id, sorted by id: new bodies start a trail, bodies
missing from a snapshot (merged, absorbed) lose theirs, and `set_settings`
changes the length keeping the newest points. A sample is stored only once
the body has moved by `min_step_angle * (distance to the most massive body)`
since the last stored one, so every trail spans about `length *
min_step_angle` radians of its orbit whatever the period (tested with two
orbits of period ratio 8); 0 stores every recorded snapshot.

Each frame `append_trail_segments` converts a ring plus the body's current
position into segments relative to the camera (`Camera::to_render_double`,
then one float cast): float-safe at any distance from the origin (tested at
1e4 AU with 1 km spacing, which a float world coordinate cannot resolve).
The fade is `1 - age / capacity`; the shader squares it. Repeated world
positions (a stationary body, the newest sample equal to the head) are
merged on the CPU: a run of equal points counts once, with the fade of its
first point, so a stationary body has no trail at all instead of hundreds of
stacked zero-length quads. Segments are drawn as instanced screen-space
quads: both ends clipped against the near plane in clip space, widened
perpendicular to the screen direction, with a one-pixel coverage ramp for
anti-aliasing (independent of MSAA), log depth, and alpha blending. A segment
whose projection is shorter than 0.01 px (seen along the view axis) has no
direction to widen along and is dropped by the vertex shader; neighbouring
segments overlap at the joints, so no cap is lost. Tested by rendering: an
empty ring, one stationary sample, 512 stationary duplicates and a segment
along the view axis leave the image bit-identical to the image without
trails, while a moving one-sample trail draws. Trail colours are the body hue, desaturated and dimmer than the
bodies. Recording cadence is the caller's (Phase 4c: per physics snapshot).

## 13. User interface

Dear ImGui (docking branch, vcpkg port 1.93.0 with `docking-experimental`,
`glfw-binding`, `opengl3-binding`) and ImPlot (1.1) are private dependencies
of `orrery_render`; no public header includes them.

`UiContext` owns one ImGui and one ImPlot context (made current in
`begin_frame`; no `imgui.ini`, the layout is code-defined):

- `UiContext::create(gl_context, options, attach_window_input)`: OpenGL 3
  renderer backend (GLSL `#version 460 core`), optionally the GLFW platform
  backend on the context's window. Fails cleanly for a moved-from context.
  The destructor shuts the backends down with its own GL context current and
  restores the previously current one, or no context if none was current
  (tested with a detached context).
- `UiContext::create_headless()`: no OpenGL and no GLFW; frames are laid out
  and their draw lists built, but not rasterised. Used by the unit tests on
  every platform (including CI without a display).

`dockspace()` covers the viewport with a transparent central node (the
scene) and builds the default layout once: Body Inspector on the left (23
percent), Render Settings on the right, System Monitor at the bottom between
them. Offscreen, `OffscreenRenderer::render(scene, overlay)` calls the overlay
after tone mapping with the 8-bit sRGB target bound
(`HdrPipeline::begin_overlay`, `GL_FRAMEBUFFER_SRGB` off), so ImGui colours
are display values, as ImGui expects.

**Panels** are free functions of a data struct returning an intent struct;
they read no physics state and keep no state of their own:

| Panel | Data | Intent |
|---|---|---|
| `draw_body_inspector` | body list; selected body: kind, mass (Msun, kg), radius (nominal flagged), position (AU) and velocity (km/s), distance, speed, osculating a, e, i, period around its primary | `select`, `focus_camera` |
| `draw_system_monitor` | sim time, body count, frame and physics step time, integrator, one recorded series (for example the relative energy drift) plotted with ImPlot | `clear_series` |
| `draw_render_settings` | `RenderSettings` (scale mode, minimum size, LOD bias, star glare, trails, starfield, Milky Way, exposure in EV, light falloff; with `RenderSettingsContext::black_hole` also the BLACK HOLE section of section 17) | `changed` (sanitised copy) |
| `draw_scene_labels` | `make_scene_labels`: visible bodies, selected first, then by mass, decluttered | none |

`make_body_inspector_data(snapshot, selected)` computes the osculating orbit
with `core::to_elements` relative to the body's primary: the least massive
heavier body whose sphere of influence `r (m / M_heaviest)^(2/5)` contains it,
else the heaviest body (Moon around Earth, planets around the Sun; tested).

**Theme.** Slate surfaces close to the black of space (window #101318,
controls #1B1F27 .. #2D333F), light grey text (#DADEE5, 14:1 on the panel),
muted labels (#8E96A4, 6:1), one warm accent (#E2A75C, sunlight amber) for
interaction and data, and a cool secondary (#74AACC). Contrast ratios (WCAG)
and the ordering of the surfaces are unit-tested; the stock ImGui blue is not
used. Embedded scalable ImGui font at 15 px, 4 px rounding, 12/10 px window
padding, upper-case muted section captions, label-left form rows.

Readability details, all checked in the multi-size render test below:

- Slider grabs are the accent at 45 percent over the track
  (`UiTheme::slider_grab`), because ImGui draws the value text over the
  grab: the text keeps 4.6:1 there (tested >= 4.5:1) while the grab stays an
  amber mark on the grey track (2.6:1). It is opaque only while dragged.
- The settings rows put controls at a column sized for the longest label
  and capped at 40 percent of the panel; in a narrow panel the second Scale
  option wraps under the first. Property tables use a fixed label column
  ("Period" plus spacing) and give the values the rest, so "3.0035e-06 Msun"
  fits a 235 px inspector. The kind next to each body name is dropped where
  it would run into the name.
- Empty states: an inspector without bodies says "No bodies to show."; the
  System Monitor plot without samples shows "No data recorded yet" in an
  empty frame (no placeholder [0, 1] axes). The time axis is in days with as
  many decimals as the span needs (2 below 2 d, 1 below 20 d), or in years
  from five years on, plotted in that unit so that the ticks fall on round
  values.
- Scene labels carry a 1 px dark shadow (70 percent black) so that names
  stay readable over the Milky Way and trails.
- The dock nodes hide their window menu button (the arrow in front of each
  panel tab): the layout is code-defined and the menu could only undo it.

## 14. Phase 4b golden snapshots and tests

Same procedure as section 8 (`ORRERY_UPDATE_GOLDEN=1 orrery_test_render
"[golden]"`, SSIM > 0.98, actual images in `<build>/tests/render_output/`):

| Reference | Size | Scene |
|---|---|---|
| `space_solar_system.png` | 960x540 | J2000 solar system loaded with orrery_scene, oblique Orbit camera 12.5 AU from a point 4.2 AU from the Sun, 40 degree FOV, trails from a 150-year backward leapfrog run (600 samples, 85 percent of each orbit), starfield and Milky Way |
| `space_earth_moon_true_scale.png`, `space_earth_moon_clamped.png` | 640x360 | the J2000 Earth from 32,000 km with the Moon beside it, 55 degree FOV, no trails; true scale and clamped (min. size x2) |
| `ui_space_workspace.png` | 1280x720 | the window's composition: app bar (running, 1e6x), docked panels (Earth selected, drift plot of the backward run) and scene labels over the solar system |

Whole-image SSIM cannot see a small planet: in the mostly empty 960x540
frame, erasing a 10x10 patch changes at most 20x20 of the 950x530 SSIM
windows, so even the worst score there leaves the mean above 0.998. The
solar-system comparison therefore also checks every visible planet on its
own (`golden_mismatches`): around each body that is at least 14 px from the
Sun (Venus, Earth/Moon, Mars, Jupiter, Saturn), the window of half-size
`ceil(radius_px) + 2` must show the body in both images (peak at least 25
luma levels above the window border's median) with summed and peak luma
above that background within 35 and 30 percent of the reference. Tested
without GL on the reference image: erasing any one of these planets keeps
the SSIM above 0.99 but is rejected, in either direction, while a half-pixel
shift of every body plus +-2 levels of noise is accepted.

Besides SSIM, the tests check structure that holds on any driver: the Sun is
the brightest pixel and nothing else clips, the sky stays dark on average,
the Earth disc is identical in both scale modes while the Moon grows, the UI
panels are docked below the app bar and carry text while the scene shows in
the central area. With the compressed PNG writer (section 7) the five
references take 0.80 MB (previously 7.9 MB of stored blocks):
`space_solar_system.png` 270 KB, `ui_space_workspace.png` 385 KB, the two
Earth-Moon images 60 to 61 KB each, `test_shape.png` 22 KB.

A further `[ui]` test renders the same composition at 1920x1080, 1366x768
and 1024x640, and at 1024x640 in the state before the first snapshot (no
bodies, no series, a limited warp and a simulation message), checks that
the panels stay docked below the bar with a usable central area, and writes
`ui_workspace_<size>[_empty]_actual.png` for review (no reference: these
images exist to catch layout regressions by eye at sizes the golden does
not cover).

CPU tests (no GL): icosphere topology and precision, LOD thresholds, scale
policy, frustum culling, appearance table and black-body colours, settings
clamping, frame preparation (lighting direction, LOD, culling, float safety
at 1e4 AU), trail ring and history, starfield statistics and galactic frame,
UI theme contrast, inspector data, label declutter, headless dock layout and
pointer routing, and a simulated click selecting a body.

## 15. Performance

`[perf]` test, 1920x1080, 4x MSAA, J2000 solar system (10 bodies), 600-sample
trails (about 6000 segments), 1e5 stars, Milky Way, glare; GPU time from a
`GL_TIME_ELAPSED` query around scene + resolve + tone map (readback
excluded), 60 frames after 5 warm-up frames, AMD Radeon RX 9060 XT
(OpenGL 4.6, driver 26.10.1):

| GPU median | GPU p95 | CPU submit median |
|---|---|---|
| 1.4 .. 1.9 ms | 1.4 .. 1.9 ms | 0.15 ms |

Two runs on the reference machine: median 1.38 / 1.86 ms, p95 1.40 / 1.93 ms
(the frame is far below the 16 ms budget; the remaining time belongs to
physics and UI). The test requires the GPU 95th percentile below 16 ms and
GPU median plus CPU submit below 16 ms.

The black-hole view (section 17: lensing pass with the disk, 512 steps, same
resolution and MSAA) has its own `[perf]` test: GPU median 1.07 ms (933 fps),
p95 1.16 ms on the same GPU, against the spec's 30 fps; the headless
`--benchmark` of `app.md` measures the full frame without a window.

## 16. Approximations

Every item is marked `// APPROXIMATION:` in the code.

| Where | What | Why | Validity |
|---|---|---|---|
| `shaders/tonemap.frag.glsl`, `tone_map.h` | ACES RRT + ODT replaced by Hill's fitted matrices and rational curve | full CTL transforms are far too expensive per pixel | a few percent from the reference over scene-linear [0, ~16]; output clamped to [0, 1] |
| `shaders/test_shape.frag.glsl` | energy-normalised Blinn-Phong, Schlick Fresnel (F0 = 0.04), hemispheric ambient | test shape only; exercises the HDR path, not a material model | test scene only; not used for bodies |
| `camera.cpp` | rotation smoothing uses a slerp analogue of the exact scalar blend | quaternions have no linear blend | exact when lag and increment share an axis; second order in the per-update rotation otherwise; affects camera feel only |
| `camera.cpp` | Surface latitude/longitude advanced with the midpoint rule per substep of at most 1/240 s | the path along a turning heading on a sphere has no closed form | tangent-plane motion exact; second order in the substep; splits into multiples of 1/240 s give identical substeps; camera placement only |
| `body_visuals.cpp` | minimum screen size: unresolved bodies drawn enlarged (clamped mode) and shaded as markers | true radii are far below a pixel at solar-system scale | positions exact; sizes of unresolved bodies symbolic; `true_scale` exact |
| `body_visuals.cpp` | body colours: hand-chosen albedo pairs; IAU poles at J2000 without precession | no measured maps or spectra are bundled | display only |
| `body_visuals.cpp` | generic stars: R ~ M^0.8, L ~ M^3.5, T from L and R | scenarios carry only the mass | within ~2x for main-sequence stars of 0.1 .. 50 Msun; wrong for giants and remnants |
| `body_visuals.cpp` | unknown bodies: radius from mass and a bulk density per kind | scenarios carry no radius | display size within ~2x for solar-system-like bodies |
| `body_visuals.cpp` | black-body colour from cubic fits of the Planckian locus | avoids integrating the Planck spectrum | ~1e-3 in xy, 1667 .. 25000 K (clamped outside) |
| `space_renderer.cpp` | one directional light per body from its brightest star, at the body centre; irradiance (1 AU/d)^p with display default p = 0.5 | single dominant star; outer planets must stay visible | exact direction for one star; brightness ratios compressed unless p = 2 |
| `space_renderer.cpp` | star brightness on screen: flux 10^(-0.4 * 0.85 m), widening sprites | 11.5 magnitudes exceed what a display shows | brightness order kept; not photometric |
| `starfield.cpp` | spectral mix of a magnitude-limited sample, log-uniform temperatures per class | no catalogue data | plausible as a population, not star by star |
| `starfield.cpp` | galactic star-count structure (disc share, scale height, longitude excess) | no catalogue data | visible concentration towards the Milky Way only |
| `shaders/body.frag.glsl` | procedural surfaces from value noise | no textures or maps | look of each body class; no real geography |
| `shaders/body.frag.glsl` | PBR-lite: Lambert + normalised Blinn-Phong with Schlick Fresnel, small ambient; no atmosphere, shadows or eclipses | restrained shading for bodies seen from space | plausible; no mutual shadowing |
| `shaders/body.frag.glsl` | photosphere: linear limb darkening (u = 0.6), faint granulation | display of emissive stars | Sun-like stars |
| `shaders/body.frag.glsl` | black-hole marker: unlensed black disc of the shadow radius with a faint ring | used only when lensing is off or the shadow is below the minimum screen size (section 17) | position and apparent size of the shadow only |
| `black_hole.cpp` | thin disk around a non-rotating hole: zero thickness, plane normal to +Z, ISCO to 30 M, Newtonian-limit temperature profile, black body without limb darkening | rendering model; ADR-003 defers Kerr | shape and Doppler/redshift pattern of a thin, optically thick disk; luminosity display-scaled |
| `black_hole.cpp` | one lens; bodies drawn unlensed and hidden by the shadow and disk through depth | each body would need its own lensed images | a single dominant hole and the sky at infinity |
| `shaders/black_hole_trace.comp.glsl` | static camera; rays that exhaust the step budget count as captured; disk opacity fade and streak texture | the space camera has no physical velocity; display only | exact at rest; at 512 steps only rays within about 1e-4 of b_c are cut short |
| `shaders/black_hole_trace.comp.glsl`, `black_hole_composite.frag.glsl` | radiance bound: lensing-pass light scaled down (hue kept) to at most 1e4 per component | near the horizon the blueshift g grows without limit and B(g T) would overflow the half-float HDR target (Inf, then NaN in tone mapping) | display only: 4500 x the disk peak, white at any exposure; reached only for g >= 2 (camera within a few M of the horizon) |
| `shaders/black_hole_composite.frag.glsl` | half-resolution trace upsampled edge-aware; occlusion depth from the nearest sample | ADR-017 | exact at the critical curve; near it the sky is filtered rather than resolved |
| `shaders/sky_cube_stars.vert.glsl` | lensed stars keep the angular brightness of a 1080-row, 40 degree screen | a cube map has no screen pixels | within about 10 percent of the sprite sky at 1440 x 810 .. 1920 x 1080 |
| `blackbody_spectrum.cpp` | CIE 1931 colour matching functions from an analytic fit; out-of-gamut components clipped | no tabulated data bundled | about 1e-3 in black-body xy; exact gamut above about 1900 K |
| `shaders/glow.frag.glsl` | star glare: analytic point spread function (Moffat core, r^-3 wings) | stands in for optical glare until the bloom pass | display only |
| `shaders/milky_way.frag.glsl` | Milky Way: Gaussian disc, bulge, noise star clouds and filamentary dust lanes along the plane | no survey data | band position and overall shape, not detail |

## 17. Black hole (Phase 5)

Spec 7.3, ADR-003 (Schwarzschild, Kerr deferred) and ADR-017 (half-resolution
strategy, step budget). Sources: `black_hole.h` (CPU side: selection, sample
grid, rays), `black_hole_pass.{h,cpp}` (GPU side, private), `blackbody_spectrum.h`,
shaders `black_hole_trace.comp.glsl`, `black_hole_composite.frag.glsl`,
`sky_cube_stars.{vert,frag}.glsl`; the double-precision reference of the geodesics is
`orrery/core/schwarzschild_geodesic.h` (graphics-free, in orrery_core).

### Frame

```
lensed hole?  select_lensed_black_hole: largest Einstein angle sqrt(4M/D) on screen >= 0.25 px
   | yes
environment   cube map 6 x 2048^2 R11G11B10F + mips: Milky Way and starfield rendered once
   |          (again only when the sky brightness settings change)
trace         compute, one ray per half-resolution sample (8 x 8 groups), RGBA32F field:
   |          disk light + sky transmittance, escape direction + depth of the first opaque hit
composite     fullscreen, full resolution, additive into the HDR scene target, depth ALWAYS:
   |          edge-aware upsampling, one cube-map lookup per pixel, analytic shadow edge,
   |          log depth where the shadow or the disk covers the sky
bodies, trails, glow sprites as before (depth-tested against the shadow and the disk)
```

Without a lensed hole (or with "Lensing" off) the sky is drawn as in section
11. The pass replaces that sky draw: the cube map is the same Milky Way shader
and the same star buffer, so the lensed and the unlensed sky look alike.

### Geodesics

Units are geometrized (G = c = 1, lengths in M = G M / c^2, horizon r = 2).
The camera is a static observer at r0 (no aberration from its motion). A
pixel ray leaves it at an angle alpha from the outward radial direction e1;
it stays in the plane of e1 and e2 = (d - cos(alpha) e1) / sin(alpha), with
impact parameter b = r0 sin(alpha) / sqrt(1 - 2/r0). With u = 1/r and phi
the angle in that plane:

    u'' = -u + 3 u^2,   u(0) = 1/r0,   u'(0) = -u0 sqrt(1 - 2 u0) cot(alpha)

- RK4 with step h = min(4 pi / N, 0.25 max(u, 0.01) / |u'|), N the step budget
  (Render Settings "Steps", 64 .. 512, default 512): the budget covers two
  turns of the photon plane; the second term limits the change of u per step
  for nearly radial rays.
- Captured when u >= 1/2 (horizon); escaped when u <= 0, with the escape
  angle phi + atan2(u, -u') from the end of that step (near u = 0 the orbit is
  the straight line u = A sin(phi_inf - phi)), escape direction
  cos(phi_inf) e1 + sin(phi_inf) e2; a ray that uses up the budget has
  circled the photon sphere and is treated as captured.
- Disk-plane crossings: with a = e1.N and c = e2.N (N the disk axis) the
  plane is crossed at phi = atan2(c, a) + pi/2 + k pi, known in advance; u
  there comes from the cubic Hermite dense output of the RK4 step. Every
  crossing between the ISCO and the outer radius contributes, front to back,
  until the transmittance falls below 1e-3, so rays that pass the plane
  inside the ISCO or beyond the disk and meet it again behind the hole draw
  the higher-order images (the arc over the shadow, the underside below it
  and the thin photon ring).

### Disk

Thin, in the plane normal to world +Z (spin is not modelled), from the ISCO
(6 M = 3 r_s) to 30 M, gas on prograde circular geodesics with
Omega = r^(-3/2).

- Temperature (Shakura and Sunyaev 1973; Page and Thorne 1974 in the
  Newtonian limit), zero torque at the inner edge:
  T(r) = T_peak f(r/r_in) / f(49/36), f(x) = x^(-3/4) (1 - x^(-1/2))^(1/4);
  T ~ r^(-3/4) far out, maximum at 49/36 r_in, zero at r_in. T_peak is the
  Render Settings "Temp." slider (2000 .. 20000 K, default 4500 K).
- Frequency ratio g = nu_obs / nu_em = sqrt(1 - 3/r) / (1 - Omega lambda) /
  sqrt(1 - 2/r0), lambda = -b (e1 x e2).N the photon's angular momentum about
  the axis per unit energy: gravitational redshift, transverse Doppler shift
  and the Doppler shift of the orbital motion (approaching side g > 1).
- Colour and beaming: I_nu / nu^3 is invariant, so the observed specific
  intensity is g^3 times the emitted one at nu / g; for a black body that is
  exactly the black body at g T, whose bolometric intensity scales as g^4.
  The pass applies neither power as a separate factor: it shades each hit
  with the visible-band radiance of the black body at g T, so beaming and
  colour shift follow from the one exact transformation (in the visible the
  boost is steeper than g^4 on the cool side, the Wien tail). Tested: the
  identity g^3 B_nu(nu/g, T) = B_nu(nu, g T) to 1e-9 and the bolometric
  ratio g^4 to 1e-4 for g = 0.5 .. 1.9.
- Planck to linear RGB (`blackbody_spectrum.h`): the Planck spectrum
  integrated over 360 .. 830 nm (1 nm trapezoid) against the CIE 1931
  colour-matching functions in the analytic fit of Wyman, Sloan and Shirley
  (2013), XYZ to linear sRGB (D65), out-of-gamut components clipped to 0;
  uploaded as a 1024-texel table even in ln T from 400 to 200000 K. Its
  chromaticity agrees with the Planckian-locus fit of section 10 within
  0.006 in x and y from 2000 to 20000 K.
- Brightness: radiance = 2.2 x "Brightness" x (black body at g T) / Y(T_peak),
  so the hottest ring at g = 1 has luminance 2.2: the approaching side
  saturates to white, the receding side stays orange. The disk is opaque out
  to 0.82 of its outer radius and thins to transparent at it; its emission
  carries a sheared streak texture (display only, see the approximations).
- Radiance bound: a camera close to the horizon sees the disk blueshifted
  by the observer factor 1 / sqrt(1 - 2/r0) (about 14 at r0 = 2.01 M), and
  B(g T) then outgrows B(T) by 10^5 and more, beyond the largest half float
  (65504) of the HDR target. The trace scales each sample's light down, hue
  kept, to at most 1e4 per component, and the composite does the same for
  the pixel it writes (APPROXIMATION, physics.md R11). The look-up of B(g T)
  itself is clamped to the table's 200000 K.

### Upsampling and sky

The trace samples sit at the pixel corners (2i + 1, 2j + 1), so every pixel
centre has four samples around it. The composite blends them bilinearly,
but leaves out (weight x 1e-4) samples on the other side of the critical
curve b = 3 sqrt(3) M, which it evaluates analytically for the pixel and for
each sample (camera outside the photon sphere: a ray is captured exactly
when it moves inwards with b < b_c). Escape directions, not colours, are
interpolated; where a neighbour's direction differs by more than about 5.7
degrees (close to the critical curve) the nearest sample's direction is
used. The sky is then looked up once per pixel, at the mip level of the
angular footprint of the lensed direction against the cube texel angle
(2/N) m^(3/2) (m the largest direction component), so stars stay sharp and
the compressed sky near the shadow is filtered. The shadow edge is
anti-aliased from the analytic b over its screen derivative. Rays within 8
percent of b_c, where the higher-order images of the disk and of the sky
(the photon ring) are thinner than a sample and escape directions change
faster than the composite can interpolate, are traced four times (the pixel
centres of the sample, those on the sample's side of the critical curve) and
their light is resolved in the compute pass: disk light averaged, the sky
looked up per sub-ray at the mip level of the spread of their escape
directions. Such a sample hands no sky to the composite.

Stars are drawn into the cube map with the angular width and the summed
brightness they have on a reference screen of 1080 rows over 40 degrees
(sprites narrower than 0.6 texels are widened at constant flux). The
unlensed sprite sky keeps a per-pixel brightness instead, so the two agree
at window sizes around 1440 x 810 .. 1920 x 1080 and the cube-map stars are
dimmer in small offscreen images. Each star is a Gaussian in the angle
between the texel's direction and the star (`sky_cube_stars.frag.glsl`), so
its profile does not depend on where it falls on the cube; near an edge or
a corner its footprint is split over two or three faces. Point sprites are
clipped by their centre, so the stars are drawn through a viewport that
reaches 16 texels past each face (projection widened to match): a face also
draws the stars whose centres lie just beyond it and keeps its own share.
Tested: a star a quarter texel from an edge or a corner keeps its
face-centre flux within 10 percent (before, a star at a corner kept 0.77 of
it, each face holding a quarter of a plane Gaussian over a third of the
angles around the corner).

### Space view

- `select_lensed_black_hole` picks the hole with the largest Einstein angle
  in pixels (at least 0.25 px); only that hole lenses (others keep their
  marker). `prepare_bodies` leaves its instance and ring sprite out while its
  shadow is at least the clamped minimum size (4 px); a smaller, unresolved
  shadow keeps the marker of section 10 on top of the lensed sky.
- Bodies, trails and sprites are not lensed: they are drawn in the normal
  passes and hidden where the shadow or the opaque disk covers them (the
  composite writes the log depth of the hole, or of the first opaque disk
  hit, for pixels whose sky transmittance is below one half).
- Render Settings show a BLACK HOLE section when the scene holds one:
  Lensing, Disk, Brightness, Temp. and Steps (tooltips explain the last
  two); the section disables what has no effect.
- The `black_hole` experiment frames a lone hole 12 degrees above its disk
  (app.md, "Space view"). The distance fits the disk in both directions
  with an 8 percent margin: its width in perspective
  (R / sqrt(D^2 - R^2 cos^2 i), widest at the rim points nearer than the
  hole), the near rim below the hole and the lensed far rim above it (weak-
  field lens equation to third order), and never less than 1.08 R, outside
  the disk and the horizon. At 16:9 the width decides (D = 58.0 M); in
  windows wider than about 2.4:1 the height does (D = 47.9 M). Tested with
  the double-precision geodesics: rays through the frame border miss the
  disk and rays at 65 percent of the frame meet it, from 8192 x 64 to
  256 x 1024.

### Tests and measured values

AMD Radeon RX 9060 XT, OpenGL 4.6, Release MSVC:

| Test | Measured |
|---|---|
| GPU (float) against the CPU reference (double), 3724 rays at 40 M, 60 degree view, excluding 1 percent around b_c | fates identical; escape direction median 6.1e-7 rad, worst 3.5e-5 rad; b 4.9e-6 relative; first disk radius 5.0e-5 relative; g 8.6e-6 |
| Shadow radius from the captured samples, 400 x 400 at 70 degrees, camera at 12 / 30 / 80 M | 122.8 / 48.5 / 18.1 px against tan(psi) x pixel scale 122.9 / 48.5 / 18.4 px |
| Einstein ring of a star exactly behind the hole, 60 M, 640 x 360 | brightness-weighted radius 143.52 px, geodesic prediction 143.52 px; closed in all eight sectors; shadow interior luma below 1 |
| Doppler asymmetry, 80 M, 9 degrees, boxes at 12 and 18 M on both sides | mean luma approaching 235.5, receding 20.2 (required: more than twice); blue/red 0.93 against 0.08 |
| Weak lens far behind the camera, 1440 x 810 | Milky Way SSIM 0.998, mean luma 8.42 against 8.48 unlensed; summed star luma within 7 percent; brightest star at the same pixel |
| No NaN or Inf in the HDR target | checked on both golden scenes |
| Frame time, 1920 x 1080, 4x MSAA, disk on, 512 steps, 65 frames | GPU median 1.07 ms (933 fps), p95 1.16 ms (spec: at least 30 fps, i.e. below 33.3 ms; the test requires p95 below 33.3 ms) |

The core tests (`tests/core/test_schwarzschild_geodesic.cpp`) check the
reference on its own: deflection within the second-order expansion
4/b + 15 pi/(4 b^2) (remainder below the third-order term plus 2e-8 rad) for
b = 1000 .. 5000 M, the strong-deflection limit -ln(b/b_c - 1) - 0.4002
(Bozza 2002) at 1 and 0.1 percent above b_c, capture just below b_c and
escape just above it, g against the emitter four-velocity, and the
temperature profile.

Golden snapshots (`ORRERY_UPDATE_GOLDEN=1 orrery_test_render "[golden]"`,
SSIM > 0.98):

| Reference | Size | Scene |
|---|---|---|
| `black_hole_einstein_ring.png` | 640x360 | lone 4.3e6 Msun hole, camera 60 M away along the galactic plane, disk off, Milky Way x3, one bright point-like star (V = -4.5, 9500 K) exactly behind the hole: Einstein ring, shadow, lensed band |
| `black_hole_disk_doppler.png` | 640x360 | same hole, camera 80 M away and 9 degrees above the disk, default settings: thin disk with the far side lensed over the shadow, the underside below it, the photon ring and the Doppler-bright left side |
