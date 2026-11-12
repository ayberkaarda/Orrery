# Orrery Rendering Reference

Conventions and design of `orrery_render`. Phase 4a provides the foundation:
camera, OpenGL context and object wrappers, shader embedding, the HDR chain,
headless snapshots, PNG output and snapshot comparison. Phase 4b adds the
space scene (sections 10 to 12: bodies, starfield, trails), the immutable
`SimSnapshot` the renderer reads (section 9) and the Dear ImGui / ImPlot
layer (section 13). The physics thread, the triple buffer and the
application loop come in Phase 4c (ADR-005, ADR-011); bloom, rings,
atmospheres and black-hole lensing later.

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
| Orbit | target body or fixed focus, distance, yaw, pitch | eye at `focus + distance (cos p cos y, cos p sin y, sin p)`, looks at the focus, world +Z up |
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
The `--headless` application flags and EGL/Mesa contexts for CI (spec 7.7)
arrive with the application loop.

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
reports core graphics-free and float-free). Handing snapshots from the
physics thread to the render thread (triple buffer) is Phase 4c.

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
black hole uses its shadow radius `sqrt(27)/2 r_s`.

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
work with. Until then an analytic glare sprite (core plus wide skirt, sized
from the disc) marks them. **Black holes** are a black disc of the shadow
radius with a faint warm ring sprite (no lensing; Phase 5). **Probes** are
small emissive cyan markers.

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
Gaussian disc that thickens towards the centre, a bulge, star clouds and fine
grain from value noise, and a meandering dust lane; surface brightness at most
about 0.04 scene-linear, which the ACES toe keeps subtle. Tested: the band is
more than twice as bright towards the centre as towards the anticentre, and
dark at the galactic pole.

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
| `draw_render_settings` | `RenderSettings` (scale mode, minimum size, LOD bias, star glare, trails, starfield, Milky Way, exposure in EV, light falloff) | `changed` (sanitised copy) |
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

## 14. Phase 4b golden snapshots and tests

Same procedure as section 8 (`ORRERY_UPDATE_GOLDEN=1 orrery_test_render
"[golden]"`, SSIM > 0.98, actual images in `<build>/tests/render_output/`):

| Reference | Size | Scene |
|---|---|---|
| `space_solar_system.png` | 960x540 | J2000 solar system loaded with orrery_scene, oblique Orbit camera 12.5 AU from a point 4.2 AU from the Sun, 40 degree FOV, trails from a 150-year backward leapfrog run (600 samples, 85 percent of each orbit), starfield and Milky Way |
| `space_earth_moon_true_scale.png`, `space_earth_moon_clamped.png` | 640x360 | the J2000 Earth from 32,000 km with the Moon beside it, 55 degree FOV, no trails; true scale and clamped (min. size x2) |
| `ui_space_workspace.png` | 1280x720 | the docked panels (Earth selected, drift plot of the backward run) and scene labels over the solar system |

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
panels are docked and carry text while the scene shows in the central area.
With the compressed PNG writer (section 7) the five references take 1.05 MB
(previously 7.9 MB of stored blocks): `space_solar_system.png` 388 KB,
`ui_space_workspace.png` 490 KB, the two Earth-Moon images 72 to 73 KB each,
`test_shape.png` 22 KB.

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
| `shaders/body.frag.glsl` | black hole: unlensed black disc of the shadow radius with a faint ring | lensing renderer is Phase 5 | position and apparent size of the shadow only |
| `shaders/glow.frag.glsl` | star glare: analytic halo sprite | stands in for optical glare until the bloom pass | display only |
| `shaders/milky_way.frag.glsl` | Milky Way: Gaussian disc, bulge, noise star clouds and dust lane | no survey data | band position and overall shape, not detail |
