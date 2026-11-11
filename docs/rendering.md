# Orrery Rendering Reference

Conventions and design of `orrery_render`. Phase 4a provides the foundation:
camera, OpenGL context and object wrappers, shader embedding, the HDR chain,
headless snapshots, PNG output and snapshot comparison. Body rendering,
starfield, trails, ImGui panels, the physics thread and the application loop
come in later phases (ADR-005, ADR-011).

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

PNG output (`png.h`) is dependency-free: 8-bit RGBA, non-interlaced, zlib
stream of stored deflate blocks (no compression), CRC-32 per chunk, Adler-32
over the data. The reader accepts the same subset (any row filter) and
validates every checksum. Uncompressed files are about `4 * width * height`
bytes (300 KB for 320x240).

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

## 9. Approximations

Every item is marked `// APPROXIMATION:` in the code.

| Where | What | Why | Validity |
|---|---|---|---|
| `shaders/tonemap.frag.glsl`, `tone_map.h` | ACES RRT + ODT replaced by Hill's fitted matrices and rational curve | full CTL transforms are far too expensive per pixel | a few percent from the reference over scene-linear [0, ~16]; output clamped to [0, 1] |
| `shaders/test_shape.frag.glsl` | energy-normalised Blinn-Phong, Schlick Fresnel (F0 = 0.04), hemispheric ambient | test shape only; exercises the HDR path, not a material model | test scene only; not used for bodies |
| `camera.cpp` | rotation smoothing uses a slerp analogue of the exact scalar blend | quaternions have no linear blend | exact when lag and increment share an axis; second order in the per-update rotation otherwise; affects camera feel only |
| `camera.cpp` | Surface latitude/longitude advanced with the midpoint rule per substep of at most 1/240 s | the path along a turning heading on a sphere has no closed form | tangent-plane motion exact; second order in the substep; splits into multiples of 1/240 s give identical substeps; camera placement only |
