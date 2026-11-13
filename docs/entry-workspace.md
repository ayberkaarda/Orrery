# Entry Workspace

The Entry workspace of `orrery_app` (spec 6 "Atmosphere and entry bridge",
spec 7 "Render: entry ground track", spec 8 "Entry Profile"; Phase 9b) puts
the entry physics of `orrery_atmo` (Phase 9a, [atmosphere.md](atmosphere.md))
behind one user interface and records every run as an experiment. Sources:
the graphics-free side in `src/app/entry/*.cpp` compiled into `orrery_sim`
(headers `orrery/sim/entry_*.h`), the user interface, the scene adapter and
the headless path in `orrery_app_core` (headers `orrery/app/entry_*.h`), and
the view in `orrery_render` (`orrery/render/entry_view.h`,
`entry_geometry.h`, shaders in `src/render/shaders/entry`).

## 1. Layout

| Area | Content |
|---|---|
| Main bar | word mark, `Entry` tab, Run/Pause, Stop, pace, planet, and on the right the run status (state, time and g-load while flying; outcome and peak g after) and the frame rate |
| Workspace strip | Space, Aero, Entry; Ctrl+1 / Ctrl+2 / Ctrl+3 |
| Left | **Entry Setup**: preset, planet and atmosphere, probe, Cd(Mach), initial state, integration, Run |
| Right | **Entry Profile**: outcome chip, peak readouts, the cursor readout, five stacked time series |
| Bottom | **Ground Track** (2D map), **Experiments** and **View** as tabs, the map in front |
| Centre | the 3D view with the outcome banner (top) and the legend (bottom right) |

The dock layout is built in code like the other workspaces'. Drag in the
scene orbits the turntable camera, the wheel zooms, F or Home reframes,
Space runs or pauses.

## 2. Entry Setup

- **Preset**: the library's scenarios (`atmo::apollo_like_entry`,
  `skip_out_entry`), a low Earth orbit return and an MSL-like Mars entry.
  The Apollo-like presets are placed near Apollo 11's entry interface over
  the western Pacific (-3.2 deg, 171.6 deg, heading 52 deg); with the air at
  rest in the inertial frame the place changes the ground track only.
- **Planet** Earth or Mars (`atmo::Planet` presets); **atmosphere** the
  planet's standard model (US Standard Atmosphere 1976, the Mars reference
  table) or an exponential one (surface density, scale height, temperature;
  CO2 gas properties on Mars); "Air turns with the planet" selects the
  co-rotating frame.
- **Probe**: mass, reference area, nose radius (Sutton-Graves heat flux); the
  ballistic coefficient m / (Cd A) is shown. Constant mass, no lift, heat
  shield forward (ADR-012).
- **Cd(Mach)**: a table of points (interpolated linearly in Mach, end values
  held outside the range; `atmo::AeroCoefficientTable`), edited in place, or
  **sampled from the aero solver**: `atmo::sample_cd_table` runs modified
  Newtonian theory at Mach 5, 10, 15, 20 and 25 (its validity range) on an
  Apollo-type or Orion-type capsule or a sphere whose frontal area is the
  reference area, so the coefficients need no renormalisation. Sampling runs
  as a background task; its warnings are listed, a failure opens the problem
  modal. A result that arrives after other inputs were loaded (an experiment,
  a preset) or the reference area changed is discarded with a notice, so it
  never replaces another experiment's table. The chip says which source the
  table has.
- **Initial state**: altitude, inertial speed and flight-path angle, or the
  **vacuum periapsis** of the approach orbit (the flight-path angle at the
  start altitude follows from energy and angular momentum and is shown; a
  periapsis at the start altitude below circular speed is refused, since the
  start is then the apoapsis); latitude, longitude, heading; time limit.
- **Integration**: IAS15 (tolerance), Yoshida 4 or leapfrog for the gravity
  part of the split step, the step inside the atmosphere (also the series
  spacing), end at skip-out.

Values no run can use (non-positive mass, area, nose radius, speed, steps;
an unsorted Cd table; a periapsis above the start) are listed under the
inputs and disable Run.

## 3. Solver thread

`sim::EntryRunner` owns the run on its own thread like the aero runner
(ADR-011): commands (start, pause, pace, stop) through a lock-free queue,
progress through a triple buffer. The run advances in slices of at most
2 s of simulated time and at most 1000 integration steps
(`atmo::EntryExperiment::advance` with a step bound, so a record with a tiny
time step cannot hold the thread for billions of steps) and polls its
commands between slices, so Pause and Stop act within one slice (tested
with a stepper that takes 20 ms per slice, and with the library at a 1e-9 s
step: Stop and shutdown within a second). **Pace**: 10x to 1000x real
time, or as fast as possible (batches of 12 ms of wall time). Progress is
published at most 30 times a second and on every state change: the series
so far, the summary, and once the run ended or was stopped the library's full
result record (`scene::EntryResultRecord`), which is what Save writes.
Failures (the library's checks, an integration error) end the run with the
reason in the profile panel and the bar; what was integrated before stays
shown. A stopped run keeps its trace without an outcome.

`sim::EntryStepper` is the workspace's only door to the physics
(`entry_backend.h`); `make_entry_stepper` builds `atmo::EntryConfig` from
the setup. Outcomes: `impact` is shown as **Landed**, `skip_out` as
**Skip-out**, `time_limit` as **Orbiting**.

The shown run follows the inputs: the workspace's run while it is active,
afterwards only while the inputs are the ones it ran with (the name aside);
a loaded record likewise. Changing an input clears the profile, the map and
the view until the next run. An active run stays shown while the inputs
change, and the view keeps drawing it on the planet (radius, interface,
style) it flew around, from the inputs it was started with.

## 4. Entry Profile

Readouts: outcome chip (label and colour), peak g-load and its time, peak
stagnation heat flux and its time, peak dynamic pressure, heat load,
duration, downrange. Five plots share their time axis (ImPlot subplots with
linked x): altitude [km], speed [km/s], g-load [g], dynamic pressure [kPa],
stagnation heat flux [kW/m2]. The **cursor** is shared by the plots, the map
and the view: hovering a plot or the map moves it, a click pins it, a
double click releases it; otherwise it sits at the peak g-load (at the
latest sample while flying). The cursor readout gives t, h, v, n, q and Q
there. Experiments ticked in the Experiments panel are overlaid as thinner
lines in Okabe-Ito colours (the current run is the theme's amber).

## 5. View

`render::EntryView` draws, in scene-linear HDR with log depth:

1. the planet: icosphere level 6 with a muted albedo (the ribbon stays the
   most saturated element), a soft terminator from the sun direction, limb
   darkening and a coastline-free graticule every 10 deg (one-pixel lines
   that fade where they crowd; two longitude branches keep the date line
   clean);
2. the atmosphere shell up to the entry interface: a single-colour
   exponential haze ray-marched in 16 samples (APPROXIMATION: no Rayleigh or
   Mie scattering);
3. a faint curtain from the ground track up to the trajectory (it shows the
   altitude) and the ground track itself, light with a dark outline;
4. the trajectory ribbon, coloured by g-load or heat flux with the
   sequential colour map of the aero visualisation (CIE L* monotonic, safe
   for colour-vision deficiencies) from 0.18 to 1, so the low values stay
   visible against the sky; a dark outline separates it from the planet;
5. markers: the peak of the coloured quantity (amber disc) with the cursor
   (white dot) on top.

Altitudes are exaggerated (x5 by default, View panel 1 to 20) so the
trajectory separates from the ground; the legend says so. The view frame
puts the middle of the track overhead and its flight along +x, so the
turntable camera looks across the track with the limb behind it; framing
leaves room for the banner and the legend. The legend (ImGui, bottom right)
gives the quantity, unit, colour bar with ticks, the exaggeration and the
marker key. The **Ground Track** map shows the track on a latitude and
longitude graticule (equal aspect, no coastlines), coloured like the
ribbon, unwrapped across the date line (labels back in -180..180 deg).

## 6. Experiments

A run's record is the `entry` domain of the experiment schema, typed by
`orrery/scene/entry_experiment.h` (keys in [atmosphere.md](atmosphere.md)).
The workspace maps its setup to it (`sim::entry_record`): the periapsis form
is stored resolved as the flight-path angle the run used (and the periapsis
is derived again on load), a sampled Cd table is marked by the vehicle name
`probe (Cd from the aero solver)`. A record that pairs a planet with the
other planet's standard atmosphere (`earth` with `mars`, `mars` with `us76`;
valid in the schema) is refused on load with an `inputs.atmosphere` error
instead of being rerun in a different atmosphere. Save writes the inputs
and, when the shown run used exactly these inputs and has ended, its full result record
(series and summary); the file is written to a temporary name and renamed.
The saved list reads every entry file of the experiments folder (other
domains skipped, unreadable entry files listed as problems); Load stops the
current run and shows the record. The **comparison table** lists the ticked
runs (all when none is ticked) sorted by planet and flight-path angle, with
outcome, peak g, peak heat flux, peak dynamic pressure, heat load and
downrange; **Export CSV** writes it next to the experiments (RFC 4180, full
precision, SI units in the headers), with text cells that a spreadsheet
would read as a formula (a leading `=`, `+`, `-`, `@`, tab or carriage
return) quoted with an apostrophe in front.

## 7. Headless

`orrery_app --headless --experiment <entry experiment> --out <file.png>
[--size WxH]` runs the entry to its end on the calling thread and draws the
whole workspace offscreen (`app::render_entry_headless`): bar, panels,
plots, map and the 3D view, four frames with a fixed 1/60 s step and no
pointer, so the image is deterministic for a given experiment and size. The
summary line names the experiment, planet, outcome, duration, peak g, peak
heat flux and downrange. Bundled: `entry_apollo_like`.

The golden test (`tests/app/test_entry_golden.cpp`, `[gl][golden]`) renders
the Apollo-like preset at 1600x900 and requires SSIM > 0.98 against
`tests/render/golden/entry/entry_workspace_apollo.png`; the reference is
written only with `ORRERY_UPDATE_GOLDEN=1 orrery_test_app "[golden]"`. It
also checks the outcome (landed, peak between 6 and 8 g), colour in the
scene and text in the profile panel.

## 8. Approximations and limits

- The view's altitudes are exaggerated, the haze is a single-colour layer,
  and the planet has no terrain, coastlines or oblateness.
- The plots show the series at the library's step (0.5 s inside the
  atmosphere, up to 30 s above it); the view decimates the track to 4000
  points, keeping the ends and the peak.
- A run's record is written only when the run has ended; a paused or running
  entry is saved as inputs only.
- The window's configuration accepts `workspace = "space"` or `"aero"` only;
  the Entry workspace is opened by the strip, Ctrl+3 or an entry experiment.
