# Orrery Application Reference

The application layer of V1 (spec section 8, "UYGULAMA", and 7.7; Phases
4c, 8b, 9b and 10): the physics thread, the command channel, the time-warp
budget, the interactive window with its workspaces, panels and keyboard
shortcuts, the configuration, the error surface and the headless command
line. Rendering itself is described in `rendering.md`.

## 1. Targets

| Target | Kind | Links | Contents |
|---|---|---|---|
| `orrery_sim` | static library | core, scene (+ oneTBB, private) | `Simulation`, `PhysicsThread`, `TripleBuffer`, `SimCommand`, `CommandQueue`, space experiments |
| `orrery_app_core` | static library | sim, render | command line, view policy, headless renderer |
| `orrery_app` | executable | app_core | `main`, and the window (`window_app.cpp`) only when `ORRERY_HEADLESS` is off |

`orrery_sim` is graphics-free: `tools/check_deps.py` lists it with core,
geom and atmo as a target that must never reach OpenGL, GLFW or ImGui, and
`orrery_test_sim` links it alone so that its final link line is checked too.
Headers live under `src/app/include/orrery/{sim,app}`.

## 2. Fixed-timestep accumulator

`sim::Simulation` owns the `SimState`, the integrator (`leapfrog_kdk`,
`yoshida4`, `rk4`, `ias15`), the force model and an `EventStepper` (tidal
disruption, horizon absorption, collisions after every step). The force
model `auto` picks Paczynski-Wiita when the system holds a black hole,
Barnes-Hut above 256 bodies and Newtonian direct summation otherwise.

`advance(wall_ns)` credits wall-clock time and takes the physics steps now
owed. The accumulator is integral (ADR-016):

- owed simulated time is an `int64` count of nanoseconds;
- a call adds `wall_ns * time_warp` exactly (the warp is an integer in
  [1, 1e7] simulated seconds per wall-clock second; `wall_ns` is clamped to
  [0, 250 ms] so a stalled caller cannot ask for an unbounded catch-up);
- one step consumes `round(dt_days * 86400e9)` nanoseconds, while the
  integrator always steps by exactly `dt_days`.

So the number of steps after a sequence of calls depends only on the total
wall time credited, not on how it was split into frames, and the state is
bit-identical for any frame pacing (tested with 60 Hz, 144 Hz, irregular
1 to 50 ms frames and 250 ms chunks: same step count and `state_hash`), and
bit-identical to calling the integrator directly the same number of times.
This holds while only the count budget limits a call: the wall-clock step
budget and the time-warp budget below make the number of steps in a given
wall time depend on measured costs, and runs then agree after the same
number of steps (tested: two governed runs at different step costs take
different step counts over the same wall time and reach the same
`state_hash` once their counts match).

**Step budget.** One `advance` takes at most `max_steps_per_advance` steps
(experiment setting `max_steps_per_frame`, default 4096) and, interactively,
at most 20 ms of wall-clock time. Simulated time the budget cannot cover is
dropped (only the fraction of a step is kept), so a warp the machine cannot
sustain never builds up debt (no spiral of death). The status then reports
`warp_limited` and `effective_warp`, the rate actually achieved (steps taken
times the step length per wall-clock time); the application bar shows it as
a `LIMITED` chip. The wall-clock budget makes the step count machine
dependent, so the headless renderer and the tests use the count budget only.

**Time-warp budget** (Phase 10, `warp_governor.h`). The step budget only
cuts a tick short; the window also keeps the physics thread within a share
of wall-clock time (`[space] physics_budget`, default 0.6) by lowering the
warp it credits. `WarpGovernor` smooths the measured wall time per step
(time constant 0.25 s) and the share of time spent stepping. The warp that
would use exactly the budget is `budget * step_ns / cost_per_step`; the
credited warp is the highest 1-2-5 level (1, 2, 5, 10, ... 1e7) below it,
never above the request:

- it drops once the load has been over the budget for 0.25 s, straight to
  the level that fits (several levels at once if need be);
- it rises once the next level would have used at most 75 percent of the
  budget for 1.5 s, to the highest level that keeps that margin.

The two thresholds and the holds are the hysteresis: a load that jitters
around a level boundary does not make the warp flicker (tested with a
+-8 percent jitter for 30 s: one change). A lower request applies at once, a
higher one as far as the measured cost allows; a new scenario forgets the
cost. The status carries the requested warp, the governed one
(`governed_warp`, `warp_reduced`), the load and the budget; `warp_limited`
is set while either budget holds the warp down. Only the warp changes,
never the step, so the state after N steps is bit-identical with or
without the governor (tested: a governed run whose warp dropped and
recovered against a plain run of the same number of steps, same
`state_hash`). The governor reads no clock: each tick hands it the wall
time, the busy time and the step count, and `SimulationConfig::clock`
replaces the step clock, so the tests drive it with a fake clock. Headless
runs and tests without a `warp_budget` credit the requested warp as before.

What the user sees: the bar's `LIMITED 2e5x` chip (its tooltip gives the
requested warp, the cost of one step and the load against the budget, and
says the warp returns by itself), and in the System Monitor a `TIME WARP`
tile `2e5x / 1e7x` in the warning colour with a one-line reason under the
tiles.

**Commands** (`sim_command.h`), applied in order at the start of a tick:

| Command | Effect |
|---|---|
| `SetPaused{bool}` | pause or resume; resuming never catches up the paused time |
| `SetTimeWarp{int64}` | clamped to [1, 1e7] |
| `LoadScenario{path}` | reads and builds the scenario on the physics thread; on failure the current system stays and the status carries the message |
| `Reset{}` | the initial state of the scenario loaded last (fresh integrator, so IAS15 history never leaks) |
| `SingleStep{}` | pause, then exactly one step |
| `LoadExperiment{path}` | reads a space experiment on the physics thread and replaces the system with its scenario and its settings (integrator, force model, step, warp, paused); the interactive budgets stay. On failure nothing changes and the status carries the message |

A failing step (non-finite state, IAS15 underflow) leaves the state as it
was (integrator contract), pauses and reports the reason. Load and reset
increment `generation`, which tells the renderer to drop trails and plots.

The status also carries the mean step time, the event count and the relative
energy drift since the scenario started (`ConservationDiagnostics`; skipped
above 2048 bodies, where the O(N^2) potential costs more than a frame).

## 3. Physics thread

`sim::PhysicsThread` (ADR-011) runs one `Simulation` on a `std::jthread`.
The render thread talks to it through three channels, none of which can make
a frame wait:

| Direction | Channel | Mechanism |
|---|---|---|
| render -> physics | `send(SimCommand)` | `tbb::concurrent_queue` (`CommandQueue`, pimpl: no public header includes oneTBB) |
| render -> physics | `wake()` once per frame | `std::counting_semaphore::release`, non-blocking |
| physics -> render | `fetch()` / `latest()` | `TripleBuffer<PublishedFrame>`: `shared_ptr<const SimSnapshot>` plus `SimStatus` |

A tick waits for a wake-up or the idle tick (8.3 ms, for a minimised window
or tests) and drains the pending wake-ups. It first advances by the
wall-clock time measured since the previous tick, under the state that held
during that time, and only then applies the commands that arrived meanwhile,
so no command acts retroactively (resuming never credits the paused
interval, a new warp never applies to the time before it). Command work is
bounded per tick (256 commands, of which 32 single steps, each a physics
step outside the step budget); the rest stays queued for the next tick,
which is requested at once, and cancellation is checked between commands so
a backlog neither delays shutdown nor starves publication. A frame is
published only when something changed. The
snapshot sequence equals the step count, so a reader can tell new states
apart. Exceptions never leave the thread: they pause the simulation and are
published as the status message.

**Triple buffer.** Three slots rotate between producer (back), middle and
consumer (front). `publish()` swaps back and middle and sets a fresh bit with
one `exchange` (acq_rel); `fetch()` swaps a fresh middle with front the same
way. Neither side ever waits, the consumer always reads a complete value,
and values published between two fetches collapse into the newest. The
acquire/release pairs make every slot hand-over a happens-before edge in both
directions, so there are no plain data races; the stress test (200,000
payloads of 32 words written in place, reader checking every word and the
order) is written to be run under ThreadSanitizer as well. Slots hold
`shared_ptr`s, so a snapshot stays alive while any reader holds it and is
released when the last reference goes (tested).

## 4. Experiments

A space experiment is an `orrery/scene` experiment record with domain
`space` (format in `src/app/include/orrery/sim/space_experiment.h`):
`inputs.scenario` (relative to the experiment file), `conditions.time_warp`,
`conditions.paused`, `solver.name` (the integrator) and `solver.settings`
`dt_days`, `force`, `softening_au`, `max_steps_per_frame`. Unknown keys and
out-of-range values are rejected with the file and key in the message.
Experiment and scenario paths that exist but are not regular files (FIFOs,
sockets, devices, directories) are refused as `not a regular file` before
anything opens them, so a listing, an Open or a `LoadScenario` naming one
neither blocks the UI nor the physics thread and its shutdown (tested with
a FIFO without a writer on POSIX).

Aero experiments (Phase 8b) use the same schema, version 1, with domain
`aero`: a geometry reference, the free-stream conditions, the solver choice
and the result, plus the optional top-level `series` key (named arrays of
finite numbers: residuals, force history) that older files simply lack. The
typed record is `orrery/scene/aero_experiment.h`; keys and examples are in
[aero-workspace.md](aero-workspace.md#experiments). The command line and the
window look at the domain first: an aero file opens the Aero workspace or
the aero headless renderer, an entry file (Phase 9, domain `entry`, the typed
record `orrery/scene/entry_experiment.h`) the Entry workspace or the entry
headless renderer ([entry-workspace.md](entry-workspace.md)), anything else
goes to the space loader.

Bundled experiments (`assets/experiments`):

| Name | Scenario | Integrator, dt | Warp |
|---|---|---|---|
| `solar_system` | J2000 solar system | leapfrog, 0.05 d | 1e6 (11.6 d/s) |
| `sun_earth_moon` | Sun, Earth, Moon | leapfrog, 0.02 d | 2e5 |
| `figure_eight` | three-body choreography | Yoshida4, 0.5 d | 2e6 |
| `sgr_a_s2` | Sgr A* and S2 | IAS15, 2 d | 1e7 |
| `black_hole` | a lone 4.3e6 Msun Schwarzschild hole (its thin disk is a rendering model, `rendering.md` section 17) | leapfrog, 1 d | 1 |

Bundled aero experiments: `aero_naca0012_panel` (2D panel method, 4 deg),
`aero_naca0012_lbm` (2D lattice Boltzmann at Re 400, 4 deg),
`aero_naca2412_wing` (3D panel method, 5 deg) and `aero_apollo_capsule`
(Newtonian impact theory at Mach 20); see aero-workspace.md. Bundled entry
experiment: `entry_apollo_like` (the ballistic Apollo-like lunar return, 11
km/s at -5.7 deg, about 6.8 g); see entry-workspace.md.

A name given on the command line is looked up as `<assets>/experiments/<name>.json`,
where `<assets>` is `assets/` next to the executable if present (located
with GetModuleFileNameW, /proc/self/exe or _NSGetExecutablePath; argv[0] only
as a fallback, since a bare name started through PATH says nothing about it), then
`../share/orrery/assets` from the executable's directory (an installed tree,
[packaging.md](packaging.md)), then the source tree's `assets/` recorded at
build time (not in package builds: `ORRERY_SOURCE_ASSET_FALLBACK=OFF`); an
existing file path is used as is.

## 5. Command line

```
orrery_app [--experiment <name-or-path>] [--size WxH] [--benchmark <frames>]
orrery_app --headless --experiment <name-or-path> --out <file.png> [--frame <N>] [--size WxH]
           [--benchmark <frames>]
orrery_app --version | --help | --modules
```

`--frame N` (default 0, at most 1e6) advances N frames of 1/60 s at the
experiment's warp. Values may follow the option or an `=`; repeated, unknown
or misplaced options are errors. Every error is one line on stderr prefixed
`orrery_app:`. Exit codes: 0 success, 1 failure (missing experiment or
scenario, invalid experiment, render or write failure), 2 invalid command
line, 3 no OpenGL 4.6 context. A headless build (`ORRERY_HEADLESS`) has no
window and answers the windowed form with exit code 2.

`--benchmark N` measures frame times. In the window it renders N frames
without vsync (after 30 warm-up frames) and prints the mean. With
`--headless` it first writes the image as usual, then draws the same view N
more times offscreen (after 10 warm-up frames; scene, resolve and tone map,
no readback) and prints the mean wall time per frame from the first command
until the frame's GPU timer query completes, the fps that follows, and the
GPU median and 95th percentile. This is the measurement that never opens a
window (`app::benchmark_headless`).

`--trace <file>` (any mode) writes the profiling zones of the run as Chrome
trace-event JSON with per-zone totals; zones are compiled in only by
`ORRERY_PROFILING` builds. Format and options: [observability.md](observability.md).

## 6. Headless rendering

`app::render_headless` advances the experiment with the same `Simulation`
the physics thread runs (count budget only, so the result is deterministic),
records trails from every new snapshot exactly as the window does, frames
the final state with the window's framing camera and draws it with
`SpaceRenderer` through the offscreen HDR pipeline (`OffscreenRenderer`,
hidden window). The PNG writer is the dependency-free one of `png.h`.

The golden test renders `solar_system` at frame 600 (2314 steps, 115.7 days)
at 960x540 and requires SSIM > 0.98 against
`tests/app/golden/headless_solar_system.png`; it writes
`<build>/tests/app_output/headless_solar_system_actual.png` on every run and
the reference only with `ORRERY_UPDATE_GOLDEN=1 orrery_test_app "[golden]"`.
Without an OpenGL 4.6 context the `[gl]` tests skip, or fail with
`ORRERY_REQUIRE_GL=1`; the ctest `app.headless_png`, which runs the
executable end to end, reports exit code 3 as skipped.

## 7. Space view

Shared by the window and the headless renderer (`space_view.h`):

- Framing: an Orbit camera at azimuth 72 deg and elevation 30 deg (40 deg
  vertical field of view) around the dominant body (one with at least half
  of the mass), else around the fixed centre of mass. The framed radius is
  the median distance of the other bodies (APPROXIMATION: framing heuristic),
  so the solar system opens on the inner planets; the wheel zooms out. A
  black hole alone (the `black_hole` experiment) is framed for its accretion
  disk instead: 12 deg above the disk plane, at the distance where the disk
  (30 G M / c^2) fits the view, limited by width up to about 2.35:1 and by height for wider
  views, and never closer than 1.08 disk radii. In the window the framing is
  computed for the scene area left by the panels.
- Trails: `trail_length` samples per body, each spanning about 85 percent of
  its orbit (`min_step_angle = 2 pi 0.85 / length`), recorded from every new
  snapshot and cleared on load and reset.

## 8. Window

The window (GLFW, OpenGL 4.6 core, vsync) runs one loop per frame: poll
events, wake the physics thread, fetch the newest frame (new generation:
clear trails and the drift plot, reframe), build the UI, update the camera,
draw the space view into the HDR pipeline, tone map, draw the UI into the
sRGB target, `HdrPipeline::present` it to the window and swap. The HDR
pipeline is rebuilt when the framebuffer size changes; a minimised window
renders nothing while the simulation keeps running.

**Application bar** (`ui_app_bar.h`, the viewport's main menu bar, so the
dock space starts below it): orbit mark and word mark, the workspace tab
(`Space`; switching workspaces is in the strip below the bar, section 9), transport
(reset, run/pause, single step; vector glyphs, tooltips), the time warp
(arrows step through 1-2-5 presets from 1x to 1e7x, the value opens the
preset list with the simulated rate per second; the achieved rate in d/s or
yr/s), a `LIMITED` warning chip while the step budget limits the warp (held
0.75 s so it does not flicker), the scenario selector (the scenarios next to
the experiment's scenario file), and right-aligned the simulation message in
the warning colour, the simulated time and the frame rate. It follows the
theme of `rendering.md` section 13.

The bar adapts to the window width. Below 80 em (1200 px at the 15 px font)
it drops the `WARP` and `SCENARIO` captions, narrows the scenario selector
from 13 to 10 em and, while the `LIMITED` chip shows, leaves the achieved
rate to the chip. The scenario name is shortened with an ellipsis to the
selector's width (the full name is in its tooltip). On the right the
simulated time always shows; the message is shortened to the room left (at
most 72 bytes, cut at a UTF-8 character boundary, full text in its tooltip)
and the frame rate is the first item dropped when space runs out. Checked
at 1024 px by the multi-size render test of `rendering.md` section 14.

**Input:** left drag orbits the camera, the wheel zooms; keys come from the
shortcut table (section 12), only when ImGui does not want the pointer or
keyboard. The Body Inspector's Focus orbits the selected body at twelve
radii. The System Monitor shows the simulated time, the time warp (achieved
against requested while the budget holds it down), the body count, the
frame and physics step times, the integrator and the energy drift series
(decimated by two at 4096 points).

**Experiment panel** (Phase 10, `space_experiment_panel.h`; spec 8
"Scenario/Experiment"), docked as a tab beside the System Monitor: on the
left what runs (scenario; the experiment: the one the window started with,
the one opened last, or none after a scenario picked on its own, told
apart by `SimStatus::source`; integrator and step, force model, warp) and
Save with a name field; on the right the saved space
experiments of the experiments folder and the bundled ones, with Open (or a
double click) and Refresh. Save writes `sim::space_record` - the scenario
file (relative to the experiments folder when both share a root), the
requested warp and paused flag, integrator, step, force model, softening and
step budget, and as result the simulated time, step and event counts and
the energy drift - to a new file (`<name>.json`, `<name>_2.json`, ...),
through a temporary file that is renamed into place. Open sends
`LoadExperiment`. Unreadable space experiments are listed as problems,
files of other domains are skipped. A missing folder is simply empty, but a
folder path that is not a directory or a folder that cannot be listed is
shown as a problem, not as an empty library.

**Measured:** `orrery_app --size 1920x1080 --benchmark 600` (vsync off, 30
warm-up frames) on an AMD Radeon RX 9060 XT, Release MSVC: 600 frames at
1920x1061 (the window was clamped by the desktop), solar system with UI,
mean 2.0 ms, 499 fps (spec target: 60 fps at 1080p). The benchmark averages
the measured frame times; interactive updates use the same delta clamped to
250 ms (`frame_stats.h`).

**Measured (headless, black hole):** `orrery_app --headless --experiment
black_hole --size 1920x1080 --out bh.png --benchmark 300`, Release MSVC, AMD
Radeon RX 9060 XT: mean 1.51 ms per frame (663 fps), GPU median 1.29 ms,
p95 1.40 ms, lensing on (spec target: at least 30 fps at 1080p). The same
command gives 0.91 ms (1098 fps) for `sgr_a_s2` (hole far away, lensing on)
and 2.67 ms (374 fps) for `solar_system` (no hole).

## 9. Workspaces

The window has three workspaces, Space, Aero and Entry (spec 8). They are
switched in a slim strip under the main bar (`workspace_strip.h`, in the
look of the bar's workspace tab, with the current workspace's caption on the
right) or with Ctrl+1 / Ctrl+2 / Ctrl+3 (section 12). Each workspace has its own main
bar, dock layout, panels and camera; the space simulation keeps its state
while another workspace is shown (the physics thread is not woken by their
frames, so a running warp continues at its idle tick rate), and the aero and
entry solver threads keep their runs. `--experiment` with an aero or entry
file opens that workspace with it; otherwise the window opens on the
workspace of the configuration. The Aero workspace is described in
[aero-workspace.md](aero-workspace.md), the Entry workspace (Phase 9b) in
[entry-workspace.md](entry-workspace.md).

The strip is drawn by the application rather than in the main bar because
`render::draw_app_bar` shows only the Space tab; a workspace list in
`AppBarData` would let the tabs move into the bar.

## 10. Configuration

`config.toml` in the platform configuration directory (`config.h`, read with
toml++): Linux `$XDG_CONFIG_HOME/orrery` (else `~/.config/orrery`), macOS
`~/Library/Application Support/orrery`, Windows `%APPDATA%\orrery\config`.
Saved data (aero experiments, the crash log) go to the data directory:
Linux `$XDG_DATA_HOME/orrery` (else `~/.local/share/orrery`), macOS
`~/Library/Application Support/orrery`, Windows `%APPDATA%\orrery\data`;
downloaded models to the model cache of url-import-security.md.

```toml
[paths]
experiments = "C:/lab/orrery"   # default: <data dir>/experiments
model_cache = "D:/cache/models" # default: the platform model cache
[window]
width = 1600                    # 320 .. 8192 px
height = 900
workspace = "entry"             # "space" (default), "aero" or "entry"
[aero]
steps_per_frame = 200           # 1 .. 100000 lattice steps per frame
[space]
physics_budget = 0.5            # 0.05 .. 0.95 of wall time (time-warp budget)
[shortcuts]
save = "F5"                     # one chord
help = ["F1", "?"]              # or a list of them
open = []                       # or none: unbound
```

A missing file means the defaults. Unknown keys and invalid values are
reported on stderr as `orrery_app: config: config.toml:<line>:<column>: <key>:
<reason>` and ignored, the rest of the file still applies; a file that is not
TOML is reported with its position and the defaults are used. Command-line
options win over the file. A `[shortcuts]` entry with an unknown action, a
value that is not a chord string or a list of them, or a chord that does
not parse is reported and the action keeps its default; so is an entry
that conflicts with another action (section 12). `config_to_toml` writes
every key, the shortcut table included, so a saved file documents itself
(round trip tested). `save_config` serialises first, writes the text to
`config.toml.tmp` beside the file and renames that over it (a replacing
rename, on Windows too), so a failed save leaves the previous file
byte-identical: tested with every allocation failing in turn, a temporary
file that cannot be written and a file that cannot be replaced.

## 11. Error surface

No failure is silent (spec 8):

- Recoverable errors are shown where they happen: command-line errors as one
  `orrery_app:` line with an exit code; scenario and simulation errors in the
  space bar's message; geometry, import and download errors of the Aero
  workspace in a modal with the full detail (and "Copy details"); solver
  setup and step failures in the Aero Results panel and the Aero bar.
- `main` installs a `std::set_terminate` handler (`error_surface.h`) before
  anything else runs: it prints `orrery_app: fatal: <reason>` (the active
  exception's message when there is one) and appends a timestamped entry
  with the version to `<data dir>/crash.log` (creating the directory), says
  where on stderr, then aborts. An exception that reaches `main` is printed
  as `orrery_app: unexpected error: <what>`, also appended to the crash log,
  and ends with exit code 1 (`run_guard.h`); the next line names the log,
  or says `orrery_app: cannot write the crash report to <log>` when it
  cannot be written (tested with a log whose parent is a regular file).
- The physics, aero and entry solver threads catch every exception, of any
  type, per tick and report it in their status (the simulation pauses).
  This matters beyond tidiness: the MSVC runtime keeps `std::terminate`
  handlers per thread, so an exception escaping a worker thread there would
  end the process without the report.
- Tested in processes of their own (`orrery_terminate_probe`, which aborts):
  `app.terminate_report` (the report on stderr, exactly once),
  `app.terminate_report_out_of_memory` (the allocation-free fallback when
  every allocation fails) and `app.terminate_writes_crash_log` (the report,
  one timestamped entry with the version in a crash log whose nested
  directory the handler creates, and the line naming the log).
- AddressSanitizer reports of the sanitizer builds go to stderr as usual
  (`ASAN_OPTIONS=log_path=...` sends them to a file); the crash log does not
  capture them.

## 12. Keyboard shortcuts

One table (`shortcuts.h`) holds every key of the window; nothing else reads
the keyboard. Defaults:

| Action (config key) | Keys | Where |
|---|---|---|
| `toggle_pause` | Space | all (Space: simulation; Aero and Entry: the run) |
| `single_step` | `.`, Right | Space (one physics step), Aero (one frame) |
| `reset_view` | F, Home | all |
| `warp_down`, `warp_up` | `[`, `]` | Space: previous or next 1-2-5 warp preset |
| `reset` | Ctrl+R | Space: initial state of the scenario |
| `workspace_space`, `workspace_aero`, `workspace_entry` | Ctrl+1, Ctrl+2, Ctrl+3 | all |
| `save` | Ctrl+S | the workspace's experiment (Space: the Experiment panel's Save) |
| `open` | Ctrl+O | brings the workspace's experiments panel to the front |
| `help` | F1, `?` | the help overlay |
| `close` | Esc | closes the help; else Space releases the selected body, Entry the pinned cursor |

**Chords.** `Ctrl+S`, `Shift+F1`, `Alt+1`, `PageDown`, `Esc` name a key
position with exactly these modifiers (Ctrl, Shift, Alt; names are
case-insensitive). A single punctuation character (`?`, `[`, `]`, `.`) is
a typed character: it matches what the keyboard layout types, so `?` works
on any layout. With a modifier, punctuation names its US key (`Ctrl+[`);
a shifted symbol has no key of its own (`Ctrl+?` is refused with a hint).
Stepping and the warp keys repeat while held; the others fire once per
press. Typed characters follow the same rule: a character that arrives in a
frame in which no key went down was typed by key repeat, so a held `?`
toggles the help once while a held `[` keeps lowering the warp. No shortcut
fires while a text field has the keyboard.

**Conflicts.** Two actions that apply in a common workspace may not share a
chord; a typed character also conflicts with the key that types it on a US
layout (`.` and the period key, `?` and Shift+/), and with every chord that
holds Alt on a key that types: AltGr (Ctrl+Alt on Windows) and Option (Alt
on macOS) type characters that depend on the layout (AltGr+8 types `[` on a
German layout). At run time one key event fires one action: while a bound
key chord is down, the characters of the frame are taken as typed by it
and fire nothing. The defaults are conflict-free (tested). User bindings are applied together, so swapping two
keys works; a binding that still conflicts is reverted to its default and
reported with both actions, so the table in use never has a conflict.

**Help overlay.** F1 or `?` opens a centred, opaque window listing every
action with its keys as key caps, grouped by where they apply (the groups
of the current workspace first) and naming `config.toml` as the place to
change them; Esc, F1, `?` or its close button closes it.

Tested without a window (`test_shortcuts.cpp`): the action list, parsing
and canonical formatting (round trip), the defaults, matching (modifiers,
repeats, text input, one action per frame), conflicts across key and
character chords, overrides with a swap and a rejected conflict, the
configuration's `[shortcuts]` and `[space]` tables and their warnings, and
the overlay under a headless UI context (placed centred inside 1600x900,
gone when closed).

## 13. End to end

`test_workspace_journeys.cpp` runs each workspace from a fresh start to a
saved result that loads back, with headless UI frames and no window. Keys
are Dear ImGui key and character events queued as the GLFW backend queues
them; they reach the actions through the window's own path
(`shortcut_dispatch.h`: `frame_shortcuts`, which calls `read_key_frame`, then
`apply_shortcuts`, `apply_space_scene_input` and `apply_late_shortcuts`,
which `window_app.cpp` calls as well), with every workspace and the physics
thread alive as in the window:

- Space: a paused `sun_earth_moon` experiment on a physics thread; Space
  runs it, `]` raises the warp and again on a key repeat, a held `?` opens
  the help once and Esc closes it, Ctrl+2, Ctrl+3 and Ctrl+1 switch
  workspaces, nothing fires while a text field has the keyboard, Ctrl+S
  saves it from the Experiment panel; the saved file loads with the raised
  warp, and `LoadExperiment` opens it on the running thread.
- Aero: the default NACA 0012 section; Space runs the panel method to the
  end, Ctrl+S saves; the record lists back with its result and opens in a
  new workspace as a recorded result.
- Entry: the Apollo-like preset; Space flies it to the ground (peak 6 to
  8 g), Esc releases the cursor, Ctrl+S saves; the record lists back with
  its series and opens in a new workspace as a recorded run.

Images (offscreen, hidden context): the Space workspace with the warp
budget at work, the Experiment panel and the help overlay are written to
`<build>/tests/app_output` for inspection, and the whole Aero workspace
after a NACA 0012 panel-method run is a golden
(`tests/render/golden/aero/aero_workspace_naca0012.png`, SSIM > 0.98),
beside the Entry workspace golden of entry-workspace.md and the space
goldens of section 6 and rendering.md.
