# Orrery Observability

Profiling zones, the Tracy integration and the `--trace` output (spec
section 10, "GOZLEMLENEBILIRLIK"; the stack table lists the Tracy client,
off in release). Phase 10a provides the zones and the trace file; the ImGui
overlay of the same spec node belongs to the user interface work.

## 1. Build options

| CMake option | Default | Effect |
|---|---|---|
| `ORRERY_PROFILING` | `OFF` | Compiles the zone macros in. Off, every macro expands to `static_cast<void>(0)`. |
| `ORRERY_TRACY` | `OFF` | Every zone is also a Tracy zone; requires `ORRERY_PROFILING`. Adds the vcpkg manifest feature `profiling` (the Tracy client, `default-features: false`, `on-demand`). |

Both are off in `debug-asan`, `release`, `ci-headless` and their Linux
counterparts, so those presets build exactly as before and install the same
vcpkg ports. The `profile` and `linux-profile` presets (RelWithDebInfo, not
used by CI) turn both on:

```
cmake --preset profile && cmake --build --preset profile
```

A build with the built-in sink but without Tracy needs no extra package:
`cmake --preset ci-headless -B build/prof -DORRERY_PROFILING=ON`.

The sink and the trace writer live in `orrery_profile` (`src/core/profile`),
a graphics-free library that `orrery_core` links publicly, so every module
can place zones. `tools/check_deps.py` lists it in the target graph, among the
graphics-free targets and in the precision check (no single precision), and
approves `Tracy::TracyClient` for it alone.

## 2. Zones

```cpp
#include "orrery/core/profile.h"

bool PanelSolver2D::step() {
    ORRERY_ZONE("panel_solve"); // measures the rest of the scope
    ...
}
```

`ORRERY_FRAME_ZONE()` opens the zone `frame` and, with Tracy, marks the frame
boundary (`FrameMark`). Rules: the name is a string literal (it is stored by
address; anything else does not compile); one zone per function, since a
zone in a nested scope of the same function would shadow the outer one
(`-Wshadow`, C4456; Tracy's `ZoneScoped` has the same limit), so nest by
calling a function; and zones go around whole steps and solves, never inside
inner loops.

| Zone | Where | Measures |
|---|---|---|
| `physics_step` | `core::EventStepper::step` | one space physics step: integrator, force model, encounter phase |
| `tree_build` | `core::Octree::build` | one Barnes-Hut octree build (once per force evaluation) |
| `lbm_step` | `step(count)` of the CPU and GPU D2Q9 and D3Q19 solvers | one call of `count` lattice steps |
| `panel_solve` | `PanelSolver2D::step`, `PanelSolver3D::step` | one panel-method solve (influence matrix and linear system) |
| `bh_raymarch` | `render::detail::BlackHolePass::draw` | the black-hole pass: geodesic dispatch and composite |
| `import` | `geom::import_mesh_file` | one model import (read, pre-scan, assimp, validation) |
| `entry_step` | `atmo::EntryExperiment` step loop | one accepted entry integration step (with its event bisection) |
| `frame` | window loop, headless image and headless benchmark loop | one frame |

Times are CPU wall-clock times of the calling thread. GPU work is
asynchronous: `lbm_step` on the GPU and `bh_raymarch` measure the submission
unless the code waits for the GPU inside the zone (the headless benchmark's
GPU timer reports device time separately). `import_mesh_memory` called
directly is not zoned (the application always imports through
`import_mesh_file`).

**Determinism.** A zone only reads the steady clock and, when a capture is
active, appends to the sink under a mutex; it never touches the state it
measures. The determinism test (`tests/profile/test_profile_determinism.cpp`)
runs 20 Barnes-Hut steps of 300 bodies with and without a capture and
requires the same `state_hash`; the core golden-hash tests pass with
`ORRERY_PROFILING` on and off.

**Cost.** Off (the default), a zone is `static_cast<void>(0)`: no object, no
call, no clock read; `tests/profile/test_profile_off.cpp` checks the
expansion text. On, an idle zone (no capture) costs one function call and one
relaxed atomic load; a recording zone two steady-clock reads and a short
locked append. Measured on the development machine (section 6): about 3 to
4 ns per idle zone and 70 ns per recorded zone, against microseconds to
milliseconds for the work a zone surrounds.

## 3. `--trace <file>`

```
orrery_app --headless --experiment solar_system --frame 600 --out a.png --trace a.json
```

`--trace` works with every run mode, in any build, headless included: the
sink is independent of Tracy. The capture starts after the command line is
parsed and stops when the run ends; the file is then written even when the
run failed (exit code 3, no OpenGL context, still leaves the physics zones of
a headless run). A trace that cannot be written is reported as
`orrery_app: cannot write the trace <file>: <reason>` and turns a successful
run into exit code 1. Without `ORRERY_PROFILING` the file is a valid trace
with no zones (`otherData.zones_compiled` is `"false"`).

## 4. Trace file format

Chrome trace-event JSON, "JSON Object Format", UTF-8, written by
`profile::chrome_trace_json` (`orrery/core/trace_capture.h`). It opens in
`chrome://tracing`, in the Perfetto UI (ui.perfetto.dev) and in Tracy through
`tracy-import-chrome`.

```json
{"displayTimeUnit":"ms",
"traceEvents":[
{"name":"process_name","ph":"M","pid":1,"tid":0,"args":{"name":"orrery_app"}},
{"name":"thread_name","ph":"M","pid":1,"tid":1,"args":{"name":"thread 1"}},
{"name":"tree_build","cat":"zone","ph":"X","pid":1,"tid":1,"ts":812.204,"dur":41.310},
{"name":"physics_step","cat":"zone","ph":"X","pid":1,"tid":1,"ts":790.115,"dur":96.882}
],
"otherData":{"format":"orrery-trace","version":"1","zones_compiled":"true","tracy":"false",
"duration_us":"15022.512","events":"2","dropped":"0"},
"zoneTotals":[
{"name":"physics_step","count":1,"total_us":96.882,"mean_us":96.882,"min_us":96.882,"max_us":96.882},
{"name":"tree_build","count":1,"total_us":41.310,"mean_us":41.310,"min_us":41.310,"max_us":41.310}
]
}
```

- `traceEvents`: first the process name (metadata event, `ph` `"M"`), then one
  `thread_name` per thread that recorded a zone (threads are numbered 1, 2, ...
  in the order they first completed a zone), then one complete event (`ph`
  `"X"`) per zone in completion order, so an inner zone precedes its outer
  zone. `ts` is the start and `dur` the duration in microseconds since the
  capture started, with exactly three decimals (nanosecond resolution, no
  rounding); `pid` is always 1, `cat` always `"zone"`.
- `otherData`: string values, as Chrome writes them. `format` is
  `orrery-trace`, `version` `1`; `zones_compiled` and `tracy` describe the
  build; `duration_us` is the capture's length; `events` the number of
  complete events in the file; `dropped` the zones beyond the cap (or lost to
  an allocation failure).
- `zoneTotals`: one entry per zone name, sorted by name, with `count` and the
  total, mean, minimum and maximum duration in microseconds. Totals include
  dropped events. Nested zones of the same name would both count; the zones
  above never nest with themselves.

A capture keeps at most 2^20 events (about 32 MB); later zones only update
the totals. Zones open when the capture stops, or started before it, are not
recorded.

## 5. Tracy

With `ORRERY_TRACY` every zone is a Tracy `ZoneScopedN` zone and every
`ORRERY_FRAME_ZONE` a `FrameMark`. The client is built `on-demand` (it
collects only while a profiler is connected, so unattended runs and tests do
not accumulate data) and without the crash handler (the terminate handler of
`docs/app.md` stays in charge). Connect a Tracy profiler of the same version
as the client (0.15.0 from the vcpkg baseline; the GUI is not part of the
manifest). Set the environment variable `TRACY_ONLY_LOCALHOST=1` to make the
client listen on the loopback interface only (the test runs do), and
`TRACY_PORT` to choose its port.

## 6. Tests and measurements

`orrery_test_profile` (links `orrery_core` only): the sink with the macros on
(nesting, durations, threads, the event cap, restarts, zones open at stop),
the empty expansion with the macros off, the JSON writer parsed back with
nlohmann_json (structure, three-decimal microseconds, escaping, totals, write
errors) and the determinism test. The `profile.trace_*` ctests run
`orrery_app --trace` on the space, black-hole, aero panel and entry
experiments and parse the file with CMake's `string(JSON)`: every complete
event has a name and numeric `ts`/`dur`, the totals agree with the events,
and with `ORRERY_PROFILING` the expected zones are present (`frame` and
`bh_raymarch` only when the run rendered).

The whole suite passes with `ORRERY_PROFILING` and `ORRERY_TRACY` on
(ci-headless flags, warnings as errors), including the golden-hash tests of
core, Barnes-Hut and IAS15, and `tools/check_deps.py` accepts that tree.

**Measured** (Release MSVC, AMD Radeon RX 9060 XT, Windows; the steady clock
there ticks in 100 ns, so the last two decimals of `ts` and `dur` are zero):

| Measurement | Result |
|---|---|
| `orrery_test_profile "[benchmark]"`, sink only | 2.8 ns per idle zone, 74 ns per recorded zone |
| the same with Tracy (on-demand, no profiler connected) | 4.0 ns idle, 68 ns recorded |
| `--headless --experiment solar_system --frame 600 --benchmark 100 --size 960x540 --trace` | 4628 `physics_step` (2314 per advance, image and benchmark), mean 2.6 us; 111 `frame`, mean 0.98 ms |
| `--headless --experiment black_hole --benchmark 50 --size 960x540 --trace` | 61 `bh_raymarch`, mean 14 us (submission); 61 `frame`, mean 1.9 ms |
| `--headless --experiment aero_naca0012_lbm --size 320x180 --trace` | 3200 `lbm_step` calls (32000 lattice steps), mean 26.8 ms; trace 0.3 MB |
