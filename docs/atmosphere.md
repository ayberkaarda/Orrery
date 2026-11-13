# Atmosphere and entry bridge (orrery_atmo, Phase 9a)

`orrery_atmo` holds the atmosphere models and the entry bridge: the chain that
carries an orbiting probe from core's state through the atmosphere and the
aerodynamic coefficients back into core's integrator (ADR-012, ADR-014,
ADR-021). Everything here is SI (ADR-001) and double precision; the module
depends on `orrery_core` and, for the Cd(Mach) table and the Sutton-Graves
correlation, on `orrery_aero`. It never links OpenGL, GLFW or ImGui
(`tools/check_deps.py`).

```
core SimState (position, velocity)  --r, v-->  IAtmosphere (density, T, a, Mach)
aero solvers --sample_cd_table-->  AeroCoefficientTable (Cd(Mach), area)
AeroCoefficientTable + IAtmosphere  --drag_force-->  DragForce
AtmosphericDrag (IForceModel, velocity kick) + NewtonianGravity --> core integrator
```

The chain is built explicitly inside `EntryExperiment::create`; there are no
globals or singletons. Each link has its own tests (`tests/atmo/`), and the
end-to-end energy balance checks the coupling.

## Headers

| Header | Contents |
|---|---|
| `orrery/atmo/error.h` | `AtmoErrorCode`, `AtmoError` (code and message) |
| `orrery/atmo/atmosphere.h` | `IAtmosphere`, `AtmosphereSample`, `GasProperties`, `mach_number`, `UsStandardAtmosphere1976`, `MarsAtmosphere`, `ExponentialAtmosphere` |
| `orrery/atmo/planet.h` | `Planet` (GM, radius, rotation, Sutton-Graves constant, entry interface, atmosphere) with `earth()` and `mars()` presets |
| `orrery/atmo/aero_table.h` | `AeroCoefficientTable`, `sample_cd_table`, `default_aero_solver_factory` |
| `orrery/atmo/drag.h` | `AtmosphereFrame`, `air_relative_velocity`, `DragForce`, `drag_force`, `AtmosphericDrag` |
| `orrery/atmo/entry.h` | `EntryConfig`, `EntryExperiment`, `EntrySeries`, events, `EntrySummary`, `EntryEnergyBalance`, `ground_point`, presets |
| `orrery/scene/entry_experiment.h` | the typed `entry` experiment record (JSON) |

## Atmosphere models

`IAtmosphere::sample(altitude)` returns density, pressure, kinetic temperature
and speed of sound at a geometric altitude above the body's reference sphere.
Above `top()` the model is a vacuum (density and pressure 0, `inside = false`,
temperature and speed of sound of the top). The Mach number of a state is
`mach_number(sample, airspeed)`.

### US Standard Atmosphere 1976

Source: *U.S. Standard Atmosphere, 1976*, NOAA-S/T 76-1562 (NOAA, NASA, USAF).

0 to 86 km: the standard's own formulation. Geopotential altitude

    H = r0 Z / (r0 + Z),   r0 = 6 356 766 m,

seven layers of linear molecular-scale temperature TM(H) with the standard's
lapse rates (-6.5, 0, +1.0, +2.8, 0, -2.8, -2.0 K/km from 0, 11, 20, 32, 47,
51, 71 km'), and the hydrostatic pressure in each layer

    L != 0:  P = Pb (Tb / (Tb + L (H - Hb)))^(g0' M0 / (R* L))
    L == 0:  P = Pb exp(-g0' M0 (H - Hb) / (R* Tb))

with the exact constants g0' = 9.80665 m'/s^2, R* = 8314.32 J/(kmol K),
M0 = 28.9644 kg/kmol, P0 = 101 325 Pa, T0 = 288.15 K, gamma = 1.4. The base
pressures are computed from these constants (they reproduce the standard's
Table 4 to all seven printed figures). Density rho = P M0 / (R* TM) and speed
of sound a = sqrt(gamma R* TM / M0) depend on TM only; the kinetic temperature
is T = TM (M / M0), with the standard's M/M0 table between 80 and 86 km
(0.999579 at 86 km). The lowest layer continues to -5 km and is clamped below.

Above 86 km (to the 1000 km top): log-linear (piecewise exponential)
interpolation of the standard's tabulated pressure and density and linear
interpolation of its kinetic temperature, nodes at 86 (the layered model's own
values, so the profile is continuous), 90, 95, 100, 110, ... 1000 km. The
speed of sound there is sqrt(gamma P / rho), a continuum quantity kept only so
that a Mach number exists (the flow is rarefied).

Tests compare TM, P, rho and a at every geopotential layer boundary (0, 11,
20, 32, 47, 51, 71, 84.852 km', plus 5 km') and at Z = 5 km and 86 km with the
standard's Table I to its printed precision (half a unit of the last digit).

### Mars

A single mean profile, not Mars-GRAM. Temperature: the NASA Glenn Research
Center "Mars atmosphere model" curve fit to Mars Global Surveyor measurements
(April 1996): T = 242.15 K - 0.998 K/km h below 7 km, then the fit's upper lapse
rate (-2.22 K/km) from the continuous join at 7 km (the two published segments
differ by 0.95 K there), isothermal above 50 km (139.7 K; the fit itself falls
below 0 K near 112 km). The altitudes are geopotential on a 3389.5 km sphere
with g0 = 3.711 m/s^2. Pressure: the hydrostatic equation integrated through
these layers from the fit's surface pressure 699 Pa, with R = 192.1 J/(kg K)
(the fit's value) and gamma = 1.29 (CO2). The fit's own pressure curve
p = 699 exp(-9e-5 h) Pa is not hydrostatic with its temperature; the model
stays within 10 percent of it up to 10 km (tested). Top: 250 km.

### Generic exponential

`rho = rho0 exp(-h / H)` with a uniform temperature T (pressure rho R T, speed of
sound sqrt(gamma R T)); altitudes below 0 are clamped to 0. Defaults are
Earth-like (1.225 kg/m^3, 7.2 km, 288.15 K). It is the atmosphere of the
Allen-Eggers solution and the test bed for it. Hydrostatic consistency would
need H = R T / g, which is not enforced.

## Planets and frames

`Planet` is a sphere of `radius` (altitude 0) with point-mass gravity `gm`,
rotating about the +z axis of the planet-centred inertial frame at
`rotation_rate`. Presets: Earth (GM 3.986004418e14 m^3/s^2, mean radius
6371.0 km, 7.292115e-5 rad/s, Sutton-Graves k 1.7415e-4 kg^0.5/m, entry
interface 121.92 km = 400 000 ft, US76) and Mars (GM 4.282837e13, 3389.5 km,
7.088218e-5 rad/s, k 1.9027e-4, 125 km, the Mars model).

Frames: the body-fixed frame coincides with the inertial frame at time 0 and
then turns at `rotation_rate`. The ground track of a position r at time t is

    latitude  = atan2(z, sqrt(x^2 + y^2)),
    longitude = wrap(atan2(y, x) - omega t)   in [-180, 180) degrees,

on the spherical body (geocentric latitude). The initial state is given in the
inertial frame at time 0: altitude, inertial speed, inertial flight-path angle
(above the local horizontal), latitude, longitude and heading (clockwise from
north).

Atmosphere motion (`AtmosphereFrame`): `inertial` (default, ADR-012: the air is
at rest in the inertial frame) or `co_rotating` (the air turns with the planet,
wind omega z x r, no winds relative to the surface). The relative velocity is
v_rel = v for an inertial atmosphere and v - omega z x r for a co-rotating one.
The ground track always uses the planet's rotation, whichever the frame.

## Cd(Mach) table

`AeroCoefficientTable` holds Cd samples at strictly increasing Mach numbers and
the reference area Cd is normalised by. It is built either from literal data
(`create`, `constant`), so that the drag and entry tests do not depend on a
solver, or from the aerodynamic solvers (`sample_cd_table`).

- Interpolation: linear in Mach between neighbouring samples.
- Clamping policy: below the first and above the last sample the end value is
  held (no extrapolation) and `lookup` reports `clamped = true`. A table never
  invents a trend outside the range that was sampled or measured; for an entry
  capsule sampled at Mach 5 to 25 this means the hypersonic Cd is also used in
  the supersonic and subsonic tail of the trajectory (approximation E5).
- A single sample is a constant Cd.

`sample_cd_table` sets the free-stream speed to Mach times the speed of sound of
the request's free-stream template for each requested Mach number (default 5,
10, 15, 20, 25), asks the regime selector (`aero::select_solver`, or the
override) for a solver, builds it through the caller's `AeroSolverFactory`,
steps it until it reports a final result (or `max_steps`), and records Cd with
its provenance (solver, inside the envelope or not, warnings, steps,
convergence). Envelope warnings never block (ADR-009). A factory that returns
nullptr (the lattice Boltzmann solver needs a compute backend, so the default
factory has none) gives `solver_unavailable`; a throwing solver or a
non-positive Cd gives `solver_failed`.

Measured: Apollo command module (geom capsule primitive, heat shield forward,
20 degrees between axis and free stream, Newtonian) at Mach 5, 10, 15, 20, 25
gives Cd within 10 percent of 1.3 at every point, non-decreasing with Mach
(Cp_max rises towards 1.839).

## Drag

    F = -1/2 rho |v_rel| v_rel Cd(Mach) A,   Mach = |v_rel| / a

`drag_force` returns the force with the altitude, airspeed, Mach number, Cd,
clamping flag, dynamic pressure and atmosphere sample it came from
(`DragForce`, the typed value of ADR-014). `AtmosphericDrag` applies it to every
probe of a BodyStore whose `Probe::aero_ref` indexes its table list; other
bodies get zero. Its accelerations are in space units (force over the probe's
mass); its potential energy is zero (drag is dissipative). Mass is constant
(ADR-012).

## Integration

Core force models are functions of positions only, so drag enters through a
Strang splitting (ADR-021): a drag kick over dt/2 at fixed position (RK4 on the
velocity, each stage through `AtmosphericDrag::accelerations`, in substeps of
at most 0.05 of the drag time |v_air| / |a_drag| so that stiff drag - dense
air, a low ballistic coefficient - stays stable and dissipative; the substep
count grows only with the logarithm of dt over the drag time, and a non-finite
stage fails the step with `integration_failed` instead of being skipped), a
gravity step
over dt with an unchanged core integrator (IAS15 recommended and default;
Yoshida4 or leapfrog selectable) and `NewtonianGravity` on a two-body SimState
(planet at the origin as body 0 with mass GM / G in space units, probe as body
1), then a second drag kick. Inside the atmosphere the step is `time_step`
(default 0.5 s, also the series spacing). Above the model's top the step grows
to half the time to reach the top at the current descent rate, between
`time_step` and `max_coast_step` (default 30 s), so a start from orbit is cheap.
Runs are deterministic: identical configurations give bit-identical series,
and the series does not depend on how `advance` splits the run: the requested
durations accumulate, a step is taken once the total asked for reaches its
end, and the clock never passes that total (an `advance` shorter than the
next step takes no step and keeps the request).

### Events

Detected on every step from the altitude before and after it; the crossing
instant and every series column are interpolated linearly within the step.
A long step can pass through a level and back (a periapsis below the
interface or the ground, an apoapsis above the interface) with both ends on
the same side: when the osculating two-body orbits at both ends agree that the
extremum inside the step lies beyond the level, the step is redone in halves
(down to 1 ms) until an end shows the crossing, so such a pass is reported
(the halves add rows to the series there).

| Event | Condition | Effect |
|---|---|---|
| `AtmosphericEntryEvent` | altitude crosses the entry interface downwards (or the run starts at or below it, time 0) | none |
| `SkipOutEvent` | after an entry, altitude rises from at or below the interface to above it (so a start exactly on it heading out is an entry at time 0 and a skip-out); carries the specific energy, whether the orbit is bound and the osculating apoapsis altitude | ends the run (`skip_out`) when `stop_on_skip_out` |
| `ImpactEvent` | altitude reaches 0 | the last row is replaced by the interpolated impact row, and the state, `time()` and the drag work are interpolated back to the same instant (on the ground, not below it); ends the run (`impact`) |

The run also ends at `max_duration` (`time_limit`). An integrator failure ends
it with `failed` and an `integration_failed` error; the state is the last good
one. The atmo events are atmo's own variant; core's event bus
(collision, disruption, horizon crossing) is not involved.

### Series

`EntrySeries` (structure of arrays, one row per step): time, altitude, inertial
speed and flight-path angle, airspeed, Mach, Cd, density, dynamic pressure
1/2 rho v_rel^2, g-load |a_drag| / g0 (sensed acceleration, g0 = 9.80665),
Sutton-Graves stagnation heat flux q = k sqrt(rho / R_n) v_rel^3 (via
`aero::sutton_graves_heat_flux`), latitude, longitude, cumulative drag work and
mechanical energy m (v^2 / 2 - GM / r). `EntrySummary` keeps the peaks (g-load,
heat flux, dynamic pressure, with times; the initial row counts, so a start in
dense air reports its peak at time 0), the trapezoidal heat load, the lowest
altitude, the duration and the final speed.

### Energy balance

core's `ConservationDiagnostics` leaves test particles out of its totals
(a probe does not pull on the planet), so the entry tracks the probe's own
mechanical energy relative to the planet. `EntryEnergyBalance` reports

- `energy_change_j`: E(end) - E(0);
- `drag_work_j`: the kinetic energy removed by the kicks (exact bookkeeping of
  the split step); `residual_j = energy_change - drag_work` is the gravity
  integrator's energy error;
- `drag_work_quadrature_j`: the trapezoidal integral of F . v (inertial
  velocity) over the series, independent of the kick bookkeeping;
- `relative_error = |energy_change - quadrature| / |quadrature|`, the acceptance
  quantity (below 1 percent).

## Scenarios and measured values

Measured with MSVC 19.44 x64 Release (/fp:precise), IAS15 tolerance 1e-9,
time step 0.5 s, US76, inertial atmosphere, unless stated otherwise.

| Scenario | Result |
|---|---|
| Apollo-like ballistic (`apollo_like_entry`: 5560 kg, 3.912 m diameter, Cd 1.3, R_n 4.694 m, 11.0 km/s at 121.92 km, -5.7 degrees, heading east on the equator) | impact after 438.1 s; peak 6.78 g at 224.5 s (second pulse after a shallow loft); peak heat flux 1.47 MW/m^2; energy balance error 2.1e-5; residual of the bookkeeping 1e-3 J of 3.4e11 J |
| Same vehicle at -6.5 degrees | peak 15.3 g at 102 s |
| Same vehicle at -5.5, -5.6, -5.8, -5.9 degrees | 7.68, 6.93, 7.03, 8.22 g |
| Skip-out (`skip_out_entry`, -5.0 degrees) | one pass to 71.6 km, peak 1.07 g, leaves the interface at 225 s, bound |
| Time step 0.1 s instead of 0.5 s | peak g changes by 4e-6 to 1e-4 relative; energy balance error falls to 1e-6 |
| Yoshida4 instead of IAS15 for the gravity step | same peak g to 1e-6 |
| Co-rotating atmosphere (Apollo-like at -6.5 degrees) | 14.4 g (airspeed 10.53 km/s at the interface) |
| Allen-Eggers check (exponential atmosphere, no gravity, 10 km/s at -60 degrees, beta 300 kg/m^2, dt 0.01 s) | peak 224.6 g against 225.6 g from the formula (-0.46 percent); speed and density at the peak within 1.5 and 5 percent of V_E exp(-1/2) and beta sin(gamma) / H |
| Mars, Pathfinder-like (585 kg, 2.65 m, Cd 1.7, 7.48 km/s at 125 km, -14 degrees) | impact; peak 17.4 g; energy balance within 1 percent |
| Stiff quadratic drag (uniform air rho 1.2, Cd A / m = 1 m^2/kg, no gravity, 100 m/s, dt 0.5 s) | speed equals v0 / (1 + k v0 t), k = 0.6 /m, to the six printed figures at every step (test bound 1e-4) |

Why -5.7 degrees for the Apollo-like case: Apollo entered at about -6.5 degrees
and held 6 to 7 g with lift (L/D about 0.3, bank-angle modulation). ADR-012
excludes lift, and a ballistic capsule at -6.5 degrees peaks at about 15 g
(steep side of the corridor). The ballistic analogue that reaches the Apollo
deceleration band sits on the shallow side of the Apollo entry corridor
(about -5.5 to -5.9 degrees here); steeper is more severe, shallower skips out
(-5.0 degrees). The test "Apollo-like ballistic entry" checks 6 to 8 g at
-5.7 degrees and that -6.5 degrees exceeds the band.

## Experiment record

`orrery/scene/entry_experiment.h` maps an entry run onto the generic
experiment schema (domain `entry`, schema version 1, no change to the schema):

| Section | Keys |
|---|---|
| inputs | name, planet (`earth`, `mars`), atmosphere (`us76`, `mars`, `exponential`; exponential adds surface_density_kg_m3, scale_height_m, temperature_k), vehicle, mass_kg, reference_area_m2, nose_radius_m |
| conditions | altitude_m, speed_m_s, flight_path_angle_deg, latitude_deg, longitude_deg, heading_deg, interface_altitude_m (optional), frame (`inertial`, `co_rotating`) |
| solver | name (`ias15`, `yoshida4`, `leapfrog_kdk`), version "1", settings tolerance, time_step_s (at least 0.001 s), max_coast_step_s, max_duration_s (at most 200 000 time steps, see [Run size](#run-size)), stop_on_skip_out (0 or 1) |
| result | outcome, duration_s, peak_g_load, peak_g_time_s, peak_heat_flux_w_m2, peak_heat_flux_time_s, peak_dynamic_pressure_pa, peak_dynamic_pressure_time_s, heat_load_j_m2, min_altitude_m, final_speed_m_s, drag_work_j, energy_change_j, energy_relative_error; entry_time_s, skip_out_time_s, impact_time_s, impact_latitude_deg, impact_longitude_deg when the event occurred |
| series | cd_table_mach, cd_table_cd (the table, an input: parameters hold scalars only); with a result the `EntrySeries` columns under their member names |

Unknown keys and values no run can use are rejected with the offending key
(`inputs.mass_kg: must be positive and finite`). `to_entry_config` builds the
`atmo::EntryConfig` of a record and `entry_result` the result record of a
finished run. `make_entry_atmosphere` is the one place that builds a record's
atmosphere, for `to_entry_config` and the Entry workspace alike; the
exponential model takes the planet's gas (Earth: the library's air default;
Mars: CO2 with `MarsAtmosphere`'s gamma 1.29 and R 192.1 J/(kg K)), so the
same record has the same speed of sound, Mach number and Cd through either
door. Fixture: `tests/scene/fixtures/entry_apollo.json`.

### Run size

Every step adds a row to the series, and the series is kept whole: the
trapezoidal drag-work quadrature of the energy balance and the plots use
every row. A record therefore bounds the size of its run instead of thinning
the series:

| Bound | Value | Constant |
|---|---|---|
| Smallest `time_step_s` | 0.001 s | `scene::kMinEntryTimeStepS` |
| Most steps, `max_duration_s / time_step_s` | 200 000 | `scene::kMaxEntrySteps` |

Coast steps are never shorter than `time_step_s`, so a run has at most
200 001 rows (about 26 MB for the 15 columns and the drag power). The step
count is compared with a relative tolerance of 1e-9, so a step that is not a
binary fraction (0.02 s over 4000 s) is accepted at the limit. Both
`entry_from_experiment` (and so `parse_entry_experiment`) and
`to_entry_config` refuse a record outside the bounds with a message naming
`solver.settings.time_step_s`; a JSON record with `time_step_s` 1e-9, which
would otherwise grow its series until the run is stopped, is rejected when it
is read. The defaults (0.5 s over 4000 s) take 8000 steps. The Entry
workspace checks the same bounds on its inputs (entry-workspace.md, "Entry
Setup"). `atmo::EntryExperiment` itself does not impose them: callers of the
library choose their own steps (its tests use steps from 0.01 s to hundreds
of seconds) and bound one call with `advance(duration, max_steps)`.

Tests: "Entry time steps below the documented minimum are rejected", "Entry
runs longer than the step limit are rejected" (both boundaries), and "A run
at the step limit stays within the row cap" (200 000 one-second steps of a
circular orbit above the atmosphere, at most 200 001 rows).

## Approximations

Listed with formulas, sources and validity envelopes in docs/physics.md,
"Atmosphere and entry (orrery_atmo, Phase 9a)" (E1 to E10), next to the
project-wide A8 to A11.
