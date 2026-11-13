# Screenshots

The images in [docs/images](images) are shown in the README. They are drawn
by the application itself. The Space and Aero images come from the real
window with its docked panels, opened hidden (`orrery_app --capture`): the
window loop runs as it does on screen, but the window is never shown and
never takes the focus, and its framebuffer is read after the UI pass
([app.md](app.md#14-scripted-window-sessions)). The Entry image comes from
the headless path (`orrery_app --headless`), which draws the whole Entry
workspace offscreen. They are illustrations, not test references: the
golden images that the test suite compares by SSIM live under `tests/` and
are regenerated only through the tests (see
[rendering.md](rendering.md#8-snapshot-comparison-and-golden-images)).

## Regenerating

The window shots need a build with a window system (not `ORRERY_HEADLESS`;
the `release` preset, for example) and an OpenGL 4.6 driver; exit code 3
means no context. Run from the repository root:

```sh
# Windows
py tools\make_screenshots.py --app build\release\src\app\orrery_app.exe
# Linux
python3 tools/make_screenshots.py --app build/linux-release/src/app/orrery_app
# only some images
py tools\make_screenshots.py --app <app> --only space-black-hole.png entry-apollo.png
```

`tools/make_screenshots.py` runs one command per image (table below),
re-encodes each PNG losslessly with Pillow at the highest zlib effort
(optional; without Pillow the rendered file is kept) and drops every text,
time and profile chunk, so no tool name, user name or path is embedded. The
images stay full colour on purpose: a 256-colour palette bands the colour
maps and shifts the colours their legends explain. Each image is 0.1 to
0.6 MB.

`--capture` runs `--frame` frames at 60 per second of wall time (so 3000
frames take 50 s, and the simulation runs at the experiment's warp in real
time), starts the shown workspace's run (the aero solve) and waits for it to
finish, lets 45 more frames settle and writes the back buffer. It uses the
default configuration and an empty experiments folder of its own, and the
panels shorten the home directory to `~`, so no user setting, saved
experiment or user name appears.

| Image | Command (after `orrery_app`) | What it shows |
|---|---|---|
| `space-solar-system.png` | `--capture <png> --experiment solar_system --frame 3000 --size 1600x900` | The window after 50 s of the J2000 solar system at 1e6x (about 590 days, leapfrog, 0.05 d steps): inner planets with their trails, Body Inspector, System Monitor with the energy drift, Render Settings |
| `space-black-hole.png` | `--capture <png> --experiment black_hole --frame 0 --size 1600x900` | Schwarzschild lensing of a thin accretion disk in the window, with the black-hole section of Render Settings |
| `space-figure-eight.png` | `--capture <png> --experiment figure_eight --frame 1200 --size 1600x900` | The Chenciner-Montgomery figure-eight three-body choreography (Yoshida 4th order) after 20 s in the window |
| `aero-naca0012-lbm.png` | `--capture <png> --experiment aero_naca0012_lbm --frame 0 --size 1600x900` | Aero workspace after a D2Q9 lattice Boltzmann run to completion: NACA 0012 at 4 degrees, Re 400, Cp on the section, speed on the section plane, streamlines, the Geometry, Flow Conditions and Aero Results panels |
| `aero-apollo-newtonian.png` | `--capture <png> --experiment aero_apollo_capsule --frame 0 --size 1600x900` | Aero workspace with an Apollo-type capsule at Mach 20 and 20 degrees, modified Newtonian theory: Cp on the heat shield, drag arrow, coefficients |
| `aero-naca0012-stall-wake.png` | `--capture <png> --experiment docs/screenshots/naca0012_stall_lbm.json --frame 0 --size 1600x900` | NACA 0012 stalled at 20 degrees, Re 500, lattice Boltzmann over 60 convective times: the separated wake rolls up into a vortex street |
| `entry-apollo.png` | `--headless --out <png> --experiment entry_apollo_like --frame 0 --size 1600x900` | Entry workspace after an Apollo-like ballistic entry (11 km/s, -5.7 degrees): setup panel, trajectory ribbon coloured by g-load, ground track, Entry Profile plots; peak 6.78 g |

Notes:

- The window shots run in wall-clock time: the Space images depend on how
  far the physics got in the given frames (the warp budget lowers the warp
  on a loaded machine), the lattice Boltzmann images only on the solver's
  fixed step count. Pixels may differ slightly between GPUs and drivers.
- The lattice Boltzmann runs use the CPU solver, as in the Aero workspace;
  the stalled wake takes the longest (a few minutes).
- A cylinder vortex street is not among the images: the only 2D shape the
  experiment format offers is the NACA section, and a 3D cylinder at Re 150
  to 200 needs far longer CPU runs than a screenshot justifies before its
  wake settles into regular shedding. The stalled airfoil shows the same
  phenomenon.
- `docs/screenshots/` holds the experiment files used only for the
  screenshots.
