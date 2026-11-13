# Screenshots

The images in [docs/images](images) are shown in the README. They are drawn
by the application's own headless path (`orrery_app --headless`), which
renders into an offscreen OpenGL 4.6 context and never opens a window. They
are illustrations, not test references: the golden images that the test
suite compares by SSIM live under `tests/` and are regenerated only through
the tests (see [rendering.md](rendering.md#8-snapshot-comparison-and-golden-images)).

## Regenerating

Build any configuration that contains `orrery_app` (the `ci-headless`
preset is enough; an OpenGL 4.6 driver is required, exit code 3 means no
context), then run from the repository root:

```sh
# Linux
python3 tools/make_screenshots.py --app build/linux-ci-headless/src/app/orrery_app
# Windows
py tools\make_screenshots.py --app build\ci-headless\src\app\orrery_app.exe
# only some images
py tools\make_screenshots.py --app <app> --only space-black-hole.png entry-apollo.png
```

`tools/make_screenshots.py` runs one headless command per image (table
below), re-encodes each PNG losslessly with Pillow at the highest zlib effort
(optional; without Pillow the rendered file is kept) and drops every text,
time and profile chunk, so no tool name, user name or path is embedded. The
images stay full colour on purpose: a 256-colour palette bands the colour
maps and shifts the colours their legends explain. Each image is 0.1 to
0.55 MB.

| Image | Command (after `orrery_app`) | What it shows |
|---|---|---|
| `space-solar-system.png` | `--headless --experiment solar_system --frame 3000 --size 1600x900` | J2000 solar system after 579 days (leapfrog, 0.05 d steps, 1e6x warp): inner planets with their trails, starfield and Milky Way |
| `space-black-hole.png` | `--headless --experiment black_hole --frame 0 --size 1600x900` | Schwarzschild lensing of a thin accretion disk: the shadow, the lensed far side of the disk above the hole, Doppler-brightened approaching side |
| `space-figure-eight.png` | `--headless --experiment figure_eight --frame 1200 --size 1600x900` | The Chenciner-Montgomery figure-eight three-body choreography after 462 days (Yoshida 4th order) |
| `aero-naca0012-lbm.png` | `--headless --experiment aero_naca0012_lbm --frame 0 --size 1600x900` | NACA 0012 at 4 degrees, Re 400, D2Q9 lattice Boltzmann run to completion (32000 steps): Cp on the section, speed on the section plane, streamlines |
| `aero-apollo-newtonian.png` | `--headless --experiment aero_apollo_capsule --frame 0 --size 1600x900` | Apollo-type capsule at Mach 20 and 20 degrees, modified Newtonian theory: Cp on the heat shield, drag arrow, Cd 1.37 |
| `aero-naca0012-stall-wake.png` | `--headless --experiment docs/screenshots/naca0012_stall_lbm.json --frame 0 --size 1600x900` | NACA 0012 stalled at 20 degrees, Re 500, D2Q9 lattice Boltzmann over 60 convective times: the separated wake rolls up into a vortex street |
| `entry-apollo.png` | `--headless --experiment entry_apollo_like --frame 0 --size 1600x900` | Entry workspace after an Apollo-like ballistic entry (11 km/s, -5.7 degrees): setup panel, trajectory ribbon coloured by g-load, ground track, Entry Profile plots; peak 6.78 g |

Notes:

- The physics behind every image is deterministic (fixed step counts, no
  wall-clock budget in headless runs); the pixels may differ slightly
  between GPUs and drivers.
- The headless path draws the docked panels of the workspace only for entry
  experiments. Space and aero images show the 3D view with its legends; the
  Space and Aero panels are drawn only in the window.
- The lattice Boltzmann images run on the CPU, as in the Aero workspace; the
  whole set takes about five minutes on a 6-core desktop, most of it the
  stalled-wake run.
- A cylinder vortex street is not among the images: the only 2D shape the
  experiment format offers is the NACA section, and a 3D cylinder at Re 150
  to 200 needs far longer CPU runs than a screenshot justifies before its
  wake settles into regular shedding. The stalled airfoil shows the same
  phenomenon.
- An Earth-Moon view is not among them either: the headless framing fits
  the Moon's orbit, which leaves the Earth a few pixels wide; the
  figure-eight choreography stands for the few-body scenarios instead.
- `docs/screenshots/` holds the experiment files used only for the
  screenshots.
