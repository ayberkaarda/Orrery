# Orrery Performance

The numeric acceptance criteria of spec section 8 ("SAYISAL KABUL
KRITERLERI"), measured; the benchmark suite that measures them; and the
release profile with link-time optimisation (spec phase 11: "bench targets,
nightly large LBM test, release profile (LTO)"). Deviations are reported with
the measurement, the reason and a recommendation, never hidden. The CI time
row is in [ci.md](ci.md).

## 1. Benchmark suite

`tests/bench` holds one benchmark executable per module, built with the tests
(option `ORRERY_BUILD_BENCHMARKS`, on by default) so every CI build compiles
them with the project's warnings, and never registered with ctest:

| Executable | Measures |
|---|---|
| `orrery_bench_core` | Barnes-Hut force evaluation at N = 10k and 100k (targets), N = 100 .. 100k with one thread, direct summation up to 10k, leaf sizes; the golden 10k-step hashes of the three fixed-step integrators |
| `orrery_bench_geom` | 500k-triangle glb import with the watertightness report, and with the full repair pass |
| `orrery_bench_aero` | 3D panel method at 2048 panels (setup and solve), Newtonian on 100k triangles, CPU lattice Boltzmann rates on the GPU target grids |
| `orrery_bench_lbm_gpu` | GPU D3Q19 at 128^3 and D2Q9 at 2048 x 1024, BGK and Smagorinsky (hidden-window OpenGL 4.6 context, never shown; recorded as not measured without one) |

Each benchmark is a Catch2 test case (names and tags select them as usual)
that records its measurement, unit and target; `--json <file>` writes the
records. A missed target is recorded, not a failure; only a broken result (a
non-finite coefficient, a mesh that is not watertight, a hash that differs
between two runs) fails the executable.

`tests/bench/run_benchmarks.py` runs them all and measures the application
through its headless mode, which never opens a window:

- **Cold start to first frame**: wall time of `orrery_app --headless
  --experiment solar_system --size 1920x1080 --out <png>`, from process start
  to the PNG written and the process gone. It includes the OpenGL context,
  shader compilation, scenario loading, the first frame, its readback and the
  PNG encoding, so it is an upper bound of the time to the first frame. The
  first run after the build is reported, the median of four more runs as
  context. A truly cold start (empty file cache) needs a reboot and is not
  measured.
- **Render**: `--benchmark` of the headless view (`app::benchmark_headless`):
  wall time per frame from the first command to the frame's GPU timer query,
  at 1920x1080, for `solar_system` (600 frames) and `black_hole` (300 frames;
  the lensing pass traces at half resolution, ADR-017).
- **Idle RAM**: peak working set (Windows) or maximum resident set (Linux) of
  the solar-system benchmark process: load, first frame, 600 frames at
  1920x1080. The window adds the Dear ImGui panels and the swap chain, which
  this does not include (the window cannot be measured without showing it).
- **Binary + assets**: `orrery_app`, every shared library next to it (the
  vcpkg app-local copies) and `assets/`, uncompressed, MB = 10^6 bytes. The
  installer of the packaging work compresses this further.

It writes one JSON file with a machine description free of host names;
`tests/bench/make_report.py` turns it into the tables below.

```
cmake --preset release && cmake --build --preset release
python tests/bench/run_benchmarks.py --build-dir build/release --out results.json
python tests/bench/make_report.py results.json
```

The nightly workflow runs the same on a hosted Linux runner (CPU rows only;
[ci.md](ci.md)).

## 2. Release profile (LTO)

`cmake/Lto.cmake` adds `ORRERY_LTO`: CMake's interprocedural optimisation for
every first-party target (MSVC `/GL` and `/LTCG`, GCC `-flto=auto`, Clang
`-flto=thin`); a toolchain without support configures without it and warns.
The `release` and `linux-release` presets turn it on (Release: `/O2` or
`-O3`, `NDEBUG`, no sanitizers); `ci-headless`, the sanitizer and profiling
presets do not.

Floating-point determinism is unchanged: the compile options of
`FpDeterminism.cmake` (`/fp:precise`, `-fno-fast-math -ffp-contract=off`)
travel with each function into the link-time optimiser, and with LTO the
GCC/Clang link step repeats them. `tools/check_deps.py` checks the release
tree like every other (it no longer reads the MSVC `/GL` switch on the link
line as a library named GL).

**Determinism under LTO, measured:**

| Toolchain | Check | Result |
|---|---|---|
| MSVC 19.44 x64, `release` preset (`/O2 /GL /LTCG /fp:precise`) | full ctest with `ORRERY_REQUIRE_GL=1`: 791 of 791 passed, including the golden initial-state and 10k-step hashes (leapfrog KDK, Yoshida4, RK4), IAS15 and Barnes-Hut golden hashes | identical to the non-LTO build |
| MSVC, `orrery_bench_core` | two runs of each 10k-step golden scenario against the pinned hash | bit-identical |
| GCC 15.2 x86-64 Linux (WSL), core sources with `-O3 -fno-fast-math -ffp-contract=off`, with and without `-flto=auto -fno-fat-lto-objects` | standalone driver over the golden scenarios (Barnes-Hut and the octree left out: no oneTBB there) | both: 0x8ad1887966f2ea12, 0x08ca0a8c4993debe, 0xd0c645137b27a3cd, 0xab3b17d03159374d = the pinned hashes |

The nightly workflow repeats the full suite under GCC 13 with LTO.

**Speed under LTO (MSVC, measured):** three alternating rounds of the CPU
benchmarks, `ci-headless` (Release, no LTO) against `release` (LTO), medians:

| Benchmark | No LTO | LTO | LTO / no LTO |
|---|---|---|---|
| Force N = 10k, Barnes-Hut, all threads | 12.9 ms | 12.3 ms | 0.95 |
| Force N = 100k, Barnes-Hut, all threads | 214 ms | 213 ms | 1.00 |
| Force N = 10k, direct, one thread | 231 ms | 232 ms | 1.00 |
| LBM D2Q9 2048 x 1024, CPU | 83.7 steps/s | 82.9 steps/s | 0.99 (rate) |
| LBM D3Q19 128^3, CPU | 33.1 steps/s | 34.8 steps/s | 1.05 (rate) |
| Model import 500k triangles with report | 0.99 s | 0.96 s | 0.97 |
| Panel 3D, N = 2000 | 0.53 s | 0.66 s | 1.25 |
| Newtonian, 100k triangles | 1.93 ms | 2.16 ms | 1.12 |

Link-time optimisation buys nothing measurable here: the hot loops already
sit in single translation units. The panel method (dense Eigen LU, inlined
into `orrery_aero`) is about 25 percent slower with MSVC's whole-program
optimisation, still 4.5 times under its target; the Newtonian difference is
0.2 ms. Recommendation: keep the release profile as the spec asks (it is
also what the installer builds), and if the panel slowdown matters, exclude
`orrery_aero`'s `panel3d.cpp` from `/GL` or try profile-guided optimisation;
GCC's LTO speed has not been measured.

## 3. Results

Release preset (MSVC 19.44, LTO), 2026-11-13, measured with
`run_benchmarks.py` while no other Orrery process ran (the machine was
shared with other builds earlier; runs taken under that load are not used:
the GPU rows dropped two to four times and the force rows varied by 30
percent).

| | |
|---|---|
| OS | Windows 11 |
| CPU | AMD Ryzen 5 7600 class: 6 cores, 12 hardware threads (Zen 4 desktop) |
| Memory | 32 GB |
| GPU | AMD Radeon RX 9060 XT class (RDNA 4 mid-range desktop) |

All rows of the section 8 table. Performance rows come from the benchmark
suite; accuracy rows from the tests that check them, re-run in the release
build to print their values (`ctest` hides the output of passing tests).

| Section 8 metric | Target | Measured | Result | Source |
|---|---|---|---|---|
| Energy drift, solar system 1000 years, Yoshida4 | abs(dE/E0) < 1e-9 | max 1.41e-10 (final 2.6e-14) | pass | `Sun and eight planets, 1000 years with Yoshida4` (orrery_test_core) |
| Two-body circular, 100 orbits, Leapfrog | < 1e-6 AU | 3.77e-7 AU | pass | `circular two-body orbit, 100 orbits with Leapfrog` |
| Barnes-Hut theta 0.5, N = 2000, acceleration error | median < 1e-3 | median 4.70e-4 (p99 1.97e-3) | pass | `Barnes-Hut accuracy against direct summation, N = 2000` |
| Force N = 10k (8 cores) | < 8 ms | **16.8 ms** (12 threads; 12.3 to 16.8 over five runs) | **miss** | `orrery_bench_core` |
| Force N = 100k (8 cores) | < 120 ms | **245 ms** (213 to 245) | **miss** | `orrery_bench_core` |
| LBM D3Q19 128^3, GTX 1660 class | >= 60 steps/s | 671 steps/s (RX 9060 XT class) | pass on this GPU; target class not measured | `orrery_bench_lbm_gpu` |
| LBM D2Q9 2048 x 1024 | >= 200 steps/s | 1511 steps/s (GPU) | pass | `orrery_bench_lbm_gpu` |
| Panel 3D N = 2000 solve | < 3 s | 0.68 s (2048 panels, setup and solve) | pass | `orrery_bench_aero` |
| Newtonian 100k triangles | < 100 ms | 2.08 ms (100352 triangles) | pass | `orrery_bench_aero` |
| NACA 0012 Cl slope | 2 pi / rad +-10% | 6.90 / rad = 1.098 x 2 pi | pass (0.2 percent inside the bound) | `NACA 0012 lift slope is 2 pi per radian within 10 percent` |
| Cylinder Re = 150 Strouhal | 0.16 to 0.20 | St 0.189 (nightly CPU case, D = 40); pull-request case Re 100: St 0.167 | pass (nightly) | `orrery_nightly_lbm_validation`, `test_cylinder.cpp` |
| Sphere Cd Re = 100 | 1.0 to 1.2 | 1.186 (CPU, 12 cells/D), 1.140 (GPU, 16 cells/D) | pass | `test_sphere.cpp` |
| Render Space 1080p | >= 60 fps | 397 fps (mean 2.52 ms, GPU median 2.33 ms) | pass | headless `--benchmark 600` |
| Render black hole, half-resolution trace | >= 30 fps | 815 fps at 1920x1080 (mean 1.23 ms) | pass | headless `--benchmark 300` |
| Model import 500k triangles glTF (local), with report | < 5 s | 1.06 s with the report, 1.82 s with the full repair | pass | `orrery_bench_geom` |
| Determinism (core), 10k steps | bit-identical | identical, equal to the golden hashes (LTO build) | pass | `orrery_bench_core`, `test_determinism.cpp` |
| Binary + asset | < 80 MB | 17.4 MB (executable 3.5, libraries 13.9, assets 0.02) | pass | `run_benchmarks.py` |
| Idle RAM (solar system) | < 400 MB | 172 MB peak working set (headless; peak commit 429 MB) | pass, with the caveat of section 4 | `run_benchmarks.py` |
| Cold start to first frame | < 3 s | 0.59 s (upper bound, headless, warm file cache) | pass | `run_benchmarks.py` |
| CI time (full, nightly excluded) | < 20 min | 38 to 49 min before; 10 to 17 min estimated after | see [ci.md](ci.md) | step logs |

Context measurements of the same run:

| Measurement | Value |
|---|---|
| Barnes-Hut, one thread, N = 100 / 1k / 10k / 100k | 0.064 / 3.49 / 82.3 / 1370 ms |
| Direct summation, one thread, N = 100 / 1k / 10k | 0.034 / 3.49 / 241 ms |
| Barnes-Hut N = 10k, all threads, leaf 16 / 32 | 14.0 / 15.4 ms |
| LBM on the CPU (double precision, 12 threads), D2Q9 2048 x 1024 / D3Q19 128^3 | 79.0 / 32.0 steps/s (166 / 67 million lattice updates per second) |
| LBM on the GPU with the Smagorinsky closure, D2Q9 / D3Q19 | 1527 / 680 steps/s |
| Cold start, median of four later runs | 0.59 s |

## 4. Deviations

**Force N = 10k and 100k: missed by about a factor of two.** Measured
12.3 to 16.8 ms and 213 to 245 ms on 6 cores / 12 threads (the spec's 8
cores are not available here); one thread takes 74 to 82 ms and 1.31 to
1.37 s, so the parallel speed-up is about 5.5. Reason (unchanged since phase
3a, physics.md "Barnes-Hut"): the walk is scalar double arithmetic with a
correctly rounded square root and division per interaction and a quadrupole
term, under the determinism contract (`/fp:precise`, no contraction, no
approximate reciprocal square root), and MSVC does not vectorise the receiver
loop under those rules; LTO does not change it (section 2). Linear scaling
to 8 cores gives about 9 to 12 ms and 160 to 185 ms: still a miss.
Recommendation, in order of payoff: explicit SIMD over receivers (each lane
performs the same IEEE operations in the same order, so the results stay
bit-identical); reuse of the tree between the substeps of one frame (the
build is part of every evaluation); a monopole-only mode with a smaller
opening angle for interactive use. Re-measure on an 8-core machine.

**LBM D3Q19 target class not measured.** The target names a GTX 1660-class
GPU; this machine has a newer, faster one (671 steps/s, 11 times the
target). The kernel is bandwidth-bound: 671 steps/s moves about 214 GB/s
(19 binary32 populations read and written per node), two thirds of this
GPU's 322 GB/s peak. A GTX 1660 has 192 GB/s peak, so at the same efficiency
about 400 steps/s, well above 60.
This is an estimate, not a measurement; run `orrery_bench_lbm_gpu` on such a
GPU to close it.

**Idle RAM measured on the headless process.** The window always opens
visibly (`window_app.cpp` has no hidden mode), and the measurements must not
show windows, so the row is the peak working set of the headless solar-system
process at 1920x1080 (renderer, simulation, 600 frames): 172 MB. The
windowed application adds Dear ImGui, the panels and the swap chain, which
are small next to the margin to 400 MB. The peak private commit is 429 MB,
mostly driver allocations that never become resident; by that measure the
row would be over the target. Recommendation: add a hidden-window or
`--quit-after` mode to the window application so the full shell can be
measured idle without a visible window, and track both working set and
commit.

**Cold start is an upper bound with a warm file cache.** Process start to
the PNG written and the process gone (0.59 s), headless. A cold file cache
(after a reboot) and the window path (window creation, font atlas) are not
measured; the margin to 3 s is five-fold.

**Cylinder Re = 150 only in the nightly run.** The pull-request suite
checks Re 100 at D = 20 (St 0.167 on CPU and GPU); Re 150 at D = 40 runs in
the nightly CPU validation, measured here: St 0.189, mean Cd 1.42 (11
shedding cycles), inside 0.16 to 0.20. The same run gave, at Re 100 and
D = 40, St 0.171 and Cd 1.43; for the sphere at 16 cells per diameter Cd
1.140 at Re 100 (1.0 to 1.2) and 0.693 at Re 300 (Schiller-Naumann 0.684),
and at Re 10 Cd 4.41 (ratio 1.06). All five nightly cases passed (28 minutes
on 12 threads).

**NACA 0012 lift slope close to the bound.** 6.90 per radian is 1.098 x 2 pi,
0.2 percent inside the +10 percent bound; the thick-section (Joukowski)
slope it should approach is 6.86, matched within 0.5 percent, so the margin
reflects the thickness effect, not an error.

**CI time.** Before: 38 to 49 minutes. After the restructuring of
[ci.md](ci.md): an estimate of 10 to 17 minutes with warm caches, from the
step logs; a run with a cold vcpkg cache (manifest or runner image change)
can still exceed 20 minutes. The first runs on `main` will confirm it.

## 5. Raw data

[`performance/results-windows-msvc-release.json`](performance/results-windows-msvc-release.json)
is the `run_benchmarks.py` output of the run in section 3 (29 records,
machine description without host names). `make_report.py` on it prints the
benchmark rows of the table above. The nightly workflow uploads the same file
for the Linux runner as the `bench-results` artifact.
