# Orrery Continuous Integration

The pull-request workflow (`.github/workflows/ci.yml`), the nightly workflow
(`.github/workflows/nightly.yml`) and the V1 time budget: the full CI,
nightly excluded, should finish in under 20 minutes.

## 1. Pull-request workflow

Every push to `main`, every pull request and manual runs. The required checks
keep their names:

| Check | Runner | What it does |
|---|---|---|
| `build (ubuntu-24.04-gcc-13)` | ubuntu-24.04 | `linux-ci-headless` (Release, warnings as errors, headless): configure, `check_deps.py`, build, full ctest |
| `build (ubuntu-22.04-clang-17)` | ubuntu-22.04 | the same with clang 17 |
| `build (windows-2022-msvc)` | windows-2022 | `ci-headless` with MSVC |
| `asan (ubuntu-24.04-gcc-13)` | ubuntu-24.04 | passes when both sanitizer shards pass (below) |
| `lint (format, tidy, cppcheck, deps)` | ubuntu-22.04 | `check_deps.py` (sources and configured tree), clang-format 22, clang-tidy 17 on the changed translation units, cppcheck |

The sanitizer suite runs as two extra jobs, `asan (ubuntu-24.04-gcc-13)
shard 1/2` and `shard 2/2`: each configures and builds `linux-debug-asan`
(ASan and UBSan, `-fno-sanitize-recover=all`, leak detection on) and runs
every second test number (`ctest -I <shard>,,2`), so together they run the
whole suite once. The `asan (ubuntu-24.04-gcc-13)` job only checks their
result, which keeps the required check name of the single job it replaces.

The warning flags, warnings-as-errors, sanitizer options and the test set
are unchanged. Every test of the suite still runs on every pull request;
nothing moved to the nightly workflow.

### Caches

- **vcpkg binary cache** (`.vcpkg-cache`, `actions/cache`), in every job.
  The key is the compiler line (`ubuntu-24.04-gcc-13`, `ubuntu-22.04-clang-17`,
  `windows-2022-msvc`), the hash of `vcpkg.json` and the runner image version
  (`ImageOS-ImageVersion`), because vcpkg hashes the compiler build into each
  package. A miss restores the newest entry of the same compiler line
  (`restore-keys`), so a new image or a manifest change rebuilds only the
  packages whose hash changed. `asan` shares the entry of the GCC 13 build
  job and `lint` the one of the clang 17 build job: vcpkg builds the
  dependencies with the same compiler and triplet whatever the project's build
  type or sanitizer flags. Before this, `asan` and `lint` had no cache and
  built every dependency from source in each run.
- **vcpkg checkout pinned** to the manifest's `builtin-baseline`. The scripts
  of the checkout are part of every package hash; cloning the moving `master`
  made the hashes change between runs whenever upstream touched them.
- **ccache** for the compiling Linux jobs (the two Linux builds and the
  sanitizer shards), through `CMAKE_CXX_COMPILER_LAUNCHER=ccache` (first-party
  code only; vcpkg ports are hashed and cached on their own). The cache
  (`.ccache`, at most 500 MB) is restored from the newest push to `main` and
  saved only by pushes to `main`, so pull requests read it without filling the
  repository's cache quota. Windows has no ccache: MSVC caching needs a
  separate install and object-format changes, and the Windows build step
  takes about 6 minutes; what makes that job the longest is its vcpkg
  configure when the binary cache is cold (section 2).

### Parallel tests

`ctest --parallel $(nproc)` (4 on the hosted runners). The tests are
independent processes; the ones that write files use names of their own. The
physics and solver thread tests of `orrery_test_sim` observe intermediate
frames of a running thread and could miss them on a fully loaded machine
(reproduced locally with 12 parallel tests), so `tests/CMakeLists.txt` marks
that executable's tests `RUN_SERIAL`: they run alone, about a minute under
ASan. Checked locally with 12 parallel tests: every test passes except, on a
machine with a GPU, the two `[gl]` frame-time tests (space view and black
hole at 1080p), whose GPU time is shared with the other GPU tests running at
the same moment; the hosted runners have no OpenGL context and skip them. On
a GPU machine run those two serially (`ctest -R "1080p"`).

## 2. Time budget: measured

### Before the restructuring

Measured from the step timings of three runs of the previous workflow
(`gh run view <id> --json jobs`):

| Run | Caches | Wall time (first job start to last job end) |
|---|---|---|
| 43413452374 (pull request, phase 9) | build jobs hit | 45 min |
| 43419598006 (push to main, phase 9) | build jobs hit | 38 min |
| 43472663449 (pull request, phase 10, `vcpkg.json` changed) | all missed | 49 min |

Where the time went, per job (minutes, from the same runs):

| Job | Setup | Configure | Build | Test | Total |
|---|---|---|---|---|---|
| asan | 0.7 | 11.1 to 13.7 (no cache: every dependency built) | 3.5 to 4.8 | 22.7 to 30.0 (serial, 1800 s of tests) | 38 to 49 |
| lint | 1.0 | 7.2 to 14.0 (no cache) | 0.6 to 1.4 (generated headers) | tidy 1.4 to 4.0, cppcheck 0.1 | 10 to 20 |
| build gcc 13 | 0.6 | 0.1 (hit), 9.3 (miss) | 2.2 to 3.6 | 3.3 to 3.9 (serial) | 6 to 16 |
| build clang 17 | 0.9 | 0.1 (hit), 13.6 (miss) | 1.8 to 4.4 | 2.0 to 3.3 (serial) | 5 to 22 |
| build MSVC | 0.8 to 1.3 | 0.3 to 0.4 | 4.4 to 5.0 | 3.7 to 6.8 (serial) | 10 to 13 |

### After: the first two runs of this workflow

Measured the same way on the pull request that introduced it (run
43507843543, PR #18) and on the push to `main` that merged it (run
43513388412). Cache state, read from the job logs:

- **vcpkg, Linux: warm.** Every Linux job restored the 47 packages of the
  manifest from the binary cache (an older entry of the same compiler line,
  through `restore-keys`).
- **vcpkg, Windows: cold in both runs.** The only Windows entry predated the
  pinned vcpkg checkout, so its packages no longer matched ("Restored 0
  package(s)") and configure built every dependency. The pull request's own
  entry was not visible to the push to `main` (GitHub scopes a pull request's
  caches to its merge ref), so the push rebuilt them again and saved the
  first `main` entry with the new key (image `win22-20261112.366.7`).
- **ccache: cold in both runs.** No entry existed yet (hit rates 0 to 1.3
  percent); the push to `main` saved the first one.

Per job, minutes (pull request / push):

| Job | Configure | Build | Test | Total |
|---|---|---|---|---|
| asan shard 1/2 | 0.1 / 0.1 | 5.4 / 4.3 | 13.6 / 10.1 | **19.8 / 15.5** |
| asan shard 2/2 | 0.1 / 0.1 | 5.1 / 3.8 | 9.7 / 5.9 | 15.7 / 10.4 |
| asan (result) | | | | 0.1 / 0.1 |
| lint | 0.1 (+ 0.7 to 0.8 generated headers) | | tidy 0.8 to 1.0, cppcheck 0.1 | 2.9 / 2.5 |
| build gcc 13 | 0.1 / 0.1 | 4.3 / 4.1 | 4.0 / 5.4 | 9.4 / 10.4 |
| build clang 17 | 0.1 / 0.1 | 2.5 / 3.2 | 2.0 / 2.5 | 5.5 / 6.8 |
| build MSVC | **24.6 / 24.1** (cold vcpkg) | 6.3 / 6.3 | 3.9 / 3.8 | **36.2 / 35.5** |

Wall time: **36.2 min** (pull request) and **35.5 min** (push), both set by
the Windows job and its cold vcpkg cache. Without the Windows job the
longest path was 19.8 and 15.7 minutes (the first sanitizer shard). The
sanitizer job went from 38 to 49 minutes to 16 to 20 minutes, lint from 10
to 20 minutes to under 3.

**The 20-minute target is not met yet.** It can only be met with warm caches,
and no run with a warm Windows cache has been measured. With the configure
time of a warm Windows cache (0.3 to 0.4 min in the runs before the
restructuring) instead of 24 minutes, the Windows job would take about 12
minutes and the wall time would be set by the first sanitizer shard, 15.5 to
19.8 minutes in these runs with a cold ccache. The next pull request will be
the first measurement with every cache warm; this section should be updated
with it.

Limits that remain:

- **Cold vcpkg cache.** A run whose `vcpkg.json` changed, the first run on a
  new runner image, and the first push to `main` after such a change rebuild
  the affected dependencies: 9 to 14 minutes of configure on Linux (run
  43472663449) and about 24 minutes on Windows (runs above). Such a run
  exceeds 20 minutes.
- **Cold ccache.** Pull requests reuse the cache of the last push to `main`;
  the first run, and a change to a header included everywhere, compile from
  scratch (the build column above: 2.5 to 5.4 minutes).
- **Runner cost.** Sharding adds a second sanitizer build and a short result
  job: about 5 more runner minutes per run, traded for wall time.

## 3. Nightly workflow

`nightly.yml` runs at 02:17 UTC every day and on demand
(`workflow_dispatch`); it never runs on pull requests and is not a required
check. Both jobs restore the vcpkg binary cache of the pull-request workflow
(GCC 13 entry) without saving to it.

| Job | Content | Artifact |
|---|---|---|
| `release LTO (ubuntu-24.04-gcc-13), full tests and benchmarks` | Configures the `linux-release` preset (Release with LTO, `ORRERY_LTO`, [performance.md](performance.md)), runs `check_deps.py`, builds, runs the whole ctest suite in parallel - which includes the golden 10k-step determinism hashes, so a change of the hashes under link-time optimisation fails the night - then `tests/bench/run_benchmarks.py` and `make_report.py` (report also in the job summary) | `bench-results` (JSON, Markdown report, run log), 30 days |
| `large LBM validation (CPU, ubuntu-24.04-gcc-13)` | Builds `orrery_nightly_lbm_validation` (`tests/bench/nightly`) in the release preset and runs it: cylinder at Re 150 (Strouhal 0.16 to 0.20, a V1 target) and at Re 100 with D = 40 (twice the pull-request resolution), sphere at Re 100 with 16 cells per diameter (Cd 1.0 to 1.2, a V1 target), at Re 10 and Re 300 (within 15 percent of Schiller-Naumann) | `lbm-validation` (console log with every measured value, JUnit XML) |

Why these run nightly and not per pull request:

- The large lattice Boltzmann cases took 4 to 8 minutes each, 28 minutes for
  the five, on a 12-thread desktop CPU; a four-core hosted runner needs
  roughly three times that (the pull-request suite has the small CPU cases:
  cylinder Re 100 at D = 20, sphere Re 100 at 12 cells per diameter). The GPU
  validation at Re 10, 100, 300 and 1000 needs an OpenGL 4.6 context, which
  hosted runners lack; the nightly job runs those grids on the CPU solvers
  instead (Re 1000 needs 48 cells per diameter and is left to GPU machines).
- The benchmarks measure; on shared hosted runners the numbers vary from
  night to night, so they are reported, never gating. GPU rows are reported
  as not measured there.
- The release preset adds link-time optimisation, which only changes code
  generation; the warnings-as-errors build of the same sources is checked on
  every pull request. Running it nightly keeps the release profile and its
  determinism checked without lengthening pull requests.

## 4. Reproducing locally

```
cmake --preset ci-headless && cmake --build --preset ci-headless
ctest --preset ci-headless --parallel 8
python tools/check_deps.py --build-dir build/ci-headless
```

The workflow files are checked with `actionlint` (with shellcheck): no error;
the remaining shellcheck notes are the deliberate word splitting of `$APT`
and `$LINUX_DEPS` and one single-quoted `$1`, all present before.
