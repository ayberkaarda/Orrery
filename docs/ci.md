# Orrery Continuous Integration

The pull-request workflow (`.github/workflows/ci.yml`), the nightly workflow
(`.github/workflows/nightly.yml`) and the time budget of spec section 8: the
full CI, nightly excluded, should finish in under 20 minutes.

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
  separate install and object-format changes for little gain on a job that is
  not the longest.

### Parallel tests

`ctest --parallel $(nproc)` (4 on the hosted runners). The tests are
independent processes; the ones that write files use names of their own. The
physics and solver thread tests of `orrery_test_sim` observe intermediate
frames of a running thread and could miss them on a fully loaded machine
(reproduced locally with 12 parallel tests), so `tests/CMakeLists.txt` marks
that executable's tests `RUN_SERIAL`: they run alone, about a minute under
ASan.

## 2. Time budget: before and after

Measured from the step timings of three recent runs (`gh run view <id>
--json jobs`):

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

The two long poles were the sanitizer job (no binary cache, serial tests)
and the uncached configure steps.

**After** (estimated from the same step timings; to be confirmed by the first
runs with the new workflow). Test times come from replaying the per-test
durations of run 43472663449 on four parallel slots in ctest's order, with
the `RUN_SERIAL` tests added on top; the range up to 1.6 times that replay
allows for the lattice Boltzmann and Barnes-Hut tests, which already use
several cores and so slow down when they share the runner.

| Job | Setup + configure (cache hit) | Build | Test | Total |
|---|---|---|---|---|
| asan shard 1 | 1.0 | 1.5 (ccache warm) to 4.0 | 5.0 to 8.0 | 7.5 to 13 |
| asan shard 2 | 1.0 | 1.5 to 4.0 | 7.2 to 11.5 | 10 to 16.5 |
| asan (result) | 0.3 after the shards | | | |
| lint | 1.5 | 1.4 | tidy 1.4 to 4.0 | 5 to 7 |
| build gcc 13 | 0.8 | 0.8 to 2.4 | 2.9 to 3.5 | 4.5 to 7 |
| build clang 17 | 1.0 | 0.8 to 4.4 | 1.7 to 2.5 | 3.5 to 8 |
| build MSVC | 1.3 | 4.4 to 5.0 | 2.0 to 3.0 | 8 to 9.5 |

Expected wall time with warm caches: **about 10 to 17 minutes**, set by the
second sanitizer shard; under the 20-minute target.

Honest limits:

- **Cold vcpkg cache.** A run whose `vcpkg.json` changed, or the first run on
  a new runner image, rebuilds the affected dependencies: 9 to 14 minutes of
  configure on Linux (as in run 43472663449) and more on Windows. Such a run
  can exceed 20 minutes; the next one is back to the warm figures. The first
  run of this workflow is such a run too: the pinned vcpkg checkout and the
  new cache keys start new entries (the `restore-keys` fallback reuses what
  still matches).
- **Cold ccache.** Pull requests reuse the cache of the last push to `main`;
  the very first run, and a change to a header included everywhere, compile
  from scratch (the upper ends of the build column).
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
| `large LBM validation (CPU, ubuntu-24.04-gcc-13)` | Builds `orrery_nightly_lbm_validation` (`tests/bench/nightly`) in the release preset and runs it: cylinder at Re 150 (Strouhal 0.16 to 0.20, spec section 8) and at Re 100 with D = 40 (twice the pull-request resolution), sphere at Re 100 with 16 cells per diameter (Cd 1.0 to 1.2, section 8), at Re 10 and Re 300 (within 15 percent of Schiller-Naumann) | `lbm-validation` (console log with every measured value, JUnit XML) |

Why these run nightly and not per pull request:

- The large lattice Boltzmann cases take about 10 to 60 minutes each on the
  CPU of a hosted runner (the pull-request suite has the small CPU cases:
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
