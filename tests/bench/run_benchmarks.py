#!/usr/bin/env python3
"""Run the Orrery benchmarks and merge their results into one JSON file.

Runs every benchmark executable of a build tree (tests/bench, one per module,
each writing its own JSON), then measures the application itself through its
headless mode, which never opens a window:

  - cold start to the first frame: wall time of a headless render of the
    solar system at 1920x1080 (process start to PNG written and exit, an
    upper bound), first run and median of the following runs;
  - frame rate: --benchmark of the space view (solar system) and of the
    black-hole view (half-resolution trace) at 1920x1080;
  - memory: peak working set (Windows) or maximum resident set (Linux) of the
    solar-system benchmark process;
  - size: the executable, the shared libraries next to it and assets/.

Section 8 of the specification sets the targets; docs/performance.md
describes the method. make_report.py turns the result into a table.

usage: run_benchmarks.py --build-dir build/release --out results.json
                         [--suites core,geom,aero,lbm_gpu] [--skip-app]
                         [--log-dir DIR]
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import platform
import re
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SUITES = ("core", "geom", "aero", "lbm_gpu")
EXE_SUFFIX = ".exe" if os.name == "nt" else ""
EXIT_NO_GL = 3
BENCHMARK_LINE = re.compile(
    r"benchmark: (\d+) frames at (\d+)x(\d+) \(.*\): mean ([0-9.eE+-]+) ms, ([0-9.eE+-]+) fps; "
    r"GPU median ([0-9.eE+-]+) ms, p95 ([0-9.eE+-]+) ms on (.*)$",
    re.MULTILINE,
)


# --------------------------------------------------------------------------
# Machine description (no host names or user data)
# --------------------------------------------------------------------------


def cpu_name() -> str:
    if os.name == "nt":
        try:
            import winreg

            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                                 r"HARDWARE\DESCRIPTION\System\CentralProcessor\0")
            return str(winreg.QueryValueEx(key, "ProcessorNameString")[0]).strip()
        except OSError:
            return platform.processor()
    try:
        for line in Path("/proc/cpuinfo").read_text(encoding="utf-8").splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor()


def memory_gib() -> float | None:
    if os.name == "nt":
        import ctypes

        class MemoryStatus(ctypes.Structure):
            _fields_ = [("length", ctypes.c_ulong), ("load", ctypes.c_ulong),
                        ("total_phys", ctypes.c_ulonglong), ("avail_phys", ctypes.c_ulonglong),
                        ("total_page", ctypes.c_ulonglong), ("avail_page", ctypes.c_ulonglong),
                        ("total_virtual", ctypes.c_ulonglong),
                        ("avail_virtual", ctypes.c_ulonglong),
                        ("avail_extended", ctypes.c_ulonglong)]

        status = MemoryStatus()
        status.length = ctypes.sizeof(MemoryStatus)
        if ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(status)):
            return round(status.total_phys / 2**30, 1)
        return None
    try:
        return round(os.sysconf("SC_PAGE_SIZE") * os.sysconf("SC_PHYS_PAGES") / 2**30, 1)
    except (ValueError, OSError, AttributeError):
        return None


def machine() -> dict:
    return {
        "os": f"{platform.system()} {platform.release()}",
        "cpu": cpu_name(),
        "logical_cpus": os.cpu_count(),
        "memory_gib": memory_gib(),
        "python": platform.python_version(),
    }


def commit() -> str | None:
    try:
        out = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, check=True)
        return out.stdout.strip() or None
    except (OSError, subprocess.CalledProcessError):
        return None


# --------------------------------------------------------------------------
# Child processes: wall time and peak memory
# --------------------------------------------------------------------------


def peak_memory_windows(handle: int) -> tuple[int, int] | None:
    """Peak working set and peak private commit (bytes) of a process handle."""
    import ctypes
    from ctypes import wintypes

    class Counters(ctypes.Structure):
        _fields_ = [("cb", wintypes.DWORD), ("page_faults", wintypes.DWORD),
                    ("peak_working_set", ctypes.c_size_t), ("working_set", ctypes.c_size_t),
                    ("peak_paged_pool", ctypes.c_size_t), ("paged_pool", ctypes.c_size_t),
                    ("peak_nonpaged_pool", ctypes.c_size_t), ("nonpaged_pool", ctypes.c_size_t),
                    ("pagefile", ctypes.c_size_t), ("peak_pagefile", ctypes.c_size_t)]

    counters = Counters()
    counters.cb = ctypes.sizeof(Counters)
    psapi = ctypes.WinDLL("psapi")
    psapi.GetProcessMemoryInfo.argtypes = [wintypes.HANDLE, ctypes.POINTER(Counters),
                                           wintypes.DWORD]
    if not psapi.GetProcessMemoryInfo(handle, ctypes.byref(counters), counters.cb):
        return None
    return counters.peak_working_set, counters.peak_pagefile


def run_measured(args: list[str], log) -> dict:
    """Runs a process; returns exit code, output, wall seconds and peak memory."""
    start = time.perf_counter()
    proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            cwd=tempfile.gettempdir())
    peak = None
    if os.name == "nt":
        output, _ = proc.communicate()
        seconds = time.perf_counter() - start
        peak = peak_memory_windows(int(proc._handle))  # noqa: SLF001 (process handle)
    else:
        assert proc.stdout is not None
        output = proc.stdout.read()
        _, status, usage = os.wait4(proc.pid, 0)
        seconds = time.perf_counter() - start
        proc.returncode = os.waitstatus_to_exitcode(status)
        peak = (usage.ru_maxrss * 1024, None)
    log.write(f"$ {' '.join(args)}\n{output}\n[exit {proc.returncode}, {seconds:.3f} s]\n")
    log.flush()
    return {"code": proc.returncode, "output": output, "seconds": seconds, "peak": peak}


# --------------------------------------------------------------------------
# Benchmark executables
# --------------------------------------------------------------------------


def run_suite(build: Path, suite: str, work: Path, log) -> tuple[dict, list[dict]]:
    exe = build / "tests" / "bench" / f"orrery_bench_{suite}{EXE_SUFFIX}"
    if not exe.is_file():
        return {"error": f"{exe.name} not built"}, []
    out = work / f"{suite}.json"
    result = run_measured([str(exe), "--json", str(out)], log)
    if not out.is_file():
        return {"exit_code": result["code"], "error": "no JSON written"}, []
    data = json.loads(out.read_text(encoding="utf-8"))
    for entry in data["results"]:
        entry["suite"] = suite
    info = {"exit_code": result["code"], "seconds": round(result["seconds"], 1),
            "build": data.get("build", {}), "devices": data.get("devices", [])}
    return info, data["results"]


# --------------------------------------------------------------------------
# Application measurements
# --------------------------------------------------------------------------


def entry(id_: str, title: str, value: float, unit: str, goal: str, target: float | None,
          notes: str, context: dict | None = None) -> dict:
    status = "info"
    if goal == "below":
        status = "pass" if value < target else "fail"
    elif goal == "at_least":
        status = "pass" if value >= target else "fail"
    out = {"id": id_, "title": title, "value": value, "unit": unit, "goal": goal,
           "status": status, "notes": notes, "context": context or {}, "suite": "app"}
    if target is not None and goal != "info":
        out["target"] = target
    return out


def skipped(id_: str, title: str, reason: str) -> dict:
    return {"id": id_, "title": title, "status": "skipped", "notes": reason, "suite": "app"}


def headless(app: Path, experiment: str, png: Path, extra: list[str]) -> list[str]:
    return [str(app), "--headless", "--experiment", experiment, "--size", "1920x1080",
            "--out", str(png)] + extra


def cold_start(app: Path, work: Path, log, runs: int) -> list[dict]:
    title = "Cold start to first frame"
    times = []
    for index in range(runs):
        result = run_measured(headless(app, "solar_system", work / f"start_{index}.png", []), log)
        if result["code"] == EXIT_NO_GL:
            return [skipped("app.cold_start", title, "no OpenGL 4.6 context")]
        if result["code"] != 0:
            return [skipped("app.cold_start", title, f"orrery_app exited {result['code']}")]
        times.append(result["seconds"])
    warm = statistics.median(times[1:]) if len(times) > 1 else times[0]
    return [entry("app.cold_start", title, times[0], "s", "below", 3.0,
                  "headless solar system at 1920x1080: process start to PNG written and exit "
                  "(upper bound of the first frame); first run after the build",
                  {"runs": len(times), "median_later_runs_s": round(warm, 3)})]


def frame_rate(app: Path, work: Path, log, experiment: str, frames: int) -> dict:
    result = run_measured(
        headless(app, experiment, work / f"{experiment}.png", ["--benchmark", str(frames)]), log)
    match = BENCHMARK_LINE.search(result["output"])
    return {"result": result, "match": match}


def render_measurements(app: Path, work: Path, log) -> list[dict]:
    out = []
    cases = (("solar_system", 600, "app.render.space_1080p", "Render Space 1080p", 60.0,
              "solar system with trails, 1920x1080, wall time per frame including the GPU"),
             ("black_hole", 300, "app.render.black_hole_1080p",
              "Render black hole 1080p (half-resolution trace)", 30.0,
              "black_hole experiment at 1920x1080, lensing traced at half resolution"))
    space_run = None
    for experiment, frames, id_, title, target, notes in cases:
        run = frame_rate(app, work, log, experiment, frames)
        code, match = run["result"]["code"], run["match"]
        if code == EXIT_NO_GL:
            out.append(skipped(id_, title, "no OpenGL 4.6 context"))
            continue
        if code != 0 or match is None:
            out.append(skipped(id_, title, f"orrery_app exited {code} without a benchmark line"))
            continue
        out.append(entry(id_, title, float(match.group(5)), "fps", "at_least", target, notes,
                         {"frames": int(match.group(1)), "mean_ms": float(match.group(4)),
                          "gpu_median_ms": float(match.group(6)),
                          "gpu_p95_ms": float(match.group(7)), "device": match.group(8).strip()}))
        if experiment == "solar_system":
            space_run = run["result"]
    out.append(memory_entry(space_run))
    return out


def memory_entry(run: dict | None) -> dict:
    title = "Idle RAM (solar system)"
    if run is None or run["peak"] is None:
        return skipped("app.memory.solar_system", title, "no measured run")
    peak, commit_peak = run["peak"]
    context = {"measure": "peak working set" if os.name == "nt" else "maximum resident set"}
    if commit_peak is not None:
        context["peak_private_mb"] = round(commit_peak / 1e6, 1)
    return entry("app.memory.solar_system", title, peak / 1e6, "MB", "below", 400.0,
                 "peak of the headless solar-system process (load, 600 frames at 1920x1080); "
                 "the window adds the UI", context)


def size_entry(app: Path) -> dict:
    libraries = (".dll",) if os.name == "nt" else (".so",)
    binary = app.stat().st_size
    shared = sum(p.stat().st_size for p in app.parent.iterdir()
                 if p.is_file() and (p.suffix in libraries or ".so." in p.name))
    assets = sum(p.stat().st_size for p in (ROOT / "assets").rglob("*") if p.is_file())
    total = binary + shared + assets
    return entry("app.size", "Binary + assets", total / 1e6, "MB", "below", 80.0,
                 "orrery_app, the shared libraries next to it (vcpkg app-local copies) and "
                 "assets/; MB = 10^6 bytes, uncompressed",
                 {"binary_mb": round(binary / 1e6, 2), "shared_libraries_mb": round(shared / 1e6, 2),
                  "assets_mb": round(assets / 1e6, 2)})


def app_measurements(build: Path, work: Path, log, runs: int) -> list[dict]:
    app = build / "src" / "app" / f"orrery_app{EXE_SUFFIX}"
    if not app.is_file():
        return [skipped("app", "Application measurements", f"{app.name} not built")]
    results = cold_start(app, work, log, runs)
    results += render_measurements(app, work, log)
    results.append(size_entry(app))
    return results


# --------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--build-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--suites", default=",".join(SUITES))
    parser.add_argument("--skip-app", action="store_true")
    parser.add_argument("--start-runs", type=int, default=5)
    parser.add_argument("--log-dir", type=Path, default=None)
    args = parser.parse_args()

    build = args.build_dir.resolve()
    log_dir = (args.log_dir or args.out.parent).resolve()
    log_dir.mkdir(parents=True, exist_ok=True)
    suites: dict[str, dict] = {}
    results: list[dict] = []
    with tempfile.TemporaryDirectory(prefix="orrery_bench_") as tmp, \
            open(log_dir / "bench.log", "w", encoding="utf-8") as log:
        work = Path(tmp)
        for suite in [s for s in args.suites.split(",") if s]:
            print(f"running suite {suite} ...", flush=True)
            suites[suite], found = run_suite(build, suite, work, log)
            results += found
        if not args.skip_app:
            print("running application measurements ...", flush=True)
            results += app_measurements(build, work, log, max(args.start_runs, 1))

    document = {
        "schema": "orrery-bench-run/1",
        "date": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d"),
        "commit": commit(),
        "build_dir": build.name,
        "machine": machine(),
        "suites": suites,
        "results": results,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    failed = [r["id"] for r in results if r.get("status") == "fail"]
    broken = [name for name, info in suites.items() if info.get("exit_code", 1) != 0]
    print(f"wrote {args.out}: {len(results)} results, {len(failed)} missed targets"
          + (f" ({', '.join(failed)})" if failed else ""))
    if broken:
        print(f"suites with failed checks: {', '.join(broken)}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
