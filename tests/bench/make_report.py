#!/usr/bin/env python3
"""Markdown report of a benchmark run (the JSON written by run_benchmarks.py).

Prints the environment, the section 8 targets with the measured value and
pass/fail, and the context measurements, as used in docs/performance.md.

usage: make_report.py results.json [--out report.md]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

STATUS = {"pass": "pass", "fail": "**MISS**", "info": "-", "skipped": "not measured"}
# Rows in the order of the spec section 8 table; ids not listed follow.
ORDER = (
    "core.force.barnes_hut.n10k", "core.force.barnes_hut.n100k",
    "aero.lbm3d.gpu.128", "aero.lbm2d.gpu.2048x1024",
    "aero.panel3d.n2000", "aero.newtonian.n100k",
    "app.render.space_1080p", "app.render.black_hole_1080p",
    "geom.import.glb500k", "geom.import_repair.glb500k",
    "core.determinism.leapfrog_kdk", "core.determinism.yoshida4", "core.determinism.rk4",
    "app.size", "app.memory.solar_system", "app.cold_start",
)


def ordered(results: list[dict]) -> list[dict]:
    rank = {id_: i for i, id_ in enumerate(ORDER)}
    return sorted(results, key=lambda r: rank.get(r.get("id"), len(ORDER)))


def significant(value: float) -> str:
    """Three significant digits without scientific notation for usual sizes."""
    if value == 0:
        return "0"
    magnitude = abs(value)
    if magnitude >= 1000:
        return f"{value:,.0f}"
    if magnitude >= 100:
        return f"{value:.0f}"
    if magnitude >= 10:
        return f"{value:.1f}"
    if magnitude >= 1:
        return f"{value:.2f}"
    if magnitude >= 0.01:
        return f"{value:.3f}"
    return f"{value:.2e}"


def measured(result: dict) -> str:
    if result.get("status") == "skipped":
        return "-"
    if result.get("unit") == "bit-identical":
        return "yes" if result.get("value", 0) >= 1 else "no"
    return f"{significant(result['value'])} {result['unit']}"


def target(result: dict) -> str:
    goal = result.get("goal")
    if goal == "below":
        return f"< {significant(result['target'])} {result['unit']}"
    if goal == "at_least":
        if result.get("unit") == "bit-identical":
            return "bit-identical"
        return f">= {significant(result['target'])} {result['unit']}"
    return "-"


def environment(doc: dict) -> list[str]:
    m = doc.get("machine", {})
    devices = sorted({d for s in doc.get("suites", {}).values() for d in s.get("devices", [])})
    builds = {f"{s['build'].get('compiler')}, LTO {'on' if s['build'].get('lto') else 'off'}"
              for s in doc.get("suites", {}).values() if s.get("build")}
    rows = [
        ("Date", doc.get("date")),
        ("Commit", doc.get("commit")),
        ("Build tree", doc.get("build_dir")),
        ("Compiler", "; ".join(sorted(builds)) or None),
        ("OS", m.get("os")),
        ("CPU", f"{m.get('cpu')} ({m.get('logical_cpus')} logical CPUs)"),
        ("Memory", f"{m.get('memory_gib')} GiB" if m.get("memory_gib") else None),
        ("GPU (compute)", ", ".join(devices) or None),
    ]
    lines = ["| | |", "|---|---|"]
    lines += [f"| {name} | {value} |" for name, value in rows if value]
    return lines


def table(results: list[dict], with_target: bool) -> list[str]:
    if with_target:
        lines = ["| Metric | Target | Measured | Result | Notes |", "|---|---|---|---|---|"]
    else:
        lines = ["| Metric | Measured | Notes |", "|---|---|---|"]
    for r in results:
        notes = r.get("notes", "").replace("|", "/")
        if with_target:
            lines.append(f"| {r['title']} | {target(r)} | {measured(r)} | "
                         f"{STATUS.get(r.get('status'), '?')} | {notes} |")
        else:
            lines.append(f"| {r['title']} | {measured(r)} | {notes} |")
    return lines


def report(doc: dict) -> str:
    results = doc.get("results", [])
    targeted = ordered([r for r in results if r.get("goal") in ("below", "at_least")
                        or (r.get("status") == "skipped")])
    context = ordered([r for r in results if r.get("goal") == "info"])
    count = {s: sum(1 for r in targeted if r.get("status") == s)
             for s in ("pass", "fail", "skipped")}
    lines = ["## Environment", ""] + environment(doc) + [""]
    lines += ["## Section 8 targets", ""] + table(targeted, True) + [""]
    lines.append(f"{count['pass']} met, {count['fail']} missed, {count['skipped']} not measured.")
    lines += ["", "## Context measurements", ""] + table(context, False) + [""]
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("results", type=Path)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    doc = json.loads(args.results.read_text(encoding="utf-8"))
    text = report(doc)
    if args.out:
        args.out.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
