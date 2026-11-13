#!/usr/bin/env python3
"""Checks of tools/check_deps.py for orrery_profile (Phase 10a).

  * the project's CMake files as they are pass the declared-graph check;
  * orrery_profile linking a graphics library is rejected (it is graphics-free);
  * the Tracy client is approved for orrery_profile only: orrery_core linking it
    directly is reported as an unexpected dependency;
  * orrery_core must keep reaching orrery_profile.

Run directly (python3 test_check_deps_profile.py) or through ctest.
"""

from __future__ import annotations

import importlib.util
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]


def load_checker():
    spec = importlib.util.spec_from_file_location("check_deps", REPO / "tools" / "check_deps.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class ProfileGraph(unittest.TestCase):
    def setUp(self) -> None:
        self.checker = load_checker()
        self.tmp = Path(tempfile.mkdtemp(prefix="orrery_check_deps_profile_"))
        for path in self.checker.cmake_project_files():
            target = self.tmp / path.relative_to(REPO)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, target)
        self.checker.ROOT = self.tmp

    def tearDown(self) -> None:
        shutil.rmtree(self.tmp, ignore_errors=True)

    def errors(self) -> list[str]:
        errors: list[str] = []
        self.checker.check_declared_graph(errors)
        return errors

    def append(self, directory: str, text: str) -> None:
        path = self.tmp / "src" / directory / "CMakeLists.txt"
        with path.open("a", encoding="utf-8") as handle:
            handle.write("\n" + text + "\n")

    def test_project_as_is_passes(self) -> None:
        errors = self.errors()
        self.assertEqual(errors, [], "\n".join(errors))

    def test_profile_is_graphics_free(self) -> None:
        self.assertIn("orrery_profile", self.checker.GRAPHICS_FREE_TARGETS)
        self.append("core/profile", "target_link_libraries(orrery_profile PRIVATE imgui::imgui)")
        errors = self.errors()
        self.assertTrue(
            any("orrery_profile depends on forbidden library imgui::imgui" in e for e in errors),
            "\n".join(errors) or "no errors reported",
        )

    def test_tracy_only_through_profile(self) -> None:
        self.append("core", "target_link_libraries(orrery_core PRIVATE Tracy::TracyClient)")
        errors = self.errors()
        self.assertTrue(
            any("orrery_core: expected deps" in e and "Tracy::TracyClient" in e for e in errors),
            "\n".join(errors) or "no errors reported",
        )

    def test_core_reaches_profile(self) -> None:
        self.assertEqual(self.checker.EXPECTED_GRAPH["orrery_core"], {"orrery_profile"})
        for target in self.checker.EXPECTED_GRAPH:
            if target != "orrery_profile":
                self.assertIn("orrery_profile",
                              self.checker.closure(self.checker.EXPECTED_GRAPH, target), target)


if __name__ == "__main__":
    result = unittest.main(exit=False, verbosity=2).result
    sys.exit(0 if result.wasSuccessful() else 1)
