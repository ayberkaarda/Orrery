#!/usr/bin/env python3
"""Negative tests for the graphics boundary of tools/check_deps.py.

Phase 4a approves GLFW, glad and glm for orrery_render. These tests make sure
that approval did not weaken the rule that orrery_core, orrery_geom and
orrery_atmo never link OpenGL, GLFW, glad or ImGui:

  * declared graph (CMake source text): a copy of the project's CMake files is
    modified so that a graphics-free library links a graphics library, directly
    or through a wrapper target, and the checker must report it; the unmodified
    copy (render linking glfw and glad) must not be reported;
  * configured graph (CMake File API): a synthetic code model in which
    orrery_core links an imported glfw target must be reported, and one in
    which only orrery_render does must not.

Run directly (python3 test_check_deps_graphics.py) or through ctest.
"""

from __future__ import annotations

import importlib.util
import json
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


class DeclaredGraph(unittest.TestCase):
    def setUp(self) -> None:
        self.checker = load_checker()
        self.tmp = Path(tempfile.mkdtemp(prefix="orrery_check_deps_"))
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

    def append(self, module: str, text: str) -> None:
        path = self.tmp / "src" / module / "CMakeLists.txt"
        with path.open("a", encoding="utf-8") as handle:
            handle.write("\n" + text + "\n")

    def test_project_as_is_passes(self) -> None:
        errors = self.errors()
        self.assertEqual(errors, [], "\n".join(errors))

    def test_render_may_link_graphics(self) -> None:
        forbidden = [e for e in self.errors() if "forbidden" in e]
        self.assertEqual(forbidden, [])

    def test_graphics_free_targets_linking_graphics_fail(self) -> None:
        cases = [
            ("core", "orrery_core", "glfw"),
            ("core", "orrery_core", "OpenGL::GL"),
            ("geom", "orrery_geom", "glad::glad"),
            ("atmo", "orrery_atmo", "imgui::imgui"),
        ]
        for module, target, library in cases:
            with self.subTest(target=target, library=library):
                # Fresh, unmodified copy for every case.
                self.tearDown()
                self.setUp()
                self.append(module, f"target_link_libraries({target} PRIVATE {library})")
                errors = self.errors()
                self.assertTrue(
                    any(f"{target} depends on forbidden library {library}" in e for e in errors),
                    "\n".join(errors) or "no errors reported",
                )

    def test_graphics_through_a_wrapper_target_fails(self) -> None:
        self.append(
            "core",
            "add_library(core_gfx_wrapper INTERFACE)\n"
            "target_link_libraries(core_gfx_wrapper INTERFACE glfw)\n"
            "target_link_libraries(orrery_core PRIVATE core_gfx_wrapper)",
        )
        errors = self.errors()
        self.assertTrue(
            any("orrery_core depends on forbidden library glfw" in e for e in errors),
            "\n".join(errors) or "no errors reported",
        )


class ConfiguredGraph(unittest.TestCase):
    """Synthetic CMake File API code model (only the fields the checker reads)."""

    def setUp(self) -> None:
        self.checker = load_checker()
        self.tmp = Path(tempfile.mkdtemp(prefix="orrery_codemodel_"))

    def tearDown(self) -> None:
        shutil.rmtree(self.tmp, ignore_errors=True)

    def run_model(self, core_links_glfw: bool) -> list[str]:
        graph = dict(self.checker.EXPECTED_GRAPH)
        targets = []
        glfw_id = "glfw::@imported"
        glfw = {
            "name": "glfw", "id": glfw_id, "type": "SHARED_LIBRARY", "imported": True,
            "abstract": True, "artifacts": [{"path": "C:/vcpkg/lib/glfw3dll.lib"}],
        }
        (self.tmp / "glfw.json").write_text(json.dumps(glfw), encoding="utf-8")
        for name, deps in graph.items():
            links = [{"id": f"{dep}::@x"} for dep in sorted(deps)]
            if name == "orrery_render" or (core_links_glfw and name == "orrery_core"):
                links.append({"id": glfw_id})
            data = {
                "name": name, "id": f"{name}::@x", "type": "STATIC_LIBRARY",
                "nameOnDisk": f"{name}.lib", "linkLibraries": links,
            }
            (self.tmp / f"{name}.json").write_text(json.dumps(data), encoding="utf-8")
            targets.append({"id": data["id"], "jsonFile": f"{name}.json"})
        config = {
            "name": "Release", "targets": targets,
            "abstractTargets": [{"id": glfw_id, "jsonFile": "glfw.json"}],
        }
        errors: list[str] = []
        self.checker.check_configuration(config, self.tmp, REPO, "synthetic", errors,
                                         self.checker.Evidence())
        return errors

    def test_render_linking_glfw_is_allowed(self) -> None:
        forbidden = [e for e in self.run_model(core_links_glfw=False) if "forbidden" in e]
        self.assertEqual(forbidden, [])

    def test_core_linking_glfw_fails(self) -> None:
        errors = self.run_model(core_links_glfw=True)
        self.assertTrue(
            any("orrery_core reaches forbidden target glfw" in e for e in errors),
            "\n".join(errors) or "no errors reported",
        )


if __name__ == "__main__":
    result = unittest.main(exit=False, verbosity=2).result
    sys.exit(0 if result.wasSuccessful() else 1)
