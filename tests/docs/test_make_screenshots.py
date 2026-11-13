#!/usr/bin/env python3
"""Self-test of tools/make_screenshots.py (ctest: docs.make_screenshots_selftest).

A relative --app must run the file it names, not a program of the same name
found on PATH: the script resolves it to an absolute path before spawning.
The application is not started; subprocess.run is replaced by a recorder
that reports a failed run, so no image is written.
"""

from __future__ import annotations

import importlib.util
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("make_screenshots",
                                              REPO / "tools" / "make_screenshots.py")
assert SPEC is not None and SPEC.loader is not None
make_screenshots = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(make_screenshots)


class RelativeApp(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.work = Path(self._tmp.name).resolve()
        self.app = self.work / "orrery_app"
        self.app.write_text("not an executable\n", encoding="utf-8")
        self._cwd = Path.cwd()
        os.chdir(self.work)
        self.commands: list[list[str]] = []
        self._run = make_screenshots.subprocess.run
        self._argv = sys.argv
        make_screenshots.subprocess.run = self.record

    def tearDown(self) -> None:
        make_screenshots.subprocess.run = self._run
        sys.argv = self._argv
        os.chdir(self._cwd)
        self._tmp.cleanup()

    def record(self, command: list[str], **_: object) -> subprocess.CompletedProcess[str]:
        self.commands.append(list(command))
        return subprocess.CompletedProcess(command, 1, "", "recorded, not run")

    def test_relative_app_is_spawned_by_absolute_path(self) -> None:
        shot = make_screenshots.SHOTS[0][0]
        sys.argv = ["make_screenshots.py", "--app", "./orrery_app", "--only", shot]
        self.assertEqual(make_screenshots.main(), 1)
        self.assertEqual(len(self.commands), 1)
        program = Path(self.commands[0][0])
        self.assertTrue(program.is_absolute(), f"spawned '{program}', not an absolute path")
        self.assertEqual(program, self.app)

    def test_missing_app_is_refused(self) -> None:
        sys.argv = ["make_screenshots.py", "--app", "./no_such_app"]
        with self.assertRaises(SystemExit):
            make_screenshots.main()
        self.assertEqual(self.commands, [])


if __name__ == "__main__":
    sys.exit(0 if unittest.main(exit=False, verbosity=1).result.wasSuccessful() else 1)
