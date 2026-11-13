#!/usr/bin/env python3
"""Self-test of tools/check_docs.py (ctest: docs.check_selftest).

Each case writes a small Markdown tree into a temporary directory and runs
the link checker on it: links that resolve must pass, broken ones must fail.
Covered: inline links, heading anchors with inline code, repeated headings,
fences of mixed lengths, and URL-encoded paths and fragments (spaces,
Turkish characters).
"""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("check_docs", REPO / "tools" / "check_docs.py")
assert SPEC is not None and SPEC.loader is not None
check_docs = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(check_docs)


class RepositoryLinks(unittest.TestCase):
    """Links in README.md and docs/, checked with the repository rules."""

    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name).resolve()
        (self.root / "docs").mkdir()
        self.write("docs/guide.md", "# Guide\n\nText.\n")
        self.write("docs/shot.png", "not really a PNG\n")
        self._saved_root = check_docs.ROOT
        check_docs.ROOT = self.root

    def tearDown(self) -> None:
        check_docs.ROOT = self._saved_root
        self._tmp.cleanup()

    def write(self, name: str, text: str) -> None:
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")

    def errors(self, readme: str) -> list[str]:
        self.write("README.md", readme)
        errors: list[str] = []
        check_docs.check_links(errors)
        return errors

    def assert_clean(self, readme: str) -> None:
        errors = self.errors(readme)
        self.assertEqual(errors, [], f"expected no problem in:\n{readme}")

    def assert_broken(self, readme: str) -> None:
        errors = self.errors(readme)
        self.assertNotEqual(errors, [], f"expected a problem in:\n{readme}")

    # Inline links (unchanged behaviour).
    def test_inline_links(self) -> None:
        self.assert_clean("[Guide](docs/guide.md) and ![Shot](docs/shot.png)\n")
        self.assert_broken("[Guide](docs/missing.md)\n")
        self.assert_broken("![Shot](docs/missing.png)\n")

    # Heading anchors.
    def test_heading_with_inline_code(self) -> None:
        self.write("docs/run.md", "# Running `orrery_app`\n\n## The `--headless` option\n")
        self.assert_clean("[Run](docs/run.md#running-orrery_app)\n")
        self.assert_clean("[Run](docs/run.md#the---headless-option)\n")
        self.assert_broken("[Run](docs/run.md#running-)\n")
        self.assert_broken("[Run](docs/run.md#running)\n")

    def test_repeated_headings(self) -> None:
        self.write("docs/setup.md", "# Setup\n\n## Setup\n\n## Setup\n")
        self.assert_clean("[A](docs/setup.md#setup) [B](docs/setup.md#setup-1) "
                          "[C](docs/setup.md#setup-2)\n")
        self.assert_broken("[D](docs/setup.md#setup-3)\n")

    def test_fences_of_mixed_lengths(self) -> None:
        self.write("docs/fence.md",
                   "````md\n```\n# Inside four\n```\n````\n\n"
                   "~~~\n```\n# Inside tilde\n~~~\n\n"
                   "# After\n")
        self.assert_clean("[After](docs/fence.md#after)\n")
        self.assert_broken("[Inside](docs/fence.md#inside-four)\n")
        self.assert_broken("[Inside](docs/fence.md#inside-tilde)\n")

    def test_same_file_fragment(self) -> None:
        self.assert_clean("# Top `code`\n\n[Top](#top-code)\n")
        self.assert_broken("# Top\n\n[Missing](#bottom)\n")

    # URL-encoded paths and fragments.
    def test_encoded_space(self) -> None:
        self.write("docs/my guide.md", "# My guide\n")
        self.assert_clean("[Guide](docs/my%20guide.md)\n")
        self.assert_clean("[Guide](docs/my%20guide.md#my-guide)\n")
        self.assert_clean("[Guide](<docs/my guide.md>)\n")
        self.assert_clean("[Guide][g]\n\n[g]: <docs/my guide.md#my-guide>\n")
        self.assert_broken("[Guide](docs/my%20guides.md)\n")

    def test_encoded_turkish(self) -> None:
        self.write("docs/çalışma notları.md", "# Çalıştırma ve Ölçüm\n")
        encoded = "docs/%C3%A7al%C4%B1%C5%9Fma%20notlar%C4%B1.md"
        fragment = "%C3%A7al%C4%B1%C5%9Ft%C4%B1rma-ve-%C3%B6l%C3%A7%C3%BCm"
        self.assert_clean(f"[Notlar]({encoded})\n")
        self.assert_clean(f"[Notlar]({encoded}#{fragment})\n")
        self.assert_clean(f"[Notlar]({encoded}#çalıştırma-ve-ölçüm)\n")
        self.assert_broken(f"[Notlar]({encoded}#%C3%A7al%C4%B1%C5%9Fma)\n")
        self.assert_clean(f"# Özet\n\n[Özet](#%C3%B6zet)\n")


if __name__ == "__main__":
    sys.exit(0 if unittest.main(exit=False, verbosity=1).result.wasSuccessful() else 1)
