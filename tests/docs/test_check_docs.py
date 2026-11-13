#!/usr/bin/env python3
"""Self-test of tools/check_docs.py (ctest: docs.check_selftest).

Each case writes a small Markdown tree into a temporary directory and runs
the link checker on it: links that resolve must pass, broken ones must fail.
Covered: inline, reference-style (full, collapsed, shortcut) links and
images, heading anchors with inline code, repeated headings, fences of mixed
lengths, URL-encoded paths and fragments (spaces, Turkish characters), and
the check of an installed tree (--tree), where no link may leave the tree.
The rule against numbered specification references is tested on text
fragments: references that must be flagged and wording that must pass.
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

    # Reference-style links and images.
    def test_full_reference(self) -> None:
        self.assert_clean("[Guide][g]\n\n[g]: docs/guide.md\n")
        self.assert_clean('[Guide][g]\n\n[g]: docs/guide.md "The guide"\n')
        self.assert_clean("[Guide][g]\n\n  [G]: <docs/guide.md#guide>\n")
        self.assert_broken("[Guide][g]\n\n[g]: docs/missing.md\n")
        self.assert_broken("[Guide][g]\n\n[g]: docs/guide.md#no-such-heading\n")
        self.assert_broken("[Guide][nowhere]\n")

    def test_collapsed_reference(self) -> None:
        self.assert_clean("[Guide][]\n\n[guide]: docs/guide.md\n")
        self.assert_broken("[Guide][]\n\n[Guide]: docs/missing.md\n")
        self.assert_broken("[Undefined][]\n")

    def test_shortcut_reference(self) -> None:
        self.assert_clean("See [the guide].\n\n[the guide]: docs/guide.md\n")
        self.assert_broken("See [the guide].\n\n[the guide]: docs/missing.md\n")

    def test_reference_images(self) -> None:
        self.assert_clean("![Shot][s]\n\n[s]: docs/shot.png\n")
        self.assert_clean("![shot]\n\n[shot]: docs/shot.png\n")
        self.assert_broken("![Shot][s]\n\n[s]: docs/none.png\n")
        self.assert_broken("![shot]\n\n[shot]: docs/none.png\n")
        self.assert_broken("![Shot][]\n\n[shot]: docs/none.png\n")

    def test_brackets_that_are_not_links(self) -> None:
        self.assert_clean("Index [i] of a list, a task [ ] and a footnote[^1].\n\n[^1]: A note.\n")
        self.assert_clean("```\n[Guide][g]\n[g]: docs/missing.md\n```\n")
        self.assert_clean("Code `[Guide][g]` is not a link.\n")

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


class InstalledTree(unittest.TestCase):
    """check_docs.py --tree: every Markdown file of a staged install tree."""

    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.base = Path(self._tmp.name).resolve()
        self.tree = self.base / "share" / "doc" / "orrery"
        (self.tree / "docs" / "images").mkdir(parents=True)
        (self.tree / "docs" / "images" / "a.png").write_text("x\n", encoding="utf-8")
        (self.tree / "docs" / "guide.md").write_text("# Guide\n\n[Back](../README.md)\n",
                                                      encoding="utf-8")
        (self.base / "outside.md").write_text("# Outside\n", encoding="utf-8")

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def tree_errors(self, readme: str) -> list[str]:
        (self.tree / "README.md").write_text(readme, encoding="utf-8")
        return check_docs.check_tree(self.tree)

    def test_valid_tree(self) -> None:
        self.assertEqual(self.tree_errors("[G](docs/guide.md#guide) ![A](docs/images/a.png)\n"), [])

    def test_missing_target(self) -> None:
        self.assertNotEqual(self.tree_errors("[T](README.tr.md)\n"), [])
        self.assertNotEqual(self.tree_errors("![B](docs/images/b.png)\n"), [])

    def test_link_out_of_the_tree(self) -> None:
        # The target exists, but not inside the installed documentation.
        self.assertNotEqual(self.tree_errors("[O](../../../outside.md)\n"), [])

    def test_nested_file_is_checked(self) -> None:
        (self.tree / "docs" / "guide.md").write_text("[Gone](gone.md)\n", encoding="utf-8")
        self.assertNotEqual(self.tree_errors("# Readme\n"), [])


class SpecificationReferences(unittest.TestCase):
    """Numbered references into the specification, which is not in the repository."""

    def problems(self, text: str) -> list[str]:
        return check_docs.spec_reference_problems(check_docs.ROOT / "docs" / "x.md", text)

    def assert_flagged(self, text: str) -> None:
        self.assertNotEqual(self.problems(text), [], f"expected a problem in:\n{text}")

    def assert_allowed(self, text: str) -> None:
        self.assertEqual(self.problems(text), [], f"expected no problem in:\n{text}")

    def test_numbered_references_fail(self) -> None:
        self.assert_flagged("Bloom (dual filter, spec 7.6) is not done.\n")
        self.assert_flagged("The spec (7.3) asks for a disk.\n")
        self.assert_flagged("The time budget of spec section 8: under 20 minutes.\n")
        self.assert_flagged("Hill spheres (Spec sections 3.6 and 7.5).\n")
        self.assert_flagged("against the spec's 30 fps\n")
        self.assert_flagged("(spec target: at least 30 fps at 1080p)\n")
        self.assert_flagged("the specification phase 11 asks for LTO\n")
        self.assert_flagged("as planned in FAZ 9\n")
        self.assert_flagged("belirtim 8. bölüm\n")
        self.assert_flagged("şartname 4.3\n")
        self.assert_flagged("| Spec | Item | Status |\n|---|---|---|\n")

    def test_reference_across_a_line_break(self) -> None:
        problems = self.problems("Intro.\n\nThe output (spec\nsection 10) is a trace.\n")
        self.assertEqual(len(problems), 1)
        self.assertIn("x.md:3:", problems[0])

    def test_other_text_passes(self) -> None:
        self.assert_allowed("Units follow the glTF 2.0 specification.\n")
        self.assert_allowed("The entry model is ADR-012; see section 4 of this page.\n")
        self.assert_allowed("A planned bloom pass, not in V1.\n")
        self.assert_allowed("Grid specs with wrapped extents are rejected.\n")
        self.assert_allowed("Code `spec 7.6` and\n\n```\nspec section 8\n```\n")
        self.assert_allowed("| Item | Spec column elsewhere |\n")


if __name__ == "__main__":
    sys.exit(0 if unittest.main(exit=False, verbosity=1).result.wasSuccessful() else 1)
