#!/usr/bin/env python3
"""Check Orrery's documentation against the repository.

Checks:
  1. Links. Every relative link and image in README.md, README.tr.md and
     docs/**/*.md resolves to an existing file or directory: inline links,
     reference-style links and images (full, collapsed and shortcut; every
     reference definition is checked, and a full or collapsed reference must
     have one), and the src/href of HTML <img> and <a>. Paths and fragments
     are URL-decoded first. A fragment into a Markdown file (or into the same
     file) must name one of its headings, with the heading anchors GitHub
     generates (inline code kept, fenced blocks of any fence length skipped,
     repeated headings numbered). External links (http, https, mailto) are
     not fetched.
  2. Physics index. docs/physics_index.json lists every implemented formula
     and model with its implementation files, the symbols each file must
     contain and the tests that verify it. docs/physics.md, section "Formula
     index", has exactly one row per entry and no row without an entry. Every
     listed file exists, every listed symbol occurs in its file as a whole
     word, and the row names every listed file and test. Every repository
     path quoted in a row exists.
  3. Approximations. Every file under src/ that carries an APPROXIMATION
     marker belongs to at least one index entry, and every short
     approximation ID quoted next to such a marker (A8, E10, D3, ...) is the
     first cell of a row in one of the approximation tables of
     docs/physics.md.

With --tree DIR only the links are checked, in every Markdown file under
DIR, and no link may point outside DIR: this validates the documentation of
a staged install tree (packaging/install_smoke.cmake).

Exit status is 0 and the last line is "OK" when every check passes;
otherwise every problem is printed on its own line and the status is 1.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parent.parent
PHYSICS_DOC = ROOT / "docs" / "physics.md"
PHYSICS_INDEX = ROOT / "docs" / "physics_index.json"
INDEX_SECTION = "Formula index"

LINK_TITLE = r"""(?:\s+(?:"[^"]*"|'[^']*'|\([^)]*\)))?"""
LINK_RE = re.compile(r"!?\[(?:[^\[\]]|\[[^\]]*\])*\]\(\s*(?:<([^>\n]+)>|([^)\s]+))"
                     + LINK_TITLE + r"\s*\)")
REF_LABEL = r"((?:[^\[\]\\]|\\.)+)"
REF_DEF_RE = re.compile(r"^ {0,3}\[" + REF_LABEL + r"\]:\s*(?:<([^>\n]*)>|(\S+))"
                        + LINK_TITLE + r"\s*$")
REF_USE_RE = re.compile(r"!?\[" + REF_LABEL + r"\]\[((?:[^\[\]\\]|\\.)*)\]")
HTML_SRC_RE = re.compile(r"""<(?:img|a)\b[^>]*?\b(?:src|href)\s*=\s*["']([^"']+)["']""", re.I)
HEADING_RE = re.compile(r"^(#{1,6})\s+(.*?)\s*#*\s*$")
FENCE_RE = re.compile(r"^ {0,3}(`{3,}|~{3,})(.*)$")
INLINE_CODE_RE = re.compile(r"(`+)(?!`).*?(?<!`)\1(?!`)")
ROW_ID_RE = re.compile(r"^\|\s*`([a-z0-9_.]+)`\s*\|")
REPO_PATH_RE = re.compile(r"`((?:src|tests|tools|docs|assets|cmake)/[^`\s]+)`")
APPROX_TABLE_ROW_RE = re.compile(r"^\|\s*([A-Z]\d{1,2})\s*\|")
APPROX_ID_RE = re.compile(r"\b([ABDEGLPRV]\d{1,2})\b")
SOURCE_SUFFIXES = {".h", ".hpp", ".cpp", ".glsl"}
EXTERNAL_PREFIXES = ("http://", "https://", "mailto:")


def rel(path: Path, base: Path | None = None) -> str:
    try:
        return path.resolve().relative_to(base or ROOT).as_posix()
    except ValueError:
        return path.as_posix()


def read_text(path: Path, errors: list[str]) -> str | None:
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as exc:
        errors.append(f"{rel(path)}: cannot read ({exc})")
        return None


def strip_fences(lines: list[str]) -> list[str]:
    """Blank fenced code blocks, keeping line numbers.

    A fence closes only with the character that opened it, at least as many
    times and with nothing after it, so a ```` block may contain ``` lines.
    """
    out: list[str] = []
    fence: tuple[str, int] | None = None
    for line in lines:
        match = FENCE_RE.match(line)
        if fence is None:
            if match and not (match.group(1)[0] == "`" and "`" in match.group(2)):
                fence = (match.group(1)[0], len(match.group(1)))
                out.append("")
            else:
                out.append(line)
            continue
        out.append("")
        if (match and match.group(1)[0] == fence[0] and len(match.group(1)) >= fence[1]
                and not match.group(2).strip()):
            fence = None
    return out


def strip_code(lines: list[str]) -> list[str]:
    """Blank fenced blocks and inline code spans, keeping line numbers."""
    return [INLINE_CODE_RE.sub("", line) for line in strip_fences(lines)]


def github_slug(heading: str) -> str:
    """The anchor GitHub gives a heading: the text of inline code is kept."""
    text = re.sub(r"!?\[([^\]]*)\]\([^)]*\)", r"\1", heading)
    text = text.replace("`", "").strip().lower()
    text = re.sub(r"[^\w\- ]", "", text)
    return text.replace(" ", "-")


def anchors_of(text: str) -> set[str]:
    anchors: set[str] = set()
    counts: dict[str, int] = {}
    for line in strip_fences(text.splitlines()):
        match = HEADING_RE.match(line)
        if not match:
            continue
        base = github_slug(match.group(2))
        seen = counts.get(base, 0)
        counts[base] = seen + 1
        anchors.add(base if seen == 0 else f"{base}-{seen}")
    return anchors


def markdown_files() -> list[Path]:
    files = [ROOT / "README.md", ROOT / "README.tr.md"]
    files.extend(sorted((ROOT / "docs").rglob("*.md")))
    return [f for f in files if f.is_file()]


def reference_label(label: str) -> str:
    return " ".join(label.split()).casefold()


def scan_links(text: str) -> tuple[list[tuple[int, str]], list[tuple[int, str]]]:
    """Return (line, target) for every link, and (line, label) for every full or
    collapsed reference without a definition."""
    found: list[tuple[int, str]] = []
    uses: list[tuple[int, str]] = []
    defined: set[str] = set()
    for number, line in enumerate(strip_code(text.splitlines()), start=1):
        definition = REF_DEF_RE.match(line)
        if definition and not definition.group(1).startswith("^"):
            defined.add(reference_label(definition.group(1)))
            found.append((number, definition.group(2) or definition.group(3) or ""))
            continue
        for match in LINK_RE.finditer(line):
            found.append((number, match.group(1) or match.group(2)))
        for match in REF_USE_RE.finditer(line):
            label = match.group(2) or match.group(1)
            if not label.startswith("^"):
                uses.append((number, label))
        for match in HTML_SRC_RE.finditer(line):
            found.append((number, match.group(1)))
    undefined = [(n, label) for n, label in uses if reference_label(label) not in defined]
    return found, undefined


def check_link(source: Path, number: int, target: str, cache: dict[Path, set[str]],
               errors: list[str], tree: Path | None = None) -> None:
    if target.startswith(EXTERNAL_PREFIXES):
        return
    path_part, _, fragment = target.partition("#")
    path_part, fragment = unquote(path_part), unquote(fragment)
    resolved = (source.parent / path_part).resolve() if path_part else source.resolve()
    where = f"{rel(source, tree)}:{number}"
    if tree is not None and resolved != tree and tree not in resolved.parents:
        errors.append(f"{where}: link '{target}' points outside {tree.as_posix()}")
        return
    if not resolved.exists():
        errors.append(f"{where}: broken link '{target}' ({rel(resolved, tree)} does not exist)")
        return
    if not fragment or resolved.suffix.lower() != ".md" or resolved.is_dir():
        return
    if resolved not in cache:
        text = read_text(resolved, errors)
        cache[resolved] = anchors_of(text) if text is not None else set()
    if fragment.lower() not in cache[resolved]:
        errors.append(f"{where}: link '{target}' names no heading of {rel(resolved, tree)}")


def check_files(files: list[Path], errors: list[str], tree: Path | None = None) -> int:
    cache: dict[Path, set[str]] = {}
    checked = 0
    for source in files:
        text = read_text(source, errors)
        if text is None:
            continue
        targets, undefined = scan_links(text)
        for number, target in targets:
            check_link(source, number, target, cache, errors, tree)
            checked += 1
        for number, label in undefined:
            errors.append(f"{rel(source, tree)}:{number}: reference '[{label}]' has no "
                          "definition")
    return checked


def check_links(errors: list[str]) -> int:
    return check_files(markdown_files(), errors)


def check_tree(tree: Path) -> list[str]:
    """Check the links of every Markdown file under tree, which none may leave."""
    tree = tree.resolve()
    errors: list[str] = []
    files = sorted(tree.rglob("*.md"))
    if not files:
        errors.append(f"{tree.as_posix()}: no Markdown file")
    check_files(files, errors, tree)
    return errors


def section_lines(text: str, title: str) -> list[str]:
    lines = text.splitlines()
    start = None
    for i, line in enumerate(lines):
        match = HEADING_RE.match(line)
        if match and match.group(2).strip() == title:
            start = i + 1
            level = len(match.group(1))
            break
    if start is None:
        return []
    end = len(lines)
    for j in range(start, len(lines)):
        match = HEADING_RE.match(lines[j])
        if match and len(match.group(1)) <= level:
            end = j
            break
    return lines[start:end]


def index_rows(text: str, errors: list[str]) -> dict[str, str]:
    rows: dict[str, str] = {}
    lines = section_lines(text, INDEX_SECTION)
    if not lines:
        errors.append(f"docs/physics.md: no section '{INDEX_SECTION}'")
    for line in lines:
        match = ROW_ID_RE.match(line)
        if not match:
            continue
        row_id = match.group(1)
        if row_id in rows:
            errors.append(f"docs/physics.md: duplicate formula row '{row_id}'")
        rows[row_id] = line
    return rows


def load_index(errors: list[str]) -> list[dict]:
    text = read_text(PHYSICS_INDEX, errors)
    if text is None:
        return []
    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        errors.append(f"docs/physics_index.json: invalid JSON ({exc})")
        return []
    entries = data.get("entries") if isinstance(data, dict) else None
    if not isinstance(entries, list):
        errors.append("docs/physics_index.json: expected an object with an 'entries' array")
        return []
    return entries


def entry_problem(entry: object) -> str | None:
    if not isinstance(entry, dict):
        return "entry is not an object"
    if not isinstance(entry.get("id"), str):
        return "entry without a string 'id'"
    implementation = entry.get("implementation")
    if not isinstance(implementation, dict) or not implementation:
        return f"{entry['id']}: 'implementation' must map files to symbol lists"
    for symbols in implementation.values():
        if not isinstance(symbols, list) or not all(isinstance(s, str) for s in symbols):
            return f"{entry['id']}: every implementation file needs a list of symbols"
    tests = entry.get("tests")
    if not isinstance(tests, list) or not tests or not all(isinstance(t, str) for t in tests):
        return f"{entry['id']}: 'tests' must be a non-empty list of files"
    return None


def check_symbols(entry_id: str, file_name: str, symbols: list[str],
                  errors: list[str]) -> None:
    path = ROOT / file_name
    if not path.is_file():
        errors.append(f"{entry_id}: implementation file {file_name} does not exist")
        return
    text = read_text(path, errors)
    if text is None:
        return
    for symbol in symbols:
        if not re.search(r"(?<![\w])" + re.escape(symbol) + r"(?![\w])", text):
            errors.append(f"{entry_id}: symbol '{symbol}' not found in {file_name}")


def check_entry(entry: dict, row: str, errors: list[str]) -> None:
    entry_id = entry["id"]
    for file_name, symbols in entry["implementation"].items():
        check_symbols(entry_id, file_name, symbols, errors)
        if f"`{file_name}`" not in row:
            errors.append(f"{entry_id}: the physics.md row does not name {file_name}")
    for test in entry["tests"]:
        if not (ROOT / test).is_file():
            errors.append(f"{entry_id}: test file {test} does not exist")
        if f"`{test}`" not in row:
            errors.append(f"{entry_id}: the physics.md row does not name {test}")
    for quoted in REPO_PATH_RE.findall(row):
        if not (ROOT / quoted.rstrip(".,;")).exists():
            errors.append(f"{entry_id}: the physics.md row names {quoted}, which does not exist")


def check_physics_index(errors: list[str]) -> tuple[list[dict], set[str]]:
    text = read_text(PHYSICS_DOC, errors)
    rows = index_rows(text, errors) if text is not None else {}
    entries: list[dict] = []
    for raw in load_index(errors):
        problem = entry_problem(raw)
        if problem:
            errors.append(f"docs/physics_index.json: {problem}")
            continue
        entries.append(raw)
    ids = [e["id"] for e in entries]
    for entry_id in sorted({i for i in ids if ids.count(i) > 1}):
        errors.append(f"docs/physics_index.json: duplicate id '{entry_id}'")
    for entry in entries:
        row = rows.get(entry["id"])
        if row is None:
            errors.append(f"{entry['id']}: indexed but has no row in docs/physics.md")
            continue
        check_entry(entry, row, errors)
    for row_id in sorted(set(rows) - set(ids)):
        errors.append(f"{row_id}: row in docs/physics.md without an index entry")
    approx_ids = set()
    if text is not None:
        for line in text.splitlines():
            match = APPROX_TABLE_ROW_RE.match(line)
            if match:
                approx_ids.add(match.group(1))
    return entries, approx_ids


def approximation_sites() -> dict[str, list[tuple[int, str]]]:
    sites: dict[str, list[tuple[int, str]]] = {}
    for path in sorted((ROOT / "src").rglob("*")):
        if path.suffix not in SOURCE_SUFFIXES or not path.is_file():
            continue
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except (OSError, UnicodeDecodeError):
            continue
        hits = [(n, line) for n, line in enumerate(lines, start=1) if "APPROXIMATION" in line]
        if hits:
            sites[path.relative_to(ROOT).as_posix()] = hits
    return sites


def check_approximations(entries: list[dict], approx_ids: set[str],
                         errors: list[str]) -> int:
    covered = {name for e in entries for name in e["implementation"]}
    sites = approximation_sites()
    for file_name, hits in sites.items():
        if file_name not in covered:
            errors.append(f"{file_name}: has APPROXIMATION markers but no physics index entry")
        for number, line in hits:
            after = line.split("APPROXIMATION", 1)[1][:40]
            for approx_id in APPROX_ID_RE.findall(after):
                if approx_id not in approx_ids:
                    errors.append(f"{file_name}:{number}: APPROXIMATION {approx_id} has no "
                                  "row in the approximation tables of docs/physics.md")
    return sum(len(h) for h in sites.values())


def report(errors: list[str]) -> int:
    for error in errors:
        print(f"error: {error}")
    if errors:
        print(f"FAILED ({len(errors)} problems)")
        return 1
    print("OK")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--tree", type=Path,
                        help="check only the links of the Markdown files under this directory")
    args = parser.parse_args()
    if args.tree is not None:
        files = len(list(args.tree.resolve().rglob("*.md")))
        print(f"links: {files} files under {args.tree.resolve().as_posix()}")
        return report(check_tree(args.tree))
    errors: list[str] = []
    links = check_links(errors)
    print(f"links: {links} links checked in {len(markdown_files())} files")
    entries, approx_ids = check_physics_index(errors)
    print(f"physics index: {len(entries)} entries, {len(approx_ids)} approximation IDs")
    markers = check_approximations(entries, approx_ids, errors)
    print(f"approximations: {markers} markers checked")
    return report(errors)


if __name__ == "__main__":
    sys.exit(main())
