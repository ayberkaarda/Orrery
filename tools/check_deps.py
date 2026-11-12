#!/usr/bin/env python3
"""Enforce Orrery's architectural boundaries.

Checks:
  1. The declared library graph in the project's CMake files matches the
     intended target graph exactly (direct dependencies). Link-affecting
     commands other than target_link_libraries (set_property,
     set_target_properties, link_libraries, target_link_options,
     add_link_options) are inspected as well, and a target name that cannot be
     resolved to a literal name fails the check.
  2. For every configured build tree (given with --build-dir, or every tree
     under build/), and separately for every configuration of that tree, the
     real target graph reported by the CMake File API (codemodel >= 2.9):
       - first-party direct link dependencies match the intended graph, and
         the first-party targets reachable from each target (also through
         non-first-party wrapper, interface and imported targets) match the
         intended transitive closure exactly;
       - orrery_core, orrery_geom, orrery_atmo and orrery_sim do not reach OpenGL, GLFW or
         ImGui through linkLibraries, interfaceLinkLibraries, build or compile
         dependencies, imported/interface (abstract) targets, raw link
         fragments or artifact names; an imported library without artifact
         evidence (for example an UNKNOWN imported target) is rejected;
       - the final link line of every executable that links only graphics-free
         first-party libraries contains no graphics library (this covers
         INTERFACE_LINK_OPTIONS and imported locations); at least one such
         consumer must exist for each graphics-free library;
       - no compile or link command fragment enables fast-math, contraction
         or another value-changing floating-point mode, and the last
         floating-point model option of every first-party compile command is
         the deterministic one (/fp:precise or /fp:strict, -ffp-contract=off).
     The final commands in compile_commands.json are checked the same way.
  3. Physics sources contain no single-precision floating-point type, literal,
     function or constant, and no fast-math pragma or attribute. The scanned
     set is every file under src/{core,geom,aero,atmo}, every source the
     physics targets compile according to the File API, every file in their
     first-party include directories, and every header they include that
     resolves to a file. Comments and string literals are ignored, line
     splices are joined first, and token pasting is rejected because its
     result cannot be checked without preprocessing.

Coverage: only the configured trees and their configurations are checked.
Conditions on other platforms, ORRERY_HEADLESS values or build types (for
example $<CONFIG:Debug> on a Release tree) are not evaluated; CI coverage of
those trees is a workflow decision, not something this tool can detect.

Missing evidence is a failure: a build tree without a usable code model, or a
codemodel version that does not report link libraries, fails the check. Use
--source-only to run only the source checks (1 and 3) before configuring.
--source-only is advisory: it parses CMake text and cannot evaluate it, so
only the configured check is authoritative.

Exit status is 0 and the last line is "OK" when every check passes.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

EXPECTED_GRAPH: dict[str, set[str]] = {
    "orrery_core": set(),
    "orrery_geom": {"orrery_core"},
    # Phase 6b: the URL model loader (src/geom/net) downloads and unpacks
    # files only; it never parses meshes, so it does not need orrery_geom.
    "orrery_geom_net": {"orrery_core"},
    "orrery_atmo": {"orrery_core"},
    "orrery_aero": {"orrery_core", "orrery_geom"},
    "orrery_scene": {"orrery_core", "orrery_geom", "orrery_aero", "orrery_atmo"},
    "orrery_render": {"orrery_core", "orrery_geom", "orrery_aero"},
    # Phase 4c: the simulation side of the application (physics thread,
    # commands, experiments) is graphics-free; the command line and headless
    # renderer join it with render; the executable links only those.
    "orrery_sim": {"orrery_core", "orrery_scene"},
    "orrery_app_core": {"orrery_sim", "orrery_render"},
    "orrery_app": {"orrery_app_core"},
}

# Approved third-party link dependencies, per first-party target, as spelled
# in target_link_libraries. Anything else in the declared graph fails.
# orrery_core: oneTBB (Apache-2.0, approved for Phase 3a) runs the per-body
# Barnes-Hut force loop in parallel. It is a CPU threading library with no
# graphics dependency; the graphics-free checks below still inspect it.
# orrery_render (Phase 4a): GLFW (window and context), glad (OpenGL loader)
# and glm (header-only render math). Graphics libraries are approved for
# orrery_render only; GRAPHICS_FREE_TARGETS below are still checked.
# orrery_render (Phase 4b): Dear ImGui (docking branch, GLFW and OpenGL 3
# backends) and ImPlot for the UI layer, both private.
# orrery_sim (Phase 4c): oneTBB's concurrent_queue carries SimCommands from the
# render thread to the physics thread, private like in core.
# orrery_geom_net (Phase 6b): libcurl (curl licence, https transport over
# OpenSSL) and miniz (MIT, zip reading), both private and graphics-free.
THIRD_PARTY_DEPS: dict[str, set[str]] = {
    "orrery_geom_net": {"CURL::libcurl", "miniz::miniz"},
    "orrery_core": {"TBB::tbb"},
    # orrery_geom (Phase 6a): assimp (BSD-3-Clause) imports OBJ, STL, glTF 2.0,
    # PLY and FBX models; private, graphics-free, single precision only
    # behind its API (converted to double on read).
    "orrery_geom": {"assimp::assimp"},
    # orrery_aero (Phase 7a): Eigen (MPL-2.0, header-only) solves the panel
    # method's linear systems (dense LU, GMRES); private, graphics-free, used
    # in double precision only.
    "orrery_aero": {"Eigen3::Eigen"},
    "orrery_sim": {"TBB::tbb"},
    "orrery_scene": {"nlohmann_json::nlohmann_json"},
    "orrery_render": {"glfw", "glad::glad", "glm::glm-header-only", "imgui::imgui",
                      "implot::implot"},
    # orrery_aero (Phase 7b): oneTBB runs the CPU lattice Boltzmann step over
    # rows, private and graphics-free as in orrery_core.
    "orrery_aero": {"TBB::tbb"},
}

# Imported targets created by CMake's own Find modules (FindZLIB, FindOpenSSL)
# are UNKNOWN_LIBRARY targets whose file the code model does not report on
# Linux, so the artifact check cannot inspect them. They are plain system
# compression and TLS libraries reached through assimp, curl and miniz; the
# names are matched exactly so any other artifact-less import still fails.
UNVERIFIABLE_IMPORTED_OK = frozenset({"ZLIB::ZLIB", "OpenSSL::SSL", "OpenSSL::Crypto"})

# orrery_aero (Phase 7b) reaches the GPU only through IComputeBackend; the
# OpenGL implementation lives in orrery_render.
GRAPHICS_FREE_TARGETS = ("orrery_core", "orrery_geom", "orrery_atmo", "orrery_sim",
                         "orrery_geom_net", "orrery_aero")
PHYSICS_TARGETS = ("orrery_core", "orrery_geom", "orrery_aero", "orrery_atmo")

# Graphics libraries, matched against each link name after the linker-flag
# spelling (-l, -l:, -Wl,, LINKER:, /DEFAULTLIB:, -framework), directories,
# the lib prefix, the file extension and generator expressions are removed.
FORBIDDEN_NAME = re.compile(
    r"^(?:gl|egl|glx|gles\w*|glu(?:32)?|glut\w*|gl3w|glbinding\w*|osmesa\w*|mesa\w*)$",
    re.IGNORECASE,
)
FORBIDDEN_PART = re.compile(r"(opengl|glad|glfw|glew|imgui|implot|freeglut|epoxy)", re.IGNORECASE)
GENEX_OPEN = re.compile(r"\$<[A-Za-z_]+(?:\s*:)?")
LIBRARY_EXT = re.compile(r"\.(?:so(?:\.\d+)*|a|lib|dylib|tbd|dll|framework)$", re.IGNORECASE)

# File API codemodel version that introduced linkLibraries,
# interfaceLinkLibraries and abstractTargets (CMake 4.1).
MIN_CODEMODEL = (2, 9)

# Options that change floating-point results (fast-math, contraction,
# reassociation, flush-to-zero, approximations).
FAST_MATH_FLAG = re.compile(
    r"(?<![\w-])("
    r"-ffast-math|-Ofast|[-/]fp:fast\b|[-/]fp:contract\b|[-/]Qfast_transcendentals\b"
    r"|-funsafe-math-optimizations|-fassociative-math|-freciprocal-math"
    r"|-ffp-contract=(?:fast[\w-]*|on)\b|-ffp-model=(?:fast|aggressive)\b"
    r"|-ffinite-math-only|-fno-signed-zeros|-fno-honor-nans|-fno-honor-infinities"
    r"|-fapprox-func|-fcx-limited-range|-fcx-fortran-rules|-mdaz-ftz"
    r"|-menable-unsafe-fp-math|-mrecip(?:=\S*)?"
    r")"
)
MSVC_FP_MODEL = re.compile(r"(?<![\w-])[-/]fp:(precise|strict|fast)\b")
GNU_FP_CONTRACT = re.compile(r"(?<![\w-])-ffp-contract=([\w-]+)")

PRECISION_CHECKED_DIRS = ("core", "geom", "aero", "atmo")
# Files allowed to use single precision (GPU upload boundary). Paths are
# relative to the repository root, forward slashes.
# src/aero/compute/float32_pack.cpp (Phase 7b): binary32 packing of host
# doubles for device buffers of IComputeBackend, the GPU upload boundary.
FLOAT_ALLOWLIST: frozenset[str] = frozenset({"src/aero/compute/float32_pack.cpp"})
# Every file in a scanned directory is treated as source except these
# (documentation, build scripts, data and binary formats).
NON_SOURCE_SUFFIXES = {
    ".txt", ".md", ".rst", ".cmake", ".json", ".yml", ".yaml", ".toml", ".ini",
    ".png", ".jpg", ".jpeg", ".gif", ".bmp", ".svg", ".ico", ".pdf",
    ".zip", ".gz", ".7z", ".obj", ".o", ".lib", ".a", ".so", ".dll", ".exe", ".pdb",
}
FLOAT_TYPE = re.compile(
    r"\b(float|float_t|_Float16|_Float32x?|__fp16|__bf16|b?float(?:16|32)_t"
    r"|float\d+x\d+(?:x\d+)?_t|__m128|__m256|__m512|__m128h|__m256h|__m512h"
    r"|GLfloat|GLclampf|cl_float\d*"
    r"|(?:Vector|RowVector|Matrix|Array|Quaternion|AngleAxis|Rotation2D|Translation"
    r"|Affine|Isometry|Projective|Transform|AlignedBox)[0-9X]*f"
    r"|glm::(?:f?vec[1-4]|f?mat[2-4](?:x[2-4])?|f?quat|f32\w*))\b"
)
# Decimal and hexadecimal floating literals with an f/F suffix:
# 1.0f, 1.f, .5f, 1e3f, 1'000.5F, 0x1.8p3f.
FLOAT_LITERAL = re.compile(
    r"(?<![\w.'])("
    r"0[xX][0-9a-fA-F']*\.?[0-9a-fA-F']*[pP][+-]?[0-9']+"
    r"|(?:[0-9][0-9']*\.[0-9']*|\.[0-9][0-9']*)(?:[eE][+-]?[0-9']+)?"
    r"|[0-9][0-9']*[eE][+-]?[0-9']+"
    r")[fF](?![\w])"
)
# Single-precision C library functions and float-typed constants.
FLOAT_FUNCTION = re.compile(
    r"\b((?:sqrt|cbrt|hypot|sin|cos|tan|asin|acos|atan|atan2|sinh|cosh|tanh|asinh"
    r"|acosh|atanh|exp|exp2|expm1|log|log10|log2|log1p|pow|fabs|fmod|remainder"
    r"|remquo|fma|fmax|fmin|fdim|floor|ceil|trunc|round|lround|llround|nearbyint"
    r"|rint|lrint|llrint|frexp|ldexp|modf|scalbn|scalbln|ilogb|logb|nextafter"
    r"|nexttoward|copysign|nan|erf|erfc|tgamma|lgamma|strto)f|stof)\b"
)
FLOAT_CONSTANT = re.compile(r"\b(FLT_(?!EVAL_METHOD\b|RADIX\b|ROUNDS\b)[A-Z0-9_]+|HUGE_VALF)\b")
TOKEN_PASTE = re.compile(r"##")
# Calls whose concatenated arguments could be pasted into a float token.
CALL_ARGS = re.compile(r"\b[A-Za-z_]\w*\s*\(([^()]*)\)")
FAST_MATH_WORD = (
    r"(fast-math|fast_math|Ofast|unsafe-math|associative-math|reciprocal-math"
    r"|finite-math|signed-zeros|honor-nans|honor-infinities|fp-contract|fp:fast|approx-func)"
)
FAST_MATH_SOURCE = re.compile(
    r"((?:#\s*pragma|_Pragma|__pragma|optimize\s*\().*" + FAST_MATH_WORD
    + r"|#\s*pragma\s+STDC\s+FP_CONTRACT\s+ON"
    + r"|(?:#\s*pragma|_Pragma|__pragma).*float_control\s*\(\s*(?:precise|except)\s*,\s*off"
    + r"|(?:#\s*pragma|_Pragma|__pragma).*fp_contract\s*\(\s*on"
    + r"|(?:#\s*pragma|_Pragma|__pragma).*clang\s+fp\s.*"
    + r"(?:contract\s*\(\s*(?:fast|on)|reassociate\s*\(\s*on|reciprocal\s*\(\s*on))",
    re.IGNORECASE,
)
INCLUDE_DIRECTIVE = re.compile(r'^\s*#\s*include\s*([<"])([^>"]+)[>"]', re.MULTILINE)

LINK_LIBRARY_PROPERTIES = {
    "LINK_LIBRARIES", "INTERFACE_LINK_LIBRARIES", "INTERFACE_LINK_LIBRARIES_DIRECT",
    "LINK_INTERFACE_LIBRARIES", "IMPORTED_LINK_INTERFACE_LIBRARIES",
    "IMPORTED_LINK_DEPENDENT_LIBRARIES",
}
LINK_OPTION_PROPERTIES = {
    "LINK_OPTIONS", "INTERFACE_LINK_OPTIONS", "LINK_FLAGS", "STATIC_LIBRARY_OPTIONS",
    "STATIC_LIBRARY_FLAGS", "IMPORTED_LOCATION", "IMPORTED_IMPLIB",
}
TLL_KEYWORDS = {"PUBLIC", "PRIVATE", "INTERFACE", "LINK_PUBLIC", "LINK_PRIVATE",
                "LINK_INTERFACE_LIBRARIES", "debug", "optimized", "general"}
CMAKE_COMMAND = re.compile(
    r"\b(target_link_libraries|target_link_options|set_property|set_target_properties"
    r"|link_libraries|add_link_options|set|function|macro|endfunction|endmacro"
    r"|add_library|add_executable)\s*\(",
    re.IGNORECASE,
)


def normalize(name: str) -> str:
    """Map alias spellings (orrery::core) onto real target names."""
    if name.startswith("orrery::"):
        return "orrery_" + name[len("orrery::"):]
    return name


def link_names(token: str) -> list[str]:
    """Candidate library names for one link item token."""
    token = token.strip()
    for prefix in ("-l:", "-l", "/DEFAULTLIB:", "-DEFAULTLIB:"):
        if token[: len(prefix)].lower() == prefix.lower() and len(token) > len(prefix):
            token = token[len(prefix):]
            break
    base = re.split(r"[\\/]", token)[-1]
    while True:
        stripped = LIBRARY_EXT.sub("", base)
        if stripped == base:
            break
        base = stripped
    if base[:3].lower() == "lib" and len(base) > 3:
        base = base[3:]
    names = [base] + [part for part in base.split("::") if part]
    return names


def forbidden_item(item: str) -> str | None:
    """Return the graphics library named by a link item, or None."""
    text = GENEX_OPEN.sub(" ", item).replace(">", " ")
    for token in re.split(r"[\s,;\"']+|LINKER:|-Wl|-Xlinker", text):
        if not token:
            continue
        for name in link_names(token):
            if FORBIDDEN_PART.search(name) or FORBIDDEN_NAME.match(name):
                return token
    return None


def closure(graph: dict[str, set[str]], start: str) -> set[str]:
    seen: set[str] = set()
    stack = list(graph.get(start, ()))
    while stack:
        node = stack.pop()
        if node in seen:
            continue
        seen.add(node)
        stack.extend(graph.get(node, ()))
    return seen


def rel_display(path: Path) -> str:
    try:
        return path.resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return path.as_posix()


def read_text_checked(path: Path, errors: list[str], tag: str) -> str | None:
    """Read a UTF-8 file; report a clean error instead of raising."""
    try:
        return path.read_bytes().decode("utf-8")
    except UnicodeDecodeError as exc:
        errors.append(
            f"[{tag}] {rel_display(path)}: not valid UTF-8 (byte 0x{exc.object[exc.start]:02x} "
            f"at offset {exc.start}); the file cannot be checked"
        )
    except OSError as exc:
        errors.append(f"[{tag}] {rel_display(path)}: cannot be read ({exc.strerror})")
    return None


# --------------------------------------------------------------------------
# Declared graph (CMake source text)
# --------------------------------------------------------------------------


def strip_cmake_comments(text: str) -> str:
    out = []
    for line in text.splitlines():
        in_quote = False
        cut = len(line)
        for index, char in enumerate(line):
            if char == '"' and (index == 0 or line[index - 1] != "\\"):
                in_quote = not in_quote
            elif char == "#" and not in_quote:
                cut = index
                break
        out.append(line[:cut])
    return "\n".join(out)


def cmake_arguments(text: str, start: int) -> tuple[list[str], int]:
    """Split the arguments of the command whose '(' precedes `start`."""
    args: list[str] = []
    depth, i, n = 1, start, len(text)
    current = ""
    while i < n and depth:
        char = text[i]
        if char == '"':
            end = i + 1
            while end < n and text[end] != '"':
                end += 2 if text[end] == "\\" else 1
            current += text[i + 1:end]
            i = end + 1
            continue
        if char == "(":
            depth += 1
        elif char == ")":
            depth -= 1
            if depth == 0:
                break
        if char.isspace() and depth == 1:
            if current:
                args.append(current)
            current = ""
        else:
            current += char
        i += 1
    if current:
        args.append(current)
    return args, i


def unwrap_genex(token: str) -> list[str]:
    text = GENEX_OPEN.sub(" ", token).replace(">", " ")
    return [t for t in re.split(r"[\s;,]+", text) if t]


def cmake_project_files() -> list[Path]:
    files = [ROOT / "CMakeLists.txt"]
    files += sorted((ROOT / "cmake").glob("*.cmake"))
    for subdir in ("src", "tests"):
        base = ROOT / subdir
        if base.is_dir():
            files += sorted(p for p in base.rglob("*")
                            if p.is_file() and (p.name == "CMakeLists.txt" or p.suffix == ".cmake"))
    return [p for p in files if p.is_file()]


def declared_graph(errors: list[str]) -> tuple[dict[str, set[str]], set[str],
                                               dict[str, list[tuple[str, str]]]]:
    """Return (dependency graph of every literal target, targets defined under
    src/, link options per target as (location, item))."""
    graph: dict[str, set[str]] = {}
    defined: set[str] = set()
    options: dict[str, list[tuple[str, str]]] = {}
    variables: dict[str, str] = {}
    known = set(EXPECTED_GRAPH) | {"orrery::" + t[len("orrery_"):] for t in EXPECTED_GRAPH}
    src_dir = ROOT / "src"

    def resolve(token: str) -> str:
        for _ in range(8):
            new = re.sub(r"\$\{(\w+)\}",
                         lambda m: variables.get(m.group(1), m.group(0)), token)
            if new == token:
                break
            token = new
        return token

    for cmake_file in cmake_project_files():
        raw = read_text_checked(cmake_file, errors, "graph")
        if raw is None:
            continue
        text = strip_cmake_comments(raw)
        rel = rel_display(cmake_file)
        under_src = src_dir in cmake_file.parents
        params: list[set[str]] = []
        pos = 0
        while True:
            match = CMAKE_COMMAND.search(text, pos)
            if match is None:
                break
            command = match.group(1).lower()
            args, pos = cmake_arguments(text, match.end())
            line = text.count("\n", 0, match.start()) + 1
            where = f"{rel}:{line}"

            if command in ("function", "macro"):
                params.append(set(args[1:]) | {"ARGN", "ARGV", "ARGC"}
                              | {f"ARGV{i}" for i in range(10)})
                continue
            if command in ("endfunction", "endmacro"):
                if params:
                    params.pop()
                continue
            if command == "set":
                if args and "${" not in args[0]:
                    values = [a for a in args[1:] if a not in ("PARENT_SCOPE", "CACHE")]
                    variables[args[0]] = ";".join(resolve(v) for v in values)
                continue
            if command in ("add_library", "add_executable"):
                if under_src and args:
                    defined.add(resolve(args[0]))
                    graph.setdefault(resolve(args[0]), set())
                continue

            def targets_of(names: list[str]) -> list[str]:
                """Resolve target names; fail on names that cannot be resolved."""
                result = []
                for name in names:
                    resolved = resolve(name)
                    if "${" not in resolved and "$<" not in resolved:
                        result.append(normalize(resolved))
                        continue
                    unresolved = set(re.findall(r"\$\{(\w+)\}", resolved))
                    if params and unresolved <= set().union(*params):
                        continue  # helper function applied to caller-provided targets
                    pattern = re.sub(r"\\\$\\\{\w+\\\}|\\\$<.*", ".*", re.escape(resolved))
                    if any(re.fullmatch(pattern, k) for k in known):
                        errors.append(
                            f"[graph] {where}: {command}() target '{name}' cannot be "
                            f"resolved to a literal name; the source check cannot verify it"
                        )
                return result

            def record(target: str, values: list[str], libraries: bool) -> None:
                items = [i for value in values for i in unwrap_genex(resolve(value))
                         if i not in TLL_KEYWORDS]
                if libraries:
                    graph.setdefault(target, set()).update(normalize(i) for i in items)
                else:
                    options.setdefault(target, []).extend((f"{where}: {command}()", i)
                                                          for i in items)

            if command == "target_link_libraries" and args:
                for target in targets_of(args[:1]):
                    record(target, [a for a in args[1:] if a not in TLL_KEYWORDS], True)
            elif command == "target_link_options" and args:
                for target in targets_of(args[:1]):
                    record(target, [a for a in args[1:]
                                    if a not in TLL_KEYWORDS and a != "BEFORE"], False)
            elif command == "set_property" and "TARGET" in args and "PROPERTY" in args:
                first, prop = args.index("TARGET"), args.index("PROPERTY")
                names = [a for a in args[first + 1:prop]
                         if a not in ("APPEND", "APPEND_STRING", "DIRECTORY")]
                if prop + 1 < len(args):
                    name = args[prop + 1].upper()
                    if name in LINK_LIBRARY_PROPERTIES or name in LINK_OPTION_PROPERTIES:
                        for target in targets_of(names):
                            record(target, args[prop + 2:], name in LINK_LIBRARY_PROPERTIES)
            elif command == "set_target_properties" and "PROPERTIES" in args:
                split = args.index("PROPERTIES")
                pairs = args[split + 1:]
                for target in targets_of(args[:split]):
                    for key, value in zip(pairs[::2], pairs[1::2]):
                        key = key.upper()
                        if key in LINK_LIBRARY_PROPERTIES or key in LINK_OPTION_PROPERTIES:
                            record(target, [value], key in LINK_LIBRARY_PROPERTIES)
            elif command == "link_libraries":
                errors.append(
                    f"[graph] {where}: link_libraries() adds {args} to every target in the "
                    f"directory; use target_link_libraries with a declared dependency"
                )
            elif command == "add_link_options":
                for value in args:
                    for item in unwrap_genex(resolve(value)):
                        if forbidden_item(item):
                            errors.append(
                                f"[graph] {where}: add_link_options() adds forbidden item {item}")
    return graph, defined, options


def check_declared_graph(errors: list[str]) -> None:
    graph, defined, options = declared_graph(errors)
    for target, expected in EXPECTED_GRAPH.items():
        if target not in defined:
            errors.append(f"[graph] target {target} is not defined under src/")
            continue
        actual = graph[target] - THIRD_PARTY_DEPS.get(target, set())
        if actual != expected:
            errors.append(
                f"[graph] {target}: expected deps {sorted(expected)}, found {sorted(actual)}"
            )
        reached = {d for d in closure(graph, target) if is_first_party(d)}
        if reached != closure(EXPECTED_GRAPH, target):
            errors.append(
                f"[graph] {target}: reaches first-party {sorted(reached)}, expected the "
                f"transitive closure {sorted(closure(EXPECTED_GRAPH, target))}"
            )
    for target in GRAPHICS_FREE_TARGETS:
        reached = closure(graph, target)
        for dep in sorted(reached):
            if forbidden_item(dep):
                errors.append(f"[graph] {target} depends on forbidden library {dep}")
        for node in sorted(reached | {target}):
            for where, item in options.get(node, []):
                if forbidden_item(item):
                    errors.append(f"[graph] {where} gives {node} forbidden link item {item}"
                                  + ("" if node == target else f" (reached from {target})"))


# --------------------------------------------------------------------------
# Build tree checks (CMake File API + compile_commands.json)
# --------------------------------------------------------------------------


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def load_codemodel(build_dir: Path, errors: list[str], tag: str) -> dict | None:
    reply = build_dir / ".cmake" / "api" / "v1" / "reply"
    indices = sorted(reply.glob("index-*.json"))
    if not indices:
        errors.append(f"[{tag}] no CMake File API reply found; configure first")
        return None
    index = read_json(indices[-1])
    codemodel_file = next(
        (o["jsonFile"] for o in index.get("objects", []) if o.get("kind") == "codemodel"),
        None,
    )
    if codemodel_file is None:
        errors.append(f"[{tag}] File API reply has no codemodel object")
        return None
    codemodel = read_json(reply / codemodel_file)
    version = codemodel.get("version", {})
    found = (version.get("major", 0), version.get("minor", 0))
    if found < MIN_CODEMODEL:
        errors.append(
            f"[{tag}] codemodel {found[0]}.{found[1]} does not report link libraries; "
            f"codemodel >= {MIN_CODEMODEL[0]}.{MIN_CODEMODEL[1]} (CMake >= 4.1) is required"
        )
        return None
    codemodel["_reply"] = reply
    return codemodel


def entry_ids(items) -> list[str]:
    return [item["id"] for item in items or () if "id" in item]


def entry_fragments(items) -> list[str]:
    return [item["fragment"] for item in items or () if "fragment" in item]


def id_name(target_id: str) -> str:
    """Target ids look like 'name::@hash'; recover the name for unresolved ids."""
    return target_id.rsplit("::@", 1)[0]


def is_first_party(name: str) -> bool:
    return normalize(name).startswith("orrery")


def check_fp_model(command: str, where: str, errors: list[str]) -> None:
    """The last floating-point model option must be the deterministic one."""
    models = MSVC_FP_MODEL.findall(command)
    contracts = GNU_FP_CONTRACT.findall(command)
    if models:
        if models[-1] not in ("precise", "strict"):
            errors.append(f"{where}: last floating-point model is /fp:{models[-1]}")
    elif contracts:
        if contracts[-1] != "off":
            errors.append(f"{where}: last -ffp-contract is '{contracts[-1]}', expected 'off'")
    else:
        errors.append(f"{where}: no deterministic floating-point option "
                      f"(/fp:precise or -ffp-contract=off)")


class Evidence:
    """Physics sources and include directories collected from build trees."""

    def __init__(self) -> None:
        self.files: set[Path] = set()
        self.include_dirs: set[Path] = set()


def check_configuration(config: dict, reply: Path, source_root: Path, tag: str,
                        errors: list[str], evidence: Evidence) -> int:
    nodes: dict[str, dict] = {}
    for entry in config.get("targets", []) + config.get("abstractTargets", []):
        nodes[entry["id"]] = read_json(reply / entry["jsonFile"])
    by_name = {data["name"]: data for data in nodes.values()}

    def link_edges(data: dict) -> set[str]:
        return (
            set(entry_ids(data.get("linkLibraries")))
            | set(entry_ids(data.get("interfaceLinkLibraries")))
            | set(entry_ids(data.get("compileDependencies")))
            | set(entry_ids(data.get("interfaceCompileDependencies")))
        )

    def edges(data: dict) -> set[str]:
        return link_edges(data) | set(entry_ids(data.get("dependencies")))

    def name_of(target_id: str) -> str:
        return nodes[target_id]["name"] if target_id in nodes else id_name(target_id)

    def reach(start_id: str, follow) -> tuple[set[str], list[str]]:
        """Names reachable from start (excluding start unless on a cycle) and
        the raw fragments of the link entries met on the way."""
        seen: set[str] = set()
        fragments: list[str] = []
        stack = list(follow(nodes[start_id])) if start_id in nodes else []
        if start_id in nodes:
            fragments += entry_fragments(nodes[start_id].get("linkLibraries"))
            fragments += entry_fragments(nodes[start_id].get("interfaceLinkLibraries"))
        while stack:
            node_id = stack.pop()
            if node_id in seen:
                continue
            seen.add(node_id)
            node = nodes.get(node_id)
            if node is not None:
                fragments += entry_fragments(node.get("linkLibraries"))
                fragments += entry_fragments(node.get("interfaceLinkLibraries"))
                stack.extend(follow(node))
        return {name_of(i) for i in seen}, fragments

    first_party_files = {
        link_names(by_name[t]["nameOnDisk"])[0].lower(): t
        for t in EXPECTED_GRAPH if t in by_name and by_name[t].get("nameOnDisk")
    }

    # First-party direct link graph and transitive reachability.
    for target, expected in EXPECTED_GRAPH.items():
        data = by_name.get(target)
        if data is None:
            errors.append(f"[{tag}] target {target} missing from code model")
            continue
        direct = entry_ids(data.get("linkLibraries")) + entry_ids(
            data.get("interfaceLinkLibraries")
        )
        actual = {normalize(name_of(i)) for i in direct if is_first_party(name_of(i))}
        if actual != expected:
            errors.append(
                f"[{tag}] {target}: expected link deps {sorted(expected)}, found {sorted(actual)}"
            )
        reached, fragments = reach(data["id"], link_edges)
        reached = {normalize(n) for n in reached if is_first_party(n)}
        for fragment in fragments:
            for token in re.split(r"[\s;]+", fragment):
                stem = link_names(token)[0].lower() if token else ""
                if stem in first_party_files:
                    reached.add(first_party_files[stem])
        allowed = closure(EXPECTED_GRAPH, target)
        if reached != allowed:
            errors.append(
                f"[{tag}] {target}: reaches first-party {sorted(reached)}, expected the "
                f"transitive closure {sorted(allowed)}"
            )

    # Graphics-free closure, including imported/interface targets and raw items.
    for target in GRAPHICS_FREE_TARGETS:
        data = by_name.get(target)
        if data is None:
            continue
        seen: set[str] = set()
        stack = [data["id"]]
        while stack:
            node_id = stack.pop()
            if node_id in seen:
                continue
            seen.add(node_id)
            node = nodes.get(node_id)
            name = name_of(node_id)
            via = "" if node_id == data["id"] else f" (via {name})"
            if node_id != data["id"] and forbidden_item(name):
                errors.append(f"[{tag}] {target} reaches forbidden target {name}")
            if node is None:
                continue
            if (node.get("imported") and node.get("type") != "INTERFACE_LIBRARY"
                    and not node.get("artifacts") and name not in UNVERIFIABLE_IMPORTED_OK):
                errors.append(
                    f"[{tag}] {target}{via}: imported {node.get('type', 'target')} {name} "
                    f"has no artifact in the code model; its library file cannot be verified"
                )
            items = [node.get("nameOnDisk", "")]
            items += [a.get("path", "") for a in node.get("artifacts", [])]
            items += entry_fragments(node.get("linkLibraries"))
            items += entry_fragments(node.get("interfaceLinkLibraries"))
            items += [f.get("fragment", "") for f in node.get("link", {}).get("commandFragments", [])]
            if node_id == data["id"]:
                items = items[1:]  # the target's own file name is not a dependency
                items = [i for i in items if Path(i).name != data.get("nameOnDisk")]
            for item in items:
                bad = item and forbidden_item(item)
                if bad:
                    errors.append(f"[{tag}] {target}{via} links forbidden item {bad}")
            stack.extend(edges(node))

    # Final link lines of consumers that link only graphics-free first-party
    # libraries. INTERFACE_LINK_OPTIONS, imported locations and other usage
    # requirements of the graphics-free closure appear only there.
    graphics_free = set(GRAPHICS_FREE_TARGETS)
    for target in GRAPHICS_FREE_TARGETS:
        graphics_free |= closure(EXPECTED_GRAPH, target)
    covered: set[str] = set()
    for data in nodes.values():
        if "link" not in data or data.get("abstract"):
            continue
        reached, _ = reach(data["id"], edges)
        first_party = {normalize(n) for n in reached if is_first_party(n)}
        uses = first_party & set(GRAPHICS_FREE_TARGETS)
        if not uses or not first_party <= graphics_free:
            continue
        covered |= uses
        for fragment in data["link"].get("commandFragments", []):
            bad = forbidden_item(fragment.get("fragment", ""))
            if bad:
                errors.append(
                    f"[{tag}] {data['name']} (links only {sorted(uses)}) has forbidden item "
                    f"{bad} on its link line"
                )
    for target in GRAPHICS_FREE_TARGETS:
        if target in by_name and target not in covered:
            errors.append(
                f"[{tag}] no executable links {target} together with only graphics-free "
                f"first-party libraries, so the interface link options and imported "
                f"locations it propagates cannot be verified (ORRERY_BUILD_TESTS=ON provides "
                f"orrery_test_* executables for this)"
            )

    # Fast-math in the per-configuration compile and link commands.
    for data in nodes.values():
        groups = [
            " ".join(frag.get("fragment", "") for frag in group.get("compileCommandFragments", []))
            for group in data.get("compileGroups", [])
        ]
        fragments = groups + [
            f.get("fragment", "") for f in data.get("link", {}).get("commandFragments", [])
        ]
        for fragment in fragments:
            for match in FAST_MATH_FLAG.finditer(fragment):
                errors.append(
                    f"[{tag}] {data['name']}: forbidden fast-math flag '{match.group(1)}'"
                )
        if is_first_party(data["name"]) and not data.get("imported"):
            for index, group in enumerate(groups):
                check_fp_model(group, f"[{tag}] {data['name']} compile group {index}", errors)

    # Physics sources and include directories for the precision check.
    for target in PHYSICS_TARGETS:
        data = by_name.get(target)
        if data is None:
            continue
        for source in data.get("sources", []):
            path = Path(source.get("path", ""))
            evidence.files.add(path if path.is_absolute() else source_root / path)
        for group in data.get("compileGroups", []):
            for include in group.get("includes", []):
                path = Path(include.get("path", ""))
                if "vcpkg_installed" not in path.parts:
                    evidence.include_dirs.add(path)
    return len(nodes)


def generator_of(build_dir: Path) -> str:
    cache = build_dir / "CMakeCache.txt"
    if cache.is_file():
        for line in cache.read_text(encoding="utf-8", errors="replace").splitlines():
            if line.startswith("CMAKE_GENERATOR:"):
                return line.split("=", 1)[1]
    return ""


def check_compile_commands(build_dir: Path, errors: list[str]) -> str:
    database = build_dir / "compile_commands.json"
    generator = generator_of(build_dir)
    if not database.is_file():
        if generator == "Ninja" or generator.endswith("Makefiles"):
            errors.append(f"[{build_dir.name}] compile_commands.json missing ({generator})")
            return "missing"
        return f"not produced by {generator or 'unknown generator'}"
    entries = read_json(database)
    for entry in entries:
        command = entry.get("command") or " ".join(entry.get("arguments", []))
        source = entry.get("file", "")
        for match in FAST_MATH_FLAG.finditer(command):
            errors.append(
                f"[{build_dir.name}] compile_commands.json: {source}: "
                f"forbidden fast-math flag '{match.group(1)}'"
            )
        path = Path(source)
        if not path.is_absolute():
            path = Path(entry.get("directory", "")) / path
        first_party = "vcpkg_installed" not in path.parts and (
            ROOT in path.resolve().parents)
        if first_party:
            check_fp_model(command, f"[{build_dir.name}] compile_commands.json: {source}", errors)
    return f"{len(entries)} commands"


def check_build_tree(build_dir: Path, errors: list[str], evidence: Evidence) -> None:
    before = len(errors)
    codemodel = load_codemodel(build_dir, errors, build_dir.name)
    if codemodel is None:
        return
    configs = codemodel.get("configurations", [])
    if not configs:
        errors.append(f"[{build_dir.name}] code model lists no configurations")
        return
    source_root = Path(codemodel.get("paths", {}).get("source", ROOT))
    for config in configs:
        name = config.get("name") or "<default>"
        tag = f"{build_dir.name}:{name}"
        count = check_configuration(config, codemodel["_reply"], source_root, tag, errors,
                                    evidence)
        print(f"link graph ({tag}): {count} targets, codemodel "
              f"{codemodel['version']['major']}.{codemodel['version']['minor']}")
    status = check_compile_commands(build_dir, errors)
    print(f"compile commands ({build_dir.name}): {status}")
    if len(errors) == before:
        print(f"build tree ({build_dir.name}): checked")


# --------------------------------------------------------------------------
# Precision check
# --------------------------------------------------------------------------

RAW_STRING = re.compile(r'(?:u8|[uUL])?R"([^()\\\s]{0,16})\(')


def splice_lines(text: str, lenient: bool) -> tuple[str, list[int]]:
    """Join backslash-newline splices (translation phase 2).

    Returns the spliced text and, for each resulting line, the physical line
    number where it starts. With `lenient`, whitespace between the backslash
    and the newline is accepted as well, as GCC and Clang do.
    """
    lines: list[str] = []
    starts: list[int] = []
    buffer, start = "", None
    for number, line in enumerate(text.split("\n"), 1):
        if line.endswith("\r"):
            line = line[:-1]
        if start is None:
            start = number
        body = line.rstrip(" \t") if lenient else line
        if body.endswith("\\"):
            buffer += body[:-1]
            continue
        lines.append(buffer + line)
        starts.append(start)
        buffer, start = "", None
    if start is not None:
        lines.append(buffer)
        starts.append(start)
    return "\n".join(lines), starts


def strip_comments_and_strings(text: str, keep_strings: bool = False) -> str:
    """Blank out comments and (unless keep_strings) string and character
    literals; keep newlines."""
    out: list[str] = []
    i, n = 0, len(text)

    def blank(segment: str) -> str:
        return "".join("\n" if c == "\n" else " " for c in segment)

    while i < n:
        c = text[i]
        if text.startswith("//", i):
            end = text.find("\n", i)
            end = n if end == -1 else end
            # A backslash-newline continues a line comment.
            while end < n and text[end - 1] == "\\":
                nxt = text.find("\n", end + 1)
                end = n if nxt == -1 else nxt
            out.append(blank(text[i:end]))
            i = end
            continue
        if text.startswith("/*", i):
            end = text.find("*/", i + 2)
            end = n if end == -1 else end + 2
            out.append(blank(text[i:end]))
            i = end
            continue
        raw = RAW_STRING.match(text, i)
        if raw and (i == 0 or not (text[i - 1].isalnum() or text[i - 1] == "_")):
            close = ")" + raw.group(1) + '"'
            end = text.find(close, raw.end())
            end = n if end == -1 else end + len(close)
            out.append(text[i:end] if keep_strings else blank(text[i:end]))
            i = end
            continue
        if c == '"' or (c == "'" and not (i > 0 and text[i - 1].isalnum() and _in_number(text, i))):
            end = i + 1
            while end < n and text[end] != c and text[end] != "\n":
                end += 2 if text[end] == "\\" else 1
            end = min(end + 1, n)
            out.append(text[i:end] if keep_strings else blank(text[i:end]))
            i = end
            continue
        out.append(c)
        i += 1
    return "".join(out)


def _in_number(text: str, quote: int) -> bool:
    """True if the quote at `quote` is a digit separator (1'000)."""
    j = quote - 1
    while j >= 0 and (text[j].isalnum() or text[j] in "'._"):
        j -= 1
    token = text[j + 1:quote]
    return bool(token) and (token[0].isdigit() or (token[0] == "." and token[1:2].isdigit()))


def precision_hits(line: str) -> list[tuple[str, str]]:
    """(kind, token) pairs for one comment- and string-free logical line."""
    hits = []
    for kind, pattern in (("float type", FLOAT_TYPE), ("float literal", FLOAT_LITERAL),
                          ("float function", FLOAT_FUNCTION),
                          ("float constant", FLOAT_CONSTANT)):
        hits += [(kind, m.group(0)) for m in pattern.finditer(line)]
    if TOKEN_PASTE.search(line):
        hits.append(("token pasting (result cannot be checked)", "##"))
    for call in CALL_ARGS.finditer(line):
        args = [a.strip() for a in call.group(1).split(",")]
        for first in range(len(args)):
            for last in range(first + 2, len(args) + 1):
                joined = "".join(args[first:last])
                if FLOAT_TYPE.fullmatch(joined):
                    hits.append(("float type assembled from macro arguments", joined))
    return hits


def is_source_candidate(path: Path) -> bool:
    return path.is_file() and path.suffix.lower() not in NON_SOURCE_SUFFIXES


def check_float_usage(errors: list[str], evidence: Evidence) -> int:
    include_dirs = {d for d in evidence.include_dirs if d.is_dir()}
    include_dirs |= {p for p in (ROOT / "src").glob("*/include") if p.is_dir()}

    queue: list[Path] = []
    for subdir in PRECISION_CHECKED_DIRS:
        base = ROOT / "src" / subdir
        if base.is_dir():
            queue += sorted(p for p in base.rglob("*") if is_source_candidate(p))
    queue += sorted(p for p in evidence.files if p.is_file())
    for directory in sorted(evidence.include_dirs):
        if directory.is_dir():
            queue += sorted(p for p in directory.rglob("*") if is_source_candidate(p))
    for path in evidence.files:
        if not path.is_file():
            errors.append(f"[precision] {rel_display(path)}: physics target source not found")

    seen: set[Path] = set()
    scanned = 0
    while queue:
        path = queue.pop(0).resolve()
        if path in seen:
            continue
        seen.add(path)
        rel = rel_display(path)
        scanned += 1
        text = read_text_checked(path, errors, "precision")
        if text is None:
            continue
        physical = text.replace("\r\n", "\n").split("\n")
        reported: set[tuple[int, str]] = set()
        for lenient in (False, True):
            spliced, starts = splice_lines(text, lenient)
            code = strip_comments_and_strings(spliced).split("\n")
            directives = strip_comments_and_strings(spliced, keep_strings=True)
            for index, line in enumerate(directives.split("\n")):
                match = FAST_MATH_SOURCE.search(line)
                key = (starts[index], "fp")
                if match and key not in reported:
                    reported.add(key)
                    errors.append(
                        f"[fast-math] {rel}:{starts[index]}: value-changing floating-point "
                        f"pragma or attribute in: {line.strip()}"
                    )
            if rel in FLOAT_ALLOWLIST:
                continue
            for index, line in enumerate(code):
                for kind, token in precision_hits(line):
                    key = (starts[index], token)
                    if key in reported:
                        continue
                    reported.add(key)
                    errors.append(
                        f"[precision] {rel}:{starts[index]}: {kind} '{token}' in: "
                        f"{physical[starts[index] - 1].strip()}"
                    )
            # Follow includes that resolve to files.
            for kind, name in INCLUDE_DIRECTIVE.findall(directives):
                candidates = [path.parent / name] if kind == '"' else []
                candidates += [d / name for d in sorted(include_dirs)]
                for candidate in candidates:
                    if candidate.is_file() and "vcpkg_installed" not in candidate.parts:
                        queue.append(candidate)
                        break
    return scanned


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--build-dir",
        action="append",
        type=Path,
        help="configured CMake build tree to inspect (repeatable); "
        "defaults to every tree under build/",
    )
    parser.add_argument(
        "--source-only",
        action="store_true",
        help="only run the source checks (no configured build tree required); "
        "advisory, the configured check is authoritative",
    )
    args = parser.parse_args()

    errors: list[str] = []
    evidence = Evidence()

    check_declared_graph(errors)
    print(f"declared graph: {len(EXPECTED_GRAPH)} targets checked")

    if args.source_only:
        print("note: --source-only is advisory; it parses CMake text without evaluating it. "
              "Run with --build-dir on a configured tree for the authoritative check.")
    else:
        build_dirs = args.build_dir
        if not build_dirs:
            build_root = ROOT / "build"
            build_dirs = (
                sorted(p for p in build_root.iterdir() if (p / "CMakeCache.txt").is_file())
                if build_root.is_dir()
                else []
            )
        if not build_dirs:
            errors.append("[build] no configured build tree found; configure first "
                          "or pass --source-only")
        for build_dir in build_dirs:
            check_build_tree(build_dir.resolve(), errors, evidence)

    scanned = check_float_usage(errors, evidence)
    print(f"precision: {scanned} files scanned (src/{{{','.join(PRECISION_CHECKED_DIRS)}}}, "
          f"physics target sources and included headers)")

    if errors:
        errors = list(dict.fromkeys(errors))
        for error in errors:
            print(f"ERROR {error}", file=sys.stderr)
        print(f"FAILED ({len(errors)} violation(s))")
        return 1
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
