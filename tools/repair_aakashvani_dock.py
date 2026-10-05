#!/usr/bin/env python3
"""Repair common Markdown/indentation corruption in the Aakashvani Dock source.

Usage:
    python repair_aakashvani_dock.py damaged.py

By default a repaired copy is written next to the input as *_repaired.py.
Use --in-place to replace the input file after creating a .bak backup.
"""

from __future__ import annotations

import argparse
import ast
import py_compile
import re
import shutil
import sys
from pathlib import Path


CLASS_MARKER = "class AakashvaniDock"
ENTRY_MARKER = 'if __name__ == "__main__":'


def indent_line(line: str, spaces: int = 4) -> str:
    """Add indentation while preserving blank lines."""
    if not line.strip():
        return line
    return " " * spaces + line


def remove_markdown_corruption(lines: list[str]) -> tuple[list[str], int]:
    """Remove common Markdown transformations from Python source."""
    out: list[str] = []
    changes = 0

    # These are specifically the transformations visible in the damaged file.
    replacements = {
        "def **init**(self):": "def __init__(self):",
        "super().**init**()": "super().__init__()",
        'if **name** == "**main**":': ENTRY_MARKER,
    }

    for line in lines:
        raw = line
        stripped = line.strip()

        # Markdown fenced-code markers are not Python.
        if stripped == "```":
            changes += 1
            continue

        for old, new in replacements.items():
            if old in line:
                line = line.replace(old, new)

        # Escaped Markdown punctuation leaked into the Python source.
        # This is safe for the damaged file and fixes constructs such as \[ ... \].
        line = line.replace(r"\#", "#")
        line = line.replace(r"\[", "[")
        line = line.replace(r"\]", "]")
        line = line.replace(r"\.", ".")

        # A few Markdown renderers may turn a dunder name into bold text.
        line = line.replace("**init**", "__init__")
        line = line.replace("**name**", "__name__")
        line = line.replace("**main**", "__main__")

        if line != raw:
            changes += 1
        out.append(line)

    return out, changes


def indent_top_level_try_blocks(lines: list[str], end: int) -> int:
    """Repair top-level try/except blocks before the Aakashvani class.

    The damaged file has two import try/except blocks whose suite indentation
    was stripped. Existing correctly-indented blocks are left unchanged.
    """
    changes = 0
    i = 0
    while i < end:
        if lines[i].strip() == "try:" and not lines[i].startswith(" "):
            # Find a matching top-level except.
            j = i + 1
            while j < end:
                if lines[j].strip().startswith("except ") and not lines[j].startswith(" "):
                    break
                j += 1
            if j >= end:
                i += 1
                continue

            # Indent the try-suite whenever its nonblank statements are at col 0.
            for k in range(i + 1, j):
                if lines[k].strip() and len(lines[k]) - len(lines[k].lstrip(" ")) == 0:
                    lines[k] = indent_line(lines[k])
                    changes += 1

            # The damaged import blocks end at the first blank line after the
            # except suite. Restore indentation for that suite.
            k = j + 1
            while k < end and lines[k].strip():
                current = len(lines[k]) - len(lines[k].lstrip(" "))
                if current == 0:
                    lines[k] = indent_line(lines[k])
                    changes += 1
                k += 1
            i = k
        else:
            i += 1
    return changes


def repair_function_body(lines: list[str], def_index: int, body_end: int) -> int:
    """Indent every nonblank line in a known top-level function body by 4."""
    changes = 0
    for i in range(def_index + 1, body_end):
        if lines[i].strip():
            lines[i] = indent_line(lines[i])
            changes += 1
    return changes


def repair_class_block(lines: list[str], class_index: int, entry_index: int) -> int:
    """Indent the complete class body by one level.

    The source's inner indentation is mostly intact; the class-level indentation
    was stripped, so adding four spaces to every nonblank line restores it.
    """
    changes = 0
    for i in range(class_index + 1, entry_index):
        if lines[i].strip():
            lines[i] = indent_line(lines[i])
            changes += 1
    return changes


def repair_entry_point(lines: list[str], entry_index: int) -> int:
    """Indent the entry-point body."""
    changes = 0
    for i in range(entry_index + 1, len(lines)):
        if lines[i].strip():
            current = len(lines[i]) - len(lines[i].lstrip(" "))
            if current == 0:
                lines[i] = indent_line(lines[i])
                changes += 1
    return changes


def validate_python(path: Path) -> tuple[bool, str]:
    """Validate syntax with both AST parsing and bytecode compilation."""
    try:
        source = path.read_text(encoding="utf-8")
        ast.parse(source, filename=str(path))
        py_compile.compile(str(path), doraise=True)
        return True, "Syntax check passed (AST parse + py_compile)."
    except (SyntaxError, IndentationError) as exc:
        msg = f"{exc.__class__.__name__}: {exc}"
        return False, msg
    except Exception as exc:
        # Import/dependency issues should not be confused with syntax problems.
        return False, f"Validation tool error: {exc}"


def repair_file(input_path: Path, output_path: Path) -> tuple[bool, str, int]:
    lines = input_path.read_text(encoding="utf-8").splitlines(keepends=True)
    changes = 0

    lines, n = remove_markdown_corruption(lines)
    changes += n

    # Locate structural anchors after Markdown cleanup.
    class_index = next((i for i, line in enumerate(lines) if line.lstrip().startswith(CLASS_MARKER)), None)
    if class_index is None:
        raise RuntimeError(f"Could not find '{CLASS_MARKER}'.")

    entry_index = next((i for i, line in enumerate(lines) if line.strip() == ENTRY_MARKER), None)
    if entry_index is None:
        raise RuntimeError("Could not find the __main__ entry point after cleanup.")

    # Repair import try/except blocks before the class.
    changes += indent_top_level_try_blocks(lines, class_index)

    # Find the two known module-level helper functions before the class.
    helper1 = next((i for i in range(class_index) if lines[i].startswith("def wrap_diff_180")), None)
    helper2 = next((i for i in range(class_index) if lines[i].startswith("def process_imu_attitude")), None)
    if helper1 is None or helper2 is None:
        raise RuntimeError("Could not find expected helper functions before the class.")

    changes += repair_function_body(lines, helper1, helper2)
    changes += repair_function_body(lines, helper2, class_index)

    # In process_imu_attitude(), the first orientation if/else block was
    # flattened more aggressively than the rest of the function. Restore its
    # nested suite after the normal function-level indentation pass.
    orientation_if = next(
        (i for i in range(helper2 + 1, class_index)
         if lines[i].strip() == 'if orientation_mode == "perpendicular":'),
        None,
    )
    if orientation_if is not None:
        lines[orientation_if] = "    " + lines[orientation_if].lstrip()
        nested = orientation_if + 1
        while nested < class_index and lines[nested].strip():
            if lines[nested].strip() == "else:":
                lines[nested] = "    " + lines[nested].lstrip()
                nested += 1
                while nested < class_index and lines[nested].strip():
                    lines[nested] = "        " + lines[nested].lstrip()
                    nested += 1
                break
            lines[nested] = "        " + lines[nested].lstrip()
            nested += 1

    # Restore class indentation and entry-point indentation.
    changes += repair_class_block(lines, class_index, entry_index)

    # The super().__init__() line was completely unindented in the damaged
    # source, unlike the remainder of __init__. It therefore needs one extra
    # level after the class-wide indentation repair.
    init_index = next((i for i in range(class_index, entry_index)
                       if lines[i].strip() == "def __init__(self):"), None)
    if init_index is not None:
        k = init_index + 1
        while k < entry_index and not lines[k].strip():
            k += 1
        if k < entry_index and lines[k].strip() == "super().__init__()":
            lines[k] = "        " + lines[k].lstrip()
            changes += 1

    changes += repair_entry_point(lines, entry_index)

    # Normalize trailing whitespace only; do not otherwise reformat code.
    normalized = [re.sub(r"[ \t]+(?=\r?\n$)", "", line) for line in lines]
    changes += sum(a != b for a, b in zip(lines, normalized))
    lines = normalized

    output_path.write_text("".join(lines), encoding="utf-8", newline="")
    ok, message = validate_python(output_path)
    return ok, message, changes


def main() -> int:
    parser = argparse.ArgumentParser(description="Repair the damaged Aakashvani Dock Python source.")
    parser.add_argument("source", nargs="?", help="Path to the damaged Python/text source file.")
    parser.add_argument("-o", "--output", help="Output .py path (default: *_repaired.py)")
    parser.add_argument("--in-place", action="store_true", help="Replace the source after creating source.bak")
    args = parser.parse_args()

    source = Path(args.source).expanduser() if args.source else None
    if source is None:
        candidates = [p for p in Path.cwd().glob("*.py") if p.name != Path(__file__).name]
        if len(candidates) == 1:
            source = candidates[0]
        else:
            print("Usage: python repair_aakashvani_dock.py <damaged_file.py>")
            if candidates:
                print("\nPython files found:")
                for p in candidates:
                    print(f"  {p}")
            return 2

    if not source.exists():
        print(f"ERROR: source file not found: {source}", file=sys.stderr)
        return 2

    if args.in_place:
        output = source
        backup = source.with_suffix(source.suffix + ".bak")
        if backup.exists():
            backup = source.with_suffix(source.suffix + ".bak1")
        shutil.copy2(source, backup)
        temp_output = source.with_suffix(source.suffix + ".repaired.tmp")
    else:
        if args.output:
            output = Path(args.output).expanduser()
        else:
            output = source.with_name(source.stem + "_repaired.py")
        temp_output = output
        backup = None

    try:
        ok, message, changes = repair_file(source, temp_output)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        if args.in_place:
            temp_output.unlink(missing_ok=True)
        return 1

    if args.in_place:
        if ok:
            temp_output.replace(output)
        else:
            temp_output.unlink(missing_ok=True)
            print("Repair was not written in-place because validation failed.")
            print(message)
            return 1

    print(f"Input : {source}")
    print(f"Output: {output}")
    if backup:
        print(f"Backup: {backup}")
    print(f"Changes applied: {changes}")
    print(message)

    if not ok:
        print("\nThe repair script fixed the known corruption, but additional source-specific syntax errors remain.")
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
