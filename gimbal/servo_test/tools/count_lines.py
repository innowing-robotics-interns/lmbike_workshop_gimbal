r"""Count code lines, excluding comments and blanks, for C sources.

Why a script instead of a one-liner: this project's main.c is written with a
very high comment ratio, and the whole point of the number is to separate the
prose from the code. A naive grep for lines starting with // gets block
comments, trailing comments and the comment-looking text inside string
literals all wrong - and this project has strings with /* and // in them (the
diagnostics print them). So the scanner tracks state properly.

Counts per line: a line counts if it has any non-whitespace left after comments
are removed. A line that is code followed by a trailing comment counts once.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)   # servo_test/ (parent of tools/)


def scan(text):
    """Return (raw_lines, code_lines) for one C source."""
    raw = text.splitlines()

    out = []          # the same lines with comments removed
    i = 0
    in_block = False

    while i < len(raw):
        line = raw[i]
        j = 0
        piece = []
        in_str = False
        in_chr = False
        esc = False

        while j < len(line):
            c = line[j]

            if in_block:
                if c == "*" and j + 1 < len(line) and line[j + 1] == "/":
                    in_block = False
                    j += 2
                    continue
                j += 1
                continue

            if esc:
                piece.append(c)
                esc = False
                j += 1
                continue

            if in_str or in_chr:
                piece.append(c)
                if c == "\\":
                    esc = True
                elif in_str and c == '"':
                    in_str = False
                elif in_chr and c == "'":
                    in_chr = False
                j += 1
                continue

            # not in a comment or a literal
            if c == "/" and j + 1 < len(line) and line[j + 1] == "*":
                in_block = True
                j += 2
                continue
            if c == "/" and j + 1 < len(line) and line[j + 1] == "/":
                break                      # rest of line is a comment
            if c == '"':
                in_str = True
            elif c == "'":
                in_chr = True
            piece.append(c)
            j += 1

        out.append("".join(piece))
        i += 1

    code = sum(1 for ln in out if ln.strip())
    return len(raw), code


def walk(root, exts):
    for dirpath, dirnames, filenames in os.walk(root):
        for fn in filenames:
            if os.path.splitext(fn)[1] in exts:
                yield os.path.join(dirpath, fn)


def report(title, files):
    raw = code = 0
    per_file = []
    for path in files:
        with open(path, "r", encoding="utf-8", errors="replace") as fh:
            r, c = scan(fh.read())
        raw += r
        code += c
        per_file.append((c, r, os.path.relpath(path, ROOT)))
    print("== %s ==" % title)
    if len(per_file) <= 12:
        for c, r, name in sorted(per_file, reverse=True):
            pct = 100.0 * (r - c) / r if r else 0
            print("  %-40s %6d code / %6d raw  (%4.1f%% comment+blank)" % (name, c, r, pct))
    print("  %-40s %6d code / %6d raw  (%4.1f%% comment+blank)"
          % ("TOTAL (%d files)" % len(per_file), code, raw,
             100.0 * (raw - code) / raw if raw else 0))
    print()
    return code, raw


def main():
    core = list(walk(os.path.join(ROOT, "Core"), {".c", ".h"}))
    drivers = list(walk(os.path.join(ROOT, "Drivers"), {".c", ".h"}))

    main_c = os.path.join(ROOT, "Core", "Src", "main.c")
    with open(main_c, "r", encoding="utf-8", errors="replace") as fh:
        r, c = scan(fh.read())
    print("== main.c (the only file being edited) ==")
    print("  %-40s %6d code / %6d raw  (%4.1f%% comment+blank)"
          % ("Core/Src/main.c", c, r, 100.0 * (r - c) / r))
    print()

    c1, r1 = report("Core (the project's own code)", core)
    c2, r2 = report("Drivers (ST HAL + CMSIS, not edited)", drivers)
    print("== EVERYTHING ==")
    print("  %-40s %6d code / %6d raw  (%4.1f%% comment+blank)"
          % ("TOTAL (%d files)" % (len(core) + len(drivers)), c1 + c2, r1 + r2,
             100.0 * ((r1 + r2) - (c1 + c2)) / (r1 + r2)))


if __name__ == "__main__":
    sys.exit(main())
