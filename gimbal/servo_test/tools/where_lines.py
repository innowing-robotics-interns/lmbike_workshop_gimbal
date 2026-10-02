r"""Where do main.c's code lines actually live - which function, or file scope?

count_lines.py answers "how many". This answers "of what", which is the
question that matters when the number is 1,220 and most of the file is prose.

The detection is deliberately shaped to THIS file's style rather than to C in
general: every function here is written with the closing paren of the
signature on its own line at column 0, and the opening brace on the very next
line, also at column 0. That is the ST/gnu11 house style the whole project
uses. A general C parser would be more code and no more correct here.

Anything at column 0 that is not a function body - a #define, a typedef, a
global, the enum - is counted as file scope, which is the residual bucket.
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from count_lines import scan            # reuse the comment stripper

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)   # servo_test/ (parent of tools/)
MAIN_C = os.path.join(ROOT, "Core", "Src", "main.c")

# A function definition, in this file's style:
#     static uint8_t pca_write_pulse(uint8_t ch, uint16_t counts)
#     {
# i.e. a line at column 0 that looks like a signature, followed by a lone {.
NAME_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_ \*]*?\b([A-Za-z_][A-Za-z0-9_]*)\s*\(")


def code_lines(lines):
    """Indices of lines that still have non-whitespace after comments go."""
    out = []
    in_block = False
    for i, line in enumerate(lines):
        j = 0
        piece = []
        in_str = in_chr = esc = False
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
            if c == "/" and j + 1 < len(line) and line[j + 1] == "*":
                in_block = True
                j += 2
                continue
            if c == "/" and j + 1 < len(line) and line[j + 1] == "/":
                break
            if c == '"':
                in_str = True
            elif c == "'":
                in_chr = True
            piece.append(c)
            j += 1
        if "".join(piece).strip():
            out.append(i)
    return out


def main():
    with open(MAIN_C, "r", encoding="utf-8", errors="replace") as fh:
        raw = fh.read().splitlines()

    stripped = []
    in_block = False
    for line in raw:
        j = 0
        piece = []
        in_str = in_chr = esc = False
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
                piece.append(c); esc = False; j += 1; continue
            if in_str or in_chr:
                piece.append(c)
                if c == "\\": esc = True
                elif in_str and c == '"': in_str = False
                elif in_chr and c == "'": in_chr = False
                j += 1
                continue
            if c == "/" and j + 1 < len(line) and line[j + 1] == "*":
                in_block = True; j += 2; continue
            if c == "/" and j + 1 < len(line) and line[j + 1] == "/":
                break
            if c == '"': in_str = True
            elif c == "'": in_chr = True
            piece.append(c); j += 1
        stripped.append("".join(piece))

    code_idx = set(code_lines(raw))

    # Find function bodies: a signature line at column 0 whose stripped text
    # ends in ')' and whose next non-empty raw line is a lone '{'.
    starts = []
    for i, s in enumerate(stripped):
        if not s or s[0] in " \t{}/#":
            continue
        if not s.rstrip().endswith(")"):
            continue
        k = i + 1
        while k < len(raw) and not stripped[k].strip():
            k += 1
        if k < len(raw) and stripped[k].strip() == "{":
            m = NAME_RE.match(s)
            if m:
                starts.append((i, k, m.group(1)))

    # A body runs from its opening brace to the matching column-0 '}'.
    spans = []
    for n, (sig, brace, name) in enumerate(starts):
        end = len(raw) - 1
        for k in range(brace + 1, len(raw)):
            if stripped[k].rstrip() == "}":
                end = k
                break
        spans.append((name, sig, brace, end))

    counted = set()
    rows = []
    for name, sig, brace, end in spans:
        n = sum(1 for i in range(brace, end + 1) if i in code_idx)
        counted |= {i for i in range(brace, end + 1)}
        rows.append((n, name))

    file_scope = sum(1 for i in code_idx if i not in counted)

    # Computed, not typed. This said "1,220" for months after main.c was cut
    # down to a third of that, which is the exact failure this project keeps
    # running into: a plausible printed number nobody re-checks.
    print("main.c - %d code lines, where they are" % (file_scope + sum(n for n, _ in rows)))
    print("=" * 62)
    print("  %-42s %5d" % ("[file scope: macros, typedefs, globals]", file_scope))
    print()
    for n, name in sorted(rows, reverse=True):
        print("  %-42s %5d" % (name + "()", n))
    print("-" * 62)
    print("  %-42s %5d" % ("TOTAL", file_scope + sum(n for n, _ in rows)))
    print()
    print("  %d functions, %d of the code lines are inside a function body,"
          % (len(rows), sum(n for n, _ in rows)))
    print("  %d are file-scope declarations." % file_scope)
    return 0


if __name__ == "__main__":
    sys.exit(main())
