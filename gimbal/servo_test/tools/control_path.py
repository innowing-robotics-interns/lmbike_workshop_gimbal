r"""How many code lines does it actually take to make the IMU drive the servo?

count_lines.py says main.c is 1,220 code lines. Most of that is diagnostics
left over from solved problems. This traces the intra-file call graph and
reports only what the control path needs.

Method: parse main.c into functions (same column-0 style assumption as
where_lines.py), scan each body for calls to other functions defined in the
file, then take the transitive closure of a set of roots.

WHY THE ROOTS ARE THE SUB-SYSTEMS AND NOT imu_servo_loop. Rooting at the loop
gives the wrong answer, and got it the first time this was run: the loop calls
pca_readback(), pca_burst_probe(), i2c_scan_bus(), imu_show() and
imu_link_code() once a second, so all five land inside the "runtime" set and
add ~150 lines of pure diagnostic to a number that is supposed to measure the
control path. Rooting at imu_poll_rx / imu_axis_us / servo_us instead asks the
question that was actually meant - "what does it take to read an angle and put
out a pulse" - and the diagnostics are then correctly outside it, because no
part of the real path calls them.

The loop body itself is handled separately below, by classifying its lines.

Reachability is a source-level, whole-file analysis - it does NOT evaluate #if,
so a call inside a disabled block still counts as an edge. That is the
conservative direction: it can only over-count the control path.
"""

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)   # servo_test/ (parent of tools/)
MAIN_C = os.path.join(ROOT, "Core", "Src", "main.c")

NAME_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_ \*]*?\b([A-Za-z_][A-Za-z0-9_]*)\s*\(")
CALL_RE = re.compile(r"\b([A-Za-z_][A-Za-z0-9_]*)\s*\(")

# The three entry points the control path actually has. Each is the head of a
# subsystem: read a frame off USART1, turn an angle into a pulse width, put a
# pulse on a header.
PATH_ROOTS = ["imu_poll_rx", "imu_axis_us", "servo_us"]

# One-time bring-up. Without these the loop above runs but does nothing: no
# UART means no frames, no bus init means no pulses.
INIT_ROOTS = ["imu_uart_init", "i2c_init", "pca_init"]

# Called from the loop but not part of the path. Listed rather than inferred so
# that the choice is visible and arguable.
DIAGNOSTIC = {
    "pca_readback",    # the once-a-second register readback
    "pca_burst_probe",  # the read-retry timing probe
    "i2c_scan_bus",    # bus address scan
    "imu_show",        # LED display
    "imu_build_show",  # LED display
    "imu_build_code",  # LED display
    "imu_link_code",   # link status for the LED
    "pca_reinit",      # repair for a detected chip reset
    "pca_blink_code",
}

NOT_CALLS = {"if", "while", "for", "switch", "return", "sizeof", "do", "else", "case"}


def strip_comments(lines):
    out = []
    in_block = False
    for line in lines:
        j = 0
        piece = []
        in_str = in_chr = esc = False
        while j < len(line):
            c = line[j]
            if in_block:
                if c == "*" and j + 1 < len(line) and line[j + 1] == "/":
                    in_block = False; j += 2; continue
                j += 1; continue
            if esc:
                piece.append(c); esc = False; j += 1; continue
            if in_str or in_chr:
                piece.append(c)
                if c == "\\": esc = True
                elif in_str and c == '"': in_str = False
                elif in_chr and c == "'": in_chr = False
                j += 1; continue
            if c == "/" and j + 1 < len(line) and line[j + 1] == "*":
                in_block = True; j += 2; continue
            if c == "/" and j + 1 < len(line) and line[j + 1] == "/":
                break
            if c == '"': in_str = True
            elif c == "'": in_chr = True
            piece.append(c); j += 1
        out.append("".join(piece))
    return out


def parse(path):
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        raw = fh.read().splitlines()
    stripped = strip_comments(raw)

    spans = []
    for i, s in enumerate(stripped):
        if not s or s[0] in " \t{}/#":
            continue
        if not s.rstrip().endswith(")"):
            continue
        k = i + 1
        while k < len(stripped) and not stripped[k].strip():
            k += 1
        if k < len(stripped) and stripped[k].strip() == "{":
            m = NAME_RE.match(s)
            if m:
                end = len(raw) - 1
                for j in range(k + 1, len(raw)):
                    if stripped[j].rstrip() == "}":
                        end = j
                        break
                spans.append((m.group(1), k, end))

    code_idx = {i for i, s in enumerate(stripped) if s.strip()}
    return raw, stripped, spans, code_idx


def main():
    raw, stripped, spans, code_idx = parse(MAIN_C)
    names = {n for n, _, _ in spans}
    lines_of = {n: sum(1 for i in range(b, e + 1) if i in code_idx) for n, b, e in spans}

    calls = {}
    for name, brace, end in spans:
        seen = set()
        for i in range(brace, end + 1):
            for m in CALL_RE.finditer(stripped[i]):
                c = m.group(1)
                if c in names and c != name and c not in NOT_CALLS:
                    seen.add(c)
        calls[name] = seen

    def reach(roots, keep_out=()):
        got, stack = set(), list(roots)
        blocked = set(keep_out)
        while stack:
            n = stack.pop()
            if n in got or n not in calls or n in blocked:
                continue
            got.add(n)
            stack.extend(calls[n])
        return got

    path = reach(PATH_ROOTS)
    init = reach(INIT_ROOTS) - path
    everything = reach(PATH_ROOTS + INIT_ROOTS)
    rest = names - everything - DIAGNOSTIC

    def show(title, s, note=""):
        print("=" * 68)
        print(title + ("   " + note if note else ""))
        print("=" * 68)
        for n in sorted(s, key=lambda x: -lines_of[x]):
            print("  %-36s %5d" % (n + "()", lines_of[n]))
        print("  %-36s %5d" % ("subtotal (%d functions)" % len(s), sum(lines_of[n] for n in s)))
        print()

    show("A. THE CONTROL PATH - angle in, pulse out", path)
    show("B. ONE-TIME BRING-UP - must have run first", init)

    # The loop body. Classifying a line by "does it call a diagnostic function"
    # was tried first and is wrong: the readback block is mostly a `for` loop
    # and an `if`, whose lines call nothing, so ~30 lines of diagnostic were
    # being counted as control. The blocks are identified by line number
    # instead, and the assertion below fails loudly if an edit shifts them.
    #
    # imu_servo_loop, as of 2026-09-29:
    #   body        3364..3630
    #   control     3364..3456   locals, uart init, the poll gate, the law,
    #                            the two servo_us() calls
    #   LED         3458..3460   imu_show()
    #   burst       3479..3503   now = HAL_GetTick(); the 10s burst probe
    #   readback    3505..3579   the 1s readback and the reset guard
    #   bus scan    3581..3627   the one-shot scan, PCA_LOOP_SCAN 0 so the
    #                            i2c_scan_bus() call itself is inside #if
    DIAG_RANGES = [(3458, 3460), (3479, 3503), (3505, 3579), (3581, 3627)]

    brace, end = next((b, e) for n, b, e in spans if n == "imu_servo_loop")
    # spans hold 0-based indices; DIAG_RANGES above are 1-based line numbers,
    # the same ones an editor shows. Convert once, here, and fail loudly if the
    # function has moved - the alternative is silently counting the wrong block.
    first, last = brace + 1, end + 1
    if first != 3364 or last != 3630:
        print("!! imu_servo_loop has moved (lines %d..%d, expected 3364..3630)."
              % (first, last))
        print("!! The DIAG_RANGES above are stale. Re-read the function and fix them.")
        return 1

    ctrl_lines, diag_lines = [], []
    for i in range(brace, end + 1):
        if i not in code_idx:
            continue
        in_diag = any(lo <= i + 1 <= hi for lo, hi in DIAG_RANGES)
        (diag_lines if in_diag else ctrl_lines).append(i)

    print("=" * 68)
    print("C. THE LOOP BODY, imu_servo_loop(), split by what each line calls")
    print("=" * 68)
    print("  %-36s %5d   the control law, clamps, the two servo_us() calls"
          % ("control lines", len(ctrl_lines)))
    print("  %-36s %5d   readback / burst probe / bus scan / LED"
          % ("diagnostic lines", len(diag_lines)))
    print("  %-36s %5d" % ("total", len(ctrl_lines) + len(diag_lines)))
    print()

    show("D. THE DIAGNOSTICS - reachable from the loop, not from the path", DIAGNOSTIC)

    print("=" * 68)
    print("E. NEITHER - other modes, boot, retired probes")
    print("=" * 68)
    for n in sorted(rest, key=lambda x: -lines_of[x]):
        print("  %-36s %5d" % (n + "()", lines_of[n]))
    print("  %-36s %5d" % ("subtotal (%d functions)" % len(rest), sum(lines_of[n] for n in rest)))
    print()

    # Bucket E must not contain imu_servo_loop - its lines are already in C.
    rest = rest - {"imu_servo_loop"}

    counted = set()
    for n, b, e in spans:
        counted |= set(range(b, e + 1))
    file_scope = sum(1 for i in code_idx if i not in counted)

    a = sum(lines_of[n] for n in path)
    b_ = sum(lines_of[n] for n in init)
    d = sum(lines_of[n] for n in DIAGNOSTIC)
    e = sum(lines_of[n] for n in rest)

    print("=" * 68)
    print("ANSWER")
    print("=" * 68)
    print("  A   control path functions                    %5d" % a)
    print("  C1  control lines inside the loop body        %5d" % len(ctrl_lines))
    print("  ------------------------------------------------------")
    print("  IMU -> servo, every pass                      %5d lines" % (a + len(ctrl_lines)))
    print()
    print("  B   + one-time bring-up                       %5d" % b_)
    print("  IMU -> servo, running at all                  %5d lines" % (a + len(ctrl_lines) + b_))
    print()
    print("  C2  diagnostic lines in the loop              %5d" % len(diag_lines))
    print("  D   diagnostic functions                      %5d" % d)
    print("  E   other modes / boot / retired probes       %5d" % e)
    print("      file scope (macros, typedefs, globals)    %5d" % file_scope)
    print("  ------------------------------------------------------")
    print("  main.c total                                  %5d"
          % (a + len(ctrl_lines) + b_ + len(diag_lines) + d + e + file_scope))
    return 0


if __name__ == "__main__":
    sys.exit(main())
