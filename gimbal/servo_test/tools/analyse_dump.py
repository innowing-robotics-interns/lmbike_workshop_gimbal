"""Read back the IMU bring-up capture that was dumped out of the STM32's RAM.

The board did the same scan in C; this repeats it here so the two can be
compared. If they agree, the on-board scanner is doing what it looks like it
does, and the bytes below are the module's real output.
"""

import re
import sys

BAUDS = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000, 256000]
CAP_LEN = 256
LEN_MIN, LEN_MAX = 5, 64

CAP_BUF = 0x20000070
CAP_N = 0x20000a70
LOCK_INDEX = 0x20000a84
LOCK_H1 = 0x20000a85
LOCK_H2 = 0x20000a86
LOCK_LEN = 0x20000a87
LOCK_FUNC = 0x20000a88
LOCK_CHAIN = 0x20000a8a
LOCK_BEST = 0x20000a8c

BASE = CAP_BUF


def load(path):
    mem = {}
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            m = re.match(r"\s*0x([0-9A-Fa-f]{8})\s*:\s*(.*)$", line)
            if not m:
                continue
            addr = int(m.group(1), 16)
            for i, word in enumerate(m.group(2).split()):
                # CubeProgrammer prints each 32-bit word as its VALUE, so the low
                # byte of that value is the byte at the low address. Checked
                # against uwTick, which read back as 00000626 for 1574.
                for b in range(4):
                    mem[addr + i * 4 + b] = (int(word, 16) >> (8 * b)) & 0xFF
    return mem


def rd(mem, addr, n):
    return bytes(mem.get(addr + i, 0) for i in range(n))


def rd16(mem, addr):
    return int.from_bytes(rd(mem, addr, 2), "little")


def scan(buf):
    """Same rule as the C: walk a chain of frames, checksum = sum of all but last."""
    best, at = 0, None
    n = len(buf)
    for start in range(n - LEN_MIN + 1):
        p, chain = start, 0
        while p + LEN_MIN <= n:
            ln = buf[p + 2]
            if ln < LEN_MIN or ln > LEN_MAX or p + ln > n:
                break
            if sum(buf[p : p + ln - 1]) & 0xFF != buf[p + ln - 1]:
                break
            chain += 1
            p += ln
        if chain > best:
            best, at = chain, start
    return best, at


def main(path):
    mem = load(path)
    print(f"cap_n  @0x{CAP_N:08X}: " + " ".join(str(rd16(mem, CAP_N + 2 * i)) for i in range(10)))
    print(
        "lock   index=%d h1=0x%02X h2=0x%02X len=%d func=0x%02X chain=%d best=%d"
        % (
            rd(mem, LOCK_INDEX, 1)[0],
            rd(mem, LOCK_H1, 1)[0],
            rd(mem, LOCK_H2, 1)[0],
            rd(mem, LOCK_LEN, 1)[0],
            rd(mem, LOCK_FUNC, 1)[0],
            rd16(mem, LOCK_CHAIN),
            rd16(mem, LOCK_BEST),
        )
    )
    print()

    for i, baud in enumerate(BAUDS):
        buf = rd(mem, CAP_BUF + i * CAP_LEN, CAP_LEN)
        chain, at = scan(buf)
        head = " ".join(f"{b:02X}" for b in buf[:32])
        verdict = "*** LOCK ***" if chain >= 3 else ""
        print(f"[{i}] {baud:>8}  chain={chain:<3} first={at}  {verdict}")
        print(f"      {head}")
        if chain >= 2 and at is not None:
            # Show the frames the scanner found, decoded per the documented layout
            p, k = at, 0
            while p + LEN_MIN <= CAP_LEN and k < 8:
                ln = buf[p + 2]
                if ln < LEN_MIN or ln > LEN_MAX or p + ln > CAP_LEN:
                    break
                if sum(buf[p : p + ln - 1]) & 0xFF != buf[p + ln - 1]:
                    break
                print(f"      frame {k}: {' '.join(f'{b:02X}' for b in buf[p:p+ln])}")
                p += ln
                k += 1
        print()


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "dump_raw.txt")
