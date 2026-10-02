"""Live roll/pitch/yaw and the two servo commands, over SWD, every half second.

Why this exists. Reading a number once per run means one round trip per
observation, and the thing being tested is "what does it say when I hold it
*this* way" - a question that needs to be answered while the module is being
moved, not afterwards. So print it continuously instead.

It became more useful when the second axis arrived. Two servo commands on
screen next to two angles is how the decoupling is checked: tilt the base in
roll only and the pitch column should not move. That test cannot be run from a
one-shot read, because "did it move" is a question about two moments.

The angles on screen are the raw IMU output, NOT a filtered or setpoint-corrected
value - there is no setpoint. Level is where gravity says it is, so a level
module reads zero and any non-zero reading is real tilt.

This only reads RAM. It does not halt the CPU (mode=hotplug), so the firmware
keeps running and the IMU keeps being parsed while this is on screen.

Usage:
    PYTHONIOENCODING=utf-8 python tools/watch_imu.py          # until Ctrl-C
    PYTHONIOENCODING=utf-8 python tools/watch_imu.py 20       # 20 samples, then exit
"""

import re
import struct
import subprocess
import sys
import time

# The console this runs in is not always UTF-8 - PowerShell defaults to this
# machine's ANSI code page, and a Chinese character it cannot encode would
# otherwise kill the script with a UnicodeEncodeError partway through a run.
# Doing it here rather than asking the caller to remember
# PYTHONIOENCODING=utf-8 means the command to type is just "python tools/watch_imu.py",
# which is the same in PowerShell, cmd and bash.
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

CLI = (
    "D:\\Application\\STM32CubeIDE\\STM32CubeIDE_2.2.0\\STM32CubeIDE\\plugins\\"
    "com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304"
    "\\tools\\bin\\STM32_Programmer_CLI.exe"
)

# Everything from pca_last_counts through imu_rx_errs, in ONE -r32 read.
#
# One read rather than two, because each dump() spawns a fresh
# STM32_Programmer_CLI.exe and that costs seconds - two reads would double the
# sample interval, and the thing being watched here is how the command tracks
# the angle, which needs the two numbers to be as close to simultaneous as
# possible. Re-check the offsets with
#   arm-none-eabi-nm -n -S Debug/led_blink.elf
# after any edit that adds a global, exactly as read_state.py's header says.
MEM_BASE = 0x20000000
MEM_SIZE = 0x1D8           # through imu_rx_errs

# pca_last_counts is uint16[2] now - roll at +0, pitch at +2. Same order as the
# axis_t enum and as pca_ch[]: AXIS_ROLL, then AXIS_PITCH.
SERVO_ADDR = 0x20000000    # pca_last_counts, uint16[2] - the command as written
OFF_ROLL   = 0x1B0         # imu_roll_deg
OFF_PITCH  = 0x1B4
OFF_YAW    = 0x1B8
OFF_FRAMES = 0x1C4         # imu_frames    - complete, checksum-good frames
OFF_BYTES  = 0x1D0         # imu_rx_bytes  - every byte pulled out of DR

US_PER_COUNT = 20000.0 / 4096.0   # 4.8828 us per PCA9685 count


def dump(addr, size):
    """Return {address: byte} for one -r32 read."""
    out = subprocess.run(
        [CLI, "-c", "port=SWD", "mode=hotplug", "-r32", "0x%08X" % addr, str(size)],
        capture_output=True,
        text=True,
    ).stdout

    mem = {}
    for line in out.splitlines():
        m = re.match(r"\s*0x([0-9A-Fa-f]{8})\s*:\s*([0-9A-Fa-f ]+?)\s*$", line)
        if not m:
            continue
        base = int(m.group(1), 16)
        for i, word in enumerate(m.group(2).split()):
            w = int(word, 16)
            # CubeProgrammer prints the word's VALUE, lowest address = low byte.
            for b in range(4):
                mem[base + i * 4 + b] = (w >> (8 * b)) & 0xFF
    return mem


def f32(mem, addr):
    return struct.unpack("<f", bytes(mem.get(addr + i, 0) for i in range(4)))[0]


def u32(mem, addr):
    return int.from_bytes(bytes(mem.get(addr + i, 0) for i in range(4)), "little")


def u16(mem, addr):
    return int.from_bytes(bytes(mem.get(addr + i, 0) for i in range(2)), "little")


def main():
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    prev_frames = None
    prev_bytes = None

    # The "bytes" column is what separates the two ways this can look dead.
    # bytes climbing with frames frozen means the module is still talking and
    # our parser is what stopped finding frames - a firmware bug, and the
    # angles on screen are stale. Both frozen means nothing is arriving at
    # PA10 at all - wiring or the module itself.
    print("  roll      pitch       yaw     roll_us  pitch_us   frames    bytes  link")
    print("  --------  ----------  ---------  -------  --------  --------  -------  ------")

    i = 0
    while count == 0 or i < count:
        mem = dump(MEM_BASE, MEM_SIZE)
        if not mem:
            print("  讀不到 —— ST-Link 沒連上，或 CubeIDE 正在 debug")
            return 1

        r = f32(mem, MEM_BASE + OFF_ROLL)
        p = f32(mem, MEM_BASE + OFF_PITCH)
        y = f32(mem, MEM_BASE + OFF_YAW)
        fr = u32(mem, MEM_BASE + OFF_FRAMES)
        by = u32(mem, MEM_BASE + OFF_BYTES)
        # Both axes, from the same struct, so the two commands are as close to
        # simultaneous as one read can make them.
        #
        # 0xFFFF is pca_reinit()'s "nothing has been ACKed on this axis yet"
        # marker, not a pulse. Converting it would print 320000us, which looks
        # like a wild command rather than what it is - an axis the chip has
        # never acknowledged. Print it as dashed instead, so a bus fault cannot
        # be misread as a control-law fault.
        raw_r = u16(mem, SERVO_ADDR + 0)
        raw_p = u16(mem, SERVO_ADDR + 2)
        cmd_r = "   ---" if raw_r == 0xFFFF else "%6.0f" % (raw_r * US_PER_COUNT)
        cmd_p = "   ---" if raw_p == 0xFFFF else "%6.0f" % (raw_p * US_PER_COUNT)

        if prev_frames is None:
            link = "?"
        elif fr != prev_frames:
            link = "OK"
        elif by != prev_bytes:
            link = "解析卡住"
        else:
            link = "沒資料"
        prev_frames, prev_bytes = fr, by

        print("  %8.2f  %10.2f  %9.2f  %7s  %8s  %8d  %7d  %s"
              % (r, p, y, cmd_r, cmd_p, fr, by, link))

        i += 1
        if count == 0 or i < count:
            time.sleep(0.5)

    return 0


if __name__ == "__main__":
    sys.exit(main())
