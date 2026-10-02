"""Is the PCA9685 actually holding the pulse the firmware asked for - per axis?

The question this settles: when the command is steady and the servo still
jumps, the fault is on one side or the other of a single wire - the chip's
PWM output, or the servo's own position loop. Everything in the firmware is
readable, so the chip can be checked directly instead of argued about.

pca_readback() runs once a second, once per axis, and stores what it read.
This pulls those fields out of RAM over SWD and puts the chip's OFF register,
the last command the chip ACKNOWLEDGED, and the command the control law wants,
side by side. If those three agree, the chip is putting out exactly the right
pulse and nothing upstream of the servo connector is at fault.

TWO AXES, and the split matters. The angle, the command and the chip's OFF
register are per axis - roll is on header 0, pitch on header 8. MODE1 is NOT:
it is a register of the chip, not of a channel, so both axes are asking the
same question about the same byte and a disagreement between the two rows in
that column means a lost answer on the bus, not two different chips. `rst` is
printed once for the same reason. pca_resets only moves when the chip has
rebooted, and a reboot takes both channels with it.

The column that answers "is each servo on the header the firmware thinks it
is" is `match` on a per-axis row. Row pitch stuck at match 0 while its `want`
moves means the pitch command is going out and the chip is not holding it -
wrong header, or a header that is not driven.

Usage:  py tools/read_rb.py [samples]      (default 12, about 25 seconds)
"""

import re
import struct
import subprocess
import sys
import time

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

PROG = (
    r"D:\Application\STM32CubeIDE\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins"
    r"\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304"
    r"\tools\bin\STM32_Programmer_CLI.exe"
)

BASE = 0x20000000
LEN = 0x1D0                     # 0x00 .. 0x1CF: the PCA fields AND the IMU angles

# ---- per axis -------------------------------------------------------------
# pca_last_counts is uint16[2]; pca_rb is pca_rb_t[2] with a 10-byte stride.
# Both were single scalars until the second axis arrived - see the notes at
# their definitions in main.c for why a shared slot cannot work.
OFF_LAST = (0x00, 0x02)         # u16  per axis: last command the chip ACKed
RB_BASE = 0x74                  # pca_rb_t[0]; axis a sits at RB_BASE + a*STRIDE
RB_STRIDE = 10

RB_ON = 0                       # u16  chip's LEDn_ON_L/H
RB_OFF = 2                      # u16  chip's LEDn_OFF_L/H
RB_MODE1 = 4                    # u8   chip's MODE1
RB_AGAIN = 5                    # u8   MODE1 read a second time
RB_ACKED = 6                    # u8   ACKs out of 5
RB_MATCH = 7                    # u8   does the chip agree with the command
RB_N = 8                        # u8   readbacks taken
RB_BAD = 9                      # u8   readbacks that disagreed

# ---- per chip -------------------------------------------------------------
OFF_RESETS = 0x99               # u8   power-on-default sightings

# ---- IMU ------------------------------------------------------------------
OFF_ROLL = 0x1B0                # f32  imu_roll_deg
OFF_PITCH = 0x1B4               # f32  imu_pitch_deg
OFF_FRAMES = 0x1C4              # u32  imu_frames

US_PER_COUNT = 20000.0 / 4096.0

# The P term of the control law, duplicated from gimbal_control.c on purpose:
# the `want` column is derived independently of the firmware's stored pulse.
# Full PID also has Ki/Kd with per-axis state; this script only recomputes
# CENTRE + SIGN * KP * angle. Keep KP/SIGN/CENTRE in sync with gimbal_control.c.
CENTRE = 1500.0
KP = (3.0, 3.0)                 # AXIS_ROLL, AXIS_PITCH  (KP[] in gimbal_control.c)
KI = (0.05, 0.05)               # documented here for sync; not used in `want`
KD = (0.15, 0.15)
SIGN = (1.0, 1.0)               # AXIS_ROLL, AXIS_PITCH
AXIS_NAME = ("ROLL", "PITCH")
ANGLE_OFF = (OFF_ROLL, OFF_PITCH)


def read_block():
    out = subprocess.run(
        [PROG, "-c", "port=SWD", "mode=hotplug",
         "-r32", hex(BASE), str(LEN)],
        capture_output=True, text=True, errors="replace", timeout=60,
    ).stdout

    # The CLI prints whole 32-bit words, little-endian per word:
    #     0x20000000 : 0000FFFF 044AA200 ...
    # so the words have to be split back into bytes in the right order.
    data = bytearray()
    for line in out.splitlines():
        m = re.match(r"^\s*0x([0-9A-Fa-f]{8})\s*:\s*((?:[0-9A-Fa-f]{8}\s*)+)$", line)
        if not m:
            continue
        addr = int(m.group(1), 16)
        if addr != BASE + len(data):
            continue                      # not the chunk that follows the last
        for word in m.group(2).split():
            v = int(word, 16)
            data.extend([v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF])

    return data


def u16(d, o):
    return d[o] | (d[o + 1] << 8)


def u32(d, o):
    return d[o] | (d[o + 1] << 8) | (d[o + 2] << 16) | (d[o + 3] << 24)


def f32(d, o):
    return struct.unpack("<f", bytes(d[o:o + 4]))[0]


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 12

    print(f"{'axis':>6}  {'angle':>8}  {'want':>6}  {'lastACK':>7}  {'chipOFF':>7}  "
          f"{'drift':>5}  {'mode1':>5}  {'ack':>3}  {'match':>5}  {'rbN':>4}  "
          f"{'rbBad':>5}  {'rst':>3}  {'frames':>7}")
    print("-" * 100)

    for _ in range(n):
        d = read_block()
        if len(d) < LEN:
            print(f"short read ({len(d)} of {LEN} bytes) - is the board powered?")
            time.sleep(1.0)
            continue

        resets = d[OFF_RESETS]
        frames = u32(d, OFF_FRAMES)

        for a in (0, 1):
            base = RB_BASE + a * RB_STRIDE
            angle = f32(d, ANGLE_OFF[a])
            want = CENTRE + SIGN[a] * KP[a] * angle
            last = u16(d, OFF_LAST[a]) * US_PER_COUNT
            chip_off = u16(d, base + RB_OFF) * US_PER_COUNT

            # `drift` is chipOFF minus lastACK, and the two are NOT snapshots of
            # the same instant - that is the whole point of the column, and it is
            # why it is not a fault indicator.
            #
            # chipOFF was captured by pca_readback() when it last ran, up to a
            # second ago; lastACK is read live, right now. The command is
            # recomputed on every IMU frame, so on a rig that is being moved or
            # is still settling the two legitimately differ by however far the
            # angle travelled in that second. It reads 0 only when the command
            # has not changed since the last readback - which is the condition
            # this column is really reporting.
            #
            # The authoritative agreement check is `match`, which the firmware
            # computes by comparing chipOFF against last_counts AT THE MOMENT OF
            # THE READBACK. That comparison cannot suffer from this skew. So:
            # read `match` for "did the write land", read `drift` for "has the
            # rig stopped moving". Do not read a non-zero drift as a bus fault.
            print(f"{AXIS_NAME[a]:>6}  {angle:8.2f}  {want:6.0f}  {last:7.0f}  "
                  f"{chip_off:7.0f}  {chip_off - last:5.0f}  "
                  f"0x{d[base + RB_MODE1]:02X}  {d[base + RB_ACKED]:3d}  "
                  f"{d[base + RB_MATCH]:5d}  {d[base + RB_N]:4d}  {d[base + RB_BAD]:5d}  "
                  f"{resets:3d}  {frames:7d}")

        print()
        time.sleep(0.3)

    print("angle    = what the IMU reports for that axis, in degrees")
    print("want     = CENTRE + SIGN * KP * angle  (P term only; I/D need firmware state)")
    print("lastACK  = the last command the chip ACKNOWLEDGED (RAM), read live, in us")
    print("chipOFF  = LEDn_OFF read back OUT of the chip, as of the last readback")
    print("drift    = chipOFF - lastACK. These are NOT the same instant, so a")
    print("           non-zero value only means the command moved since the last")
    print("           readback. Hold the rig still and it goes to 0.")
    print("mode1    = 0x20 is running; anything with bit4 set = outputs OFF")
    print("match    = 1 = write landed. Compares like-for-like at readback time,")
    print("           so THIS is the column to trust, not drift.")
    print("rbBad    = readbacks where match was 0. Should stay 0.")
    print("rst      = times MODE1 read the power-on default 0x11 (per chip)")


if __name__ == "__main__":
    main()
