"""Read the I2C bus scan that pca_selftest() runs on its failure path.

The scan is the whole point of the failure path: it answers "is the chip
there at all, or only deaf to the fast probe?" by walking every legal
address and recording who answers.

Decoding is done here rather than by eye, for the usual reason.
"""

import re
import subprocess
import sys

CLI = (
    "D:\\Application\\STM32CubeIDE\\STM32CubeIDE_2.2.0\\STM32CubeIDE\\plugins\\"
    "com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304"
    "\\tools\\bin\\STM32_Programmer_CLI.exe"
)

SCAN_HEAD = 0x2000009A   # done, count, first, write
ACK_BASE = 0x200000A0    # ack[128].  NOTE: indexed by the raw address, so
                         # ack[0x40] lives at ACK_BASE + 0x40 - the first
                         # eight entries are simply never used. Getting this
                         # wrong shifts every hit by 8 and invents an address
                         # that does not exist.
MODE1_BASE = 0x20000120  # mode1[128], same indexing

# 2026-09-29：兩軸化把整個 .bss 又往後推。pca_rb 從 8 bytes 長成 20 bytes 的
# struct 陣列、pca_last_counts 從 2 長到 4，前面多出 14 bytes，後面每一項都跟著走。
# 上面三個數字是 arm-none-eabi-nm -n -S Debug/led_blink.elf 查出來的。
# 上一次（2026-09-23）這裡也移過 0x0C —— 這已經是第三次，改完 main.c 一定要重查。


def dump(addr, size):
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
            for b in range(4):          # low address = low byte of the value
                mem[base + i * 4 + b] = (w >> (8 * b)) & 0xFF
    return mem


def main():
    mem = dump(SCAN_HEAD, 4)
    if not mem:
        print("讀不到 RAM —— ST-Link 沒連上？")
        return 1

    done = mem.get(SCAN_HEAD, 0)
    count = mem.get(SCAN_HEAD + 1, 0)
    first = mem.get(SCAN_HEAD + 2, 0)
    write = mem.get(SCAN_HEAD + 3, 0)

    print("===== i2c_scan_bus() 的結果 =====")
    print("  done  = %d" % done)
    print("  count = %d   <- 有幾個位址應答" % count)
    print("  first = 0x%02X" % first)
    print("  write = 0x%02X  (PRESCALE 讀回，121=0x79 才對)" % write)
    print()

    if count == 0:
        print("  ** 沒有任何位址應答 —— 晶片完全不在匯流排上。")
        print("     屬於電氣問題：SDA/SCL 接反、VCC 沒電、沒共地、或排針沒插好。**")
        return 0

    ack = dump(ACK_BASE, 0x80)
    mode1 = dump(MODE1_BASE, 0x80)

    print("  應答的位址：")
    for a in range(0x08, 0x78):
        if ack.get(ACK_BASE + a, 0):
            print("    0x%02X   MODE1 讀回 = 0x%02X" % (a, mode1.get(MODE1_BASE + a, 0)))

    print()
    print("  判讀：")
    if first == 0x40:
        print("    0x40 應答了 —— 晶片是好的、接線是通的。")
        print("    它只是在開機後超過 2 秒才開始應答，比自檢窗口還晚。")
        print("    => 這是時序/上電競爭，不是接線錯誤。")
    else:
        print("    0x40 沒有應答，但別的位址有 —— 位址跳線或 ALLCALL 的情況。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
