"""One-shot read of the live STM32 state over SWD, decoded by script.

Nothing here is hand-decoded on purpose. Every previous mistake in this
project came from reading a little-endian word by eye and getting a plausible
but wrong number, so the decoding is done by code and printed with labels.

Reads:
  USART1  SR DR BRR CR1     - is the receiver actually on, and at what divisor
  GPIOA   CRL CRH IDR ODR   - what mode is PA10 in, and what is on the pin
  RAM     the diagnostic counters and pa10_drive
"""

import re
import subprocess
import sys

CLI = (
    "D:\\Application\\STM32CubeIDE\\STM32CubeIDE_2.2.0\\STM32CubeIDE\\plugins\\"
    "com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304"
    "\\tools\\bin\\STM32_Programmer_CLI.exe"
)

# pca_rb is pca_rb_t[2], a 10-byte struct, one per axis. Everything that reads
# a readback field goes through RB() rather than spelling an address out, so a
# change to the struct is one edit and not eight.
#
# The field offsets mirror pca_rb_t in main.c and MUST be kept in step with it.
# Note they are NOT 0,1,2,3...: on and off are uint16_t, so each takes two
# bytes and mode1 starts at 4. Writing them as range(8) is a mistake that does
# not crash - every field silently reads one field early, which prints as
# "acked 32/5, match 32" and looks like a bus fault. It was made once, here.
# read_rb.py spells the same offsets out separately; if the two ever disagree,
# one of them is wrong.
RB_ON, RB_OFF, RB_M1, RB_AGAIN, RB_ACKED, RB_MATCH, RB_N, RB_BAD = 0, 2, 4, 5, 6, 7, 8, 9

RB_STRIDE = 10
RB_BASE = 0x20000074


def RB(axis, field):
    return RB_BASE + axis * RB_STRIDE + field

# pca_last_counts is uint16[2] - one early-out slot per axis.
def LAST(axis):
    return 0x20000000 + axis * 2


AXES = ("roll", "pitch")
HEADER = (0, 8)     # which PCA9685 header each axis is wired to; see pca_ch[]

REGIONS = [
    ("USART1", 0x40013800, 0x10),
    ("GPIOA", 0x40010800, 0x10),
    ("GPIOB", 0x40010C00, 0x10),
    ("SERVO", 0x20000000, 0x04),     # pca_last_counts[2]
    ("READBACK", RB_BASE, 2 * RB_STRIDE),  # pca_rb[2]
    ("BURST", 0x20000088, 0x05),     # pca_burst_n .. pca_burst_presc
    ("TRACE", 0x20000090, 0x09),     # pca_trace[9]
    ("SCAN", 0x20000099, 0x05),      # pca_resets, then i2c_scan_done/count/first/write
    ("DIAG", 0x200001A0, 0x0E),      # pca_diag_* , pa10_drive
    ("ANGLES", 0x200001B0, 0x28),    # roll/pitch/yaw .. imu_rx_errs
    ("TICK", 0x20000254, 0x04),
]

# 以上位址全部來自 arm-none-eabi-nm -n -S Debug/led_blink.elf（2026-09-29 重查）。
# 加任何全域或函式內的 static 都會推移 .bss/.data，這些數字就靜靜地錯了 ——
# 不會報錯，只會讓每個欄位錯開一格，印出「像樣但錯誤」的數字。
# 這個檔已經因為這件事錯了三次（pca_trace 讀到 pca_burst_acks、
# roll 讀到填充位元組、pca_diag_* 整組 +7），改完 main.c 一定要重新 nm 一次。
#
# 2026-09-29 這一次位移比較大，因為兩軸化把所有東西都往後推：
# pca_last_counts 2 → 4 bytes，pca_rb 8 → 20 bytes（struct，每個軸 10 bytes）。
# 那 14 個 byte 把後面每一項都推走：burst 0x7E→0x88、trace 0x84→0x90、
# pca_resets 0x8D→0x99、i2c_scan_* 0x8E→0x9A。
# pca_diag_* 原本就錯了（寫 0x194，實際 0x19D），這次一併修成 0x1A0。
# imu_* 從 0x1A4/0x1B8 移到 0x1B0/0x1C4，uwTick 0x248→0x254。


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
            # CubeProgrammer prints the word's VALUE, so the byte at the lowest
            # address is the low byte. Verified against uwTick in an earlier
            # session; do not reverse this.
            for b in range(4):
                mem[base + i * 4 + b] = (w >> (8 * b)) & 0xFF
    return mem


def u32(mem, addr):
    return int.from_bytes(bytes(mem.get(addr + i, 0) for i in range(4)), "little")


def u16(mem, addr):
    return int.from_bytes(bytes(mem.get(addr + i, 0) for i in range(2)), "little")


def u8(mem, addr):
    return mem.get(addr, 0)


def f32(mem, addr):
    import struct

    return struct.unpack("<f", bytes(mem.get(addr + i, 0) for i in range(4)))[0]


def main():
    mem = {}
    for name, addr, size in REGIONS:
        part = dump(addr, size)
        if not part:
            print("讀不到 %s @0x%08X —— ST-Link 沒連上，或 CubeIDE 正在 debug" % (name, addr))
            return 1
        mem.update(part)

    sr = u32(mem, 0x40013800)
    dr = u32(mem, 0x40013804)
    brr = u32(mem, 0x40013808)
    cr1 = u32(mem, 0x4001380C)
    crh = u32(mem, 0x40010804)
    idr = u32(mem, 0x40010808)
    odr = u32(mem, 0x4001080C)

    print("===== USART1 =====")
    print("  SR  = 0x%08X   %s" % (sr, "RXNE" if sr & 0x20 else "-"))
    print("  DR  = 0x%08X" % dr)
    # BRR is 0 when USART1's clock is off, i.e. when imu_uart_init() has not run
    # yet. Dividing by it used to kill the script here and hide everything
    # below it - which is the half of the output that actually mattered.
    if brr:
        print("  BRR = 0x%08X -> %d baud (PCLK2 72MHz / %d)" % (brr, 72000000 // brr, brr))
    else:
        print("  BRR = 0  <- USART1 時鐘沒開，程式還沒走到 imu_uart_init()")
    print(
        "  CR1 = 0x%08X   UE=%d RE=%d TE=%d"
        % (cr1, (cr1 >> 13) & 1, (cr1 >> 2) & 1, (cr1 >> 3) & 1)
    )

    print("===== GPIOA (PA10) =====")
    nib = (crh >> 8) & 0xF
    cnf, mode = (nib >> 2) & 3, nib & 3
    print("  CRH = 0x%08X  PA10 nibble = 0x%X (CNF=%d MODE=%d)" % (crh, nib, cnf, mode))
    print("  ODR bit10 = %d  -> %s" % ((odr >> 10) & 1, "pull-UP" if (odr >> 10) & 1 else "pull-DOWN"))
    print("  IDR bit10 = %d  -> 腳上實際是 %s" % ((idr >> 10) & 1, "高" if (idr >> 10) & 1 else "低"))

    print("===== 診斷計數器 =====")
    print("  pa10_drive          = %d" % u8(mem, 0x200001AD))
    print("  imu_frames          = %d" % u32(mem, 0x200001C4))
    print("  imu_badsum          = %d" % u32(mem, 0x200001C8))
    print("  imu_skipped         = %d" % u32(mem, 0x200001CC))
    print("  imu_rx_bytes        = %d   <- 關鍵" % u32(mem, 0x200001D0))
    print("  imu_rx_errs         = %d" % u32(mem, 0x200001D4))
    print("  roll/pitch/yaw      = %.2f / %.2f / %.2f" % (
        f32(mem, 0x200001B0), f32(mem, 0x200001B4), f32(mem, 0x200001B8)))

    print("===== PCA9685 自檢 =====")
    print("  pca_diag_ok       = %d" % u8(mem, 0x200001A0))
    print("  pca_diag_fail_code= %d   0=無錯 1=從不應答 2=應答但設定讀不回" % u8(mem, 0x200001A1))
    print("  pca_diag_tries    = %d   自檢花了幾次 poll" % u8(mem, 0x200001A2))
    print("  pca_diag_ack      = %d   掃描時應答的位址數" % u8(mem, 0x200001A3))
    print("  pca_diag_prescale = %d   (121 = 50Hz 設定正確)" % u8(mem, 0x200001A4))
    print("  pca_diag_reads    = %d  read_bad = %d" % (
        u8(mem, 0x200001AB), u8(mem, 0x200001AC)))

    print("===== 舵機命令 =====")
    for a in (0, 1):
        raw = u16(mem, LAST(a))
        # 0xFFFF 是「這個軸還沒被晶片確認過任何一次」的標記，不是脈波。
        # 印成 320000us 會看起來像控制律發瘋，其實是匯流排問題。
        shown = "--- (從未被 ACK)" if raw == 0xFFFF else "%.0f us" % (raw * (20000.0 / 4096.0))
        print("  %-5s pca_last_counts = %5d   -> %s" % (AXES[a], raw, shown))
    print("  pca_trace   = " + " ".join("%02X" % u8(mem, 0x20000090 + i) for i in range(9)))

    print("===== BURST 探針（本輪移到 readback 之前）=====")
    print("  pca_burst_n     = %d   總共做了幾次 pca_read(PCA_MODE1)" % u8(mem, 0x20000088))
    print("  pca_burst_first = %d   第幾次開始應答（0=從不）" % u8(mem, 0x20000089))
    print("  pca_burst_acks  = %d   其中幾次應答" % u16(mem, 0x2000008A))
    print("  pca_burst_val   = 0x%02X  ★第一次應答時的 MODE1（0x11=掉電重設, 0x20=沒重設）"
          % u8(mem, 0x2000008C))
    # 這裡原本寫「0x7F = 重設」，是錯的。PCA9685 的 PRESCALE 出廠值是 0x1E（30 → 200Hz），
    # 我們寫進去的是 0x79（121 → 50Hz）。2026-09-24 就是靠這個值抓到晶片真的掉電重啟：
    # MODE1 讀到 0x11 而 PRESCALE 讀到 0x1E，兩個暫存器同時回到出廠值 —— 匯流排雜訊或
    # 讀取失敗都不可能把 121 變成 30，只有重新上電會。錯的提示文字會讓人放過這個證據。
    print("  pca_burst_presc = 0x%02X  ★第一次應答時的 PRESCALE（0x1E=出廠/掉電重設, 0x79=我們寫的121）"
          % u8(mem, 0x2000008D))
    # 這四個位址 2026-09-29 又全部往後移：pca_rb 從 8 bytes 長成 20 bytes 的 struct 陣列，
    # 把後面每一項都推走。i2c_scan_ack 有 128 byte 對齊，所以 0x120 之後重新對齊，
    # 但 0x99..0x9D 這五個就是錯了，而且不會報錯。
    print("  i2c_scan_done   = 0x%02X  (0xA5=開機那次, 0x5A=迴圈裡那次, 0x00=沒跑)"
          % u8(mem, 0x2000009A))
    print("  i2c_scan_count  = %d  first = 0x%02X  write = 0x%02X" % (
        u8(mem, 0x2000009B), u8(mem, 0x2000009C), u8(mem, 0x2000009D)))

    # 這是整支腳本最重要的一段。上面每一項都只證明「命令被算了出來」，
    # 只有這裡證明「命令真的進了晶片」—— pca_set_us() 把六個 i2c_byte() 的
    # 回傳值全部丟掉，所以一顆對每次寫入都回 NACK 的晶片，跟一顆乖乖聽話的
    # 晶片，在 pca_last_counts 上長得一模一樣。
    print("===== 讀回比對：寫入真的落地了嗎 =====")
    # 每一軸各自一組。MODE1 是整顆晶片的暫存器，不是每個通道的，所以兩軸讀到的
    # 一定是同一個 byte —— 兩軸不一致不代表有兩顆晶片，代表匯流排掉了一次回答。
    # 這是唯一一個「兩列應該永遠相等」的欄位，把它當交叉檢查用。
    rb = {}
    for a in (0, 1):
        rb[a] = {
            "on": u16(mem, RB(a, RB_ON)),
            "off": u16(mem, RB(a, RB_OFF)),
            "mode1": u8(mem, RB(a, RB_M1)),
            "again": u8(mem, RB(a, RB_AGAIN)),
            "acked": u8(mem, RB(a, RB_ACKED)),
            "match": u8(mem, RB(a, RB_MATCH)),
            "n": u8(mem, RB(a, RB_N)),
            "bad": u8(mem, RB(a, RB_BAD)),
            "last": u16(mem, LAST(a)),
        }

    burst_n     = u8(mem, 0x20000088)
    burst_first = u8(mem, 0x20000089)
    burst_acks  = u16(mem, 0x2000008A)
    burst_val   = u8(mem, 0x2000008C)
    burst_presc = u8(mem, 0x2000008D)
    # 這是整支腳本最重要的一個新數字。晶片掉電重啟是舵機「動一下就不動」的成因，
    # 而它沒有任何其他痕跡可看：靜態電壓正常、暫存器讀寫正常、自檢也過。唯一
    # 的證據就是晶片回到出廠值 —— 這件事只有韌體每秒讀一次 MODE1 才看得到。
    # 只要舵機做大動作時這個數字就往上跑，就是舵機電流把晶片的電源拉垮了。
    resets = u8(mem, 0x20000099)
    print("  pca_resets    = %d   ★晶片掉電重啟被逮到幾次（0 才是好的）" % resets)
    for a in (0, 1):
        r = rb[a]
        print("  --- %s (header %d) ---" % (AXES[a], HEADER[a]))
        print("    pca_rb_n      = %d   總共跑了幾次讀回" % r["n"])
        print("    pca_rb_bad    = %d   其中幾次對不上" % r["bad"])
        print("    pca_rb_acked  = %d/5 晶片應答了幾個 byte" % r["acked"])
        print("    pca_rb_match  = %d   1 = 晶片回報的脈波等於命令" % r["match"])
        print("    pca_rb_mode1  = 0x%02X  MODE1 第一次讀（0x20 才對）" % r["mode1"])
        print("    pca_rb_again  = 0x%02X  MODE1 緊接著再讀一次（同一個呼叫）" % r["again"])
        print("    晶片回報  ON=%d  OFF=%d   命令 counts=%d"
              % (r["on"], r["off"], r["last"]))

    # 兩軸的 MODE1 是同一顆晶片的同一個暫存器，這兩列必須相等。
    if rb[0]["n"] and rb[1]["n"] and rb[0]["mode1"] != rb[1]["mode1"]:
        print("  -> ★ 兩軸的 MODE1 不同（0x%02X vs 0x%02X）。同一個暫存器讀出兩個值，"
              % (rb[0]["mode1"], rb[1]["mode1"]))
        print("     代表其中一次讀取是掉回答的，不是真的有兩顆晶片。匯流排不穩。")

    # 這兩組數字是整支腳本的核心。兩次一模一樣的 pca_read(PCA_MODE1) 中間
    # 沒有任何其他東西，所以它們必須相等；不相等就代表問題在程式不在電路。
    for a in (0, 1):
        r = rb[a]
        m1, ag = r["mode1"], r["again"]
        if r["n"] == 0:
            print("  %s -> 讀回一次都沒跑完。韌體不是這一版，或迴圈卡在別的地方。" % AXES[a])
        elif m1 == 0x20 and r["match"]:
            # 這一行以前印「問題在脈波暫存器或寫入路徑」—— 寫的時候 MODE1 是
            # 唯一讀得到的暫存器，脈波還沒對上，所以 0x20 只能證明「匯流排活著，
            # 但還不知道寫入有沒有落地」。現在 match 一起看了，這個組合就是
            # 全部正常，再喊有問題是假的警報，會讓人去追一個不存在的故障。
            print("  %s -> MODE1 0x20、脈波也對得上（bad %d）：這一軸完全正常。"
                  % (AXES[a], r["bad"]))
        elif m1 == 0x20:
            print("  %s -> MODE1 讀到 0x20（匯流排是通的），但脈波對不上（bad %d）。"
                  % (AXES[a], r["bad"]))
            print("     寫入路徑有問題，或這一軸的排針不是 pca_ch[] 寫的那一個。")
        elif ag == 0x20:
            print("  %s -> 奇案：第一次讀失敗(0x%02X)，接著同一個呼叫成功(0x20)。" % (AXES[a], m1))
            print("     中間沒有任何別的東西 -> 這是程式問題，不是接線或電源。")
        elif ag == 0x11 and m1 == 0x11:
            # 0x11 不是「讀不到」，是「讀到了，而且讀到出廠預設值」。這兩件事以前會
            # 一起被印成「匯流排沒反應」，於是最重要的線索被當成雜訊丟掉。要一起看
            # PRESCALE：兩個暫存器同時回到出廠值才叫重設，只有 MODE1 是 0x11 而
            # PRESCALE 還是 0x79 的話，那是別的問題。
            print("  %s -> MODE1 兩次都讀到 0x11 = PCA9685 出廠預設值（SLEEP 設著、輸出全關）。" % AXES[a])
            print("     匯流排是通的，是晶片自己回到出廠狀態了 —— 去看上面的 pca_burst_presc：")
            print("     若它也是 0x1E（出廠值）而不是 0x79，代表晶片真的掉電重啟過，")
            print("     這是硬體供電問題（VCC 不穩 / 舵機起動電流把電源拉垮），不是程式。")
        elif ag != m1:
            print("  %s -> 兩次讀值不同（0x%02X vs 0x%02X）：匯流排不穩。" % (AXES[a], m1, ag))
        else:
            print("  %s -> 兩次連續讀都失敗（0x%02X）。這一段時間匯流排真的沒反應。" % (AXES[a], m1))

    # 「每個舵機是不是插在韌體以為的那個排針上」—— 這是兩軸化之後新增的一問，
    # 而它只有分軸看才答得出來。整塊晶片一起看的話，一軸對一軸錯會平均成「還好」。
    if rb[0]["n"] and rb[1]["n"]:
        ok = [a for a in (0, 1) if rb[a]["match"]]
        if len(ok) == 2:
            print("  -> 兩軸都比對成功：命令真的進了晶片的兩個通道。")
        elif len(ok) == 1:
            a = 1 - ok[0]
            print("  -> ★ 只有 %s 比對成功，%s 一直對不上。" % (AXES[ok[0]], AXES[a]))
            print("     寫入路徑通了，所以問題只在那一個通道：舵機插錯排針，")
            print("     或那個排針根本沒被驅動（pca_ch[] 寫錯通道號）。")
        else:
            print("  -> 兩軸都對不上：往 MODE1 那一段看，問題在匯流排而不是通道。")

    # 最壞的那一軸。下面這些判斷問的是「讀回這件事到底有沒有在動」，
    # 所以取兩軸的最小值：只要有一軸完全讀不到，答案就是「沒在動」。
    rb_acked = min(rb[0]["acked"], rb[1]["acked"]) if rb[0]["n"] and rb[1]["n"] else 0

    # ── burst 探針 ──────────────────────────────────────────────────
    # 這一整段以前是本專案的主線實驗（「為什麼排在前面的那個呼叫失敗」），
    # 2026-09-24 結案：兩個呼叫從來沒有不同，它們只是先後站在同一條壞掉的
    # 地上。PCA9685 舵機排針的 GND（G3）與板子邏輯地（G2）不通，量到約 1200Ω，
    # 匯流排是在拿一個不存在的參考去判斷位準，所以「先跑的那個」先撞上。
    # 飛線把 G3 接到 G2 之後 burst 160/160、readback 也對得上。
    # 所以這裡現在是回歸檢查，不是待解的問題：burst_first 掉回 1 或
    # readback 的 acked 掉下來，就是那條地又鬆了，或是新的地問題。
    print()
    print("===== BURST 探針（回歸檢查：地還在不在）=====")
    if burst_n == 0:
        print("  burst 沒跑完 —— 韌體不是這一版。")
    elif burst_first == 0:
        print("  burst 一次都沒應答（0/%d）。地或匯流排出問題了 ——" % burst_n)
        print("  先量 PCA9685 側邊 GND 排針與舵機排針 GND 之間通不通。")
    elif burst_first > 1:
        print("  ★ burst 前 %d 次讀取失敗，第 %d 次起才應答（%d/%d）。"
              % (burst_first - 1, burst_first, burst_acks, burst_n))
        print("     地上次就是這樣開始壞的（先跑的那個先失敗）。")
        print("     去量 G2（STM32 GND）與 G3（舵機排針 GND）通不通。")
    else:
        print("  burst %d/%d，第一次就應答 —— 匯流排是乾淨的。" % (burst_acks, burst_n))

    if burst_val == 0x11 or burst_presc == 0x1E:
        # 兩個暫存器同時回到出廠值才叫掉電重設。只有 MODE1 是 0x11 而 PRESCALE
        # 還是 0x79 的話，那是別的問題 —— 錯的提示文字會讓人放過這個證據。
        print("  ★ 晶片回到出廠值了：MODE1 = 0x%02X、PRESCALE = 0x%02X"
              % (burst_val, burst_presc))
        print("     MODE1 出廠 0x11、PRESCALE 出廠 0x1E，我們寫的是 0x20 和 0x79。")
        print("     匯流排雜訊不可能把 121 變成 30 —— 只有重新上電會。")
        print("     這是硬體供電問題：舵機起動電流把 VCC 拉垮。")

    if rb[0]["n"] and rb[1]["n"] and rb[0]["match"] and rb[1]["match"]:
        print("  -> 兩軸的寫入都確實落到晶片，通道各自正確。")
        print("     韌體這一側到此為止全部可證：角度在動、控制律在算、")
        print("     命令進得了晶片的兩個通道、晶片也在驅動那兩個排針。")
        print("     若舵機還是不動，剩下的全在 PCA9685 之外：")
        print("     V+ 舵機電源、訊號線、舵機本身。")

    print("  uwTick            = %d ms" % u32(mem, 0x20000254))
    return 0


if __name__ == "__main__":
    sys.exit(main())
