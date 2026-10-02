<#
  即時顯示 STM32 裡的 IMU 角度。

  原理：ST-Link 透過 SWD 把 RAM 讀出來（mode=hotplug -> 不停 CPU），
  每秒讀一次並解碼成度數。完全不需要 IDE 的除錯器。

  前提：CubeIDE 不能在 debug session 中。ST-Link 一次只能有一個使用者，
        如果 IDE 正在 debug，這支腳本會連不上。

  用法：
        powershell -ExecutionPolicy Bypass -File watch_angle.ps1
        powershell -ExecutionPolicy Bypass -File watch_angle.ps1 -Once   # 只讀一次
        Ctrl+C 結束

  注意：這個檔案必須存成「UTF-8 with BOM」。
        PowerShell 5.1 讀沒有 BOM 的 .ps1 會當成 ANSI(GBK)，
        中文註解會變亂碼並可能提早結束字串而導致語法錯誤。
#>

param(
    [int]$Count = 0   # 讀幾次就結束；0 = 一直讀到 Ctrl+C
)

$ErrorActionPreference = 'Stop'

$cli = "D:\Application\STM32CubeIDE\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe"

# 位址是 arm-none-eabi-nm -S led_blink.elf 讀出來的。
# 注意：改動 main.c 後變數順序可能變，位址要重新確認。
#
# 2026-09-23，改成 PCA9685 之後位移了 +4：pca_set_us() 裡的
# static uint16_t last_counts 被放進 .bss 最前面（2 bytes 資料，但把後面
# 4-byte 對齊的 SystemCoreClock 推了一格，所以整體 +4）。
# 危險之處：舊的 0x20000070 不會報錯，只會讓每個欄位錯開一格 ——
# roll 讀到填充位元組、pitch 讀到真正的 roll，顯示出來仍是「像樣的數字」。
#
# 2026-09-23 同日再位移到 0x20000194：加了 pca_trace[9] + pca_diag_* 一批
# 診斷用的 volatile 全域變數。這次底下六個位移「剛好沒變」，只有 $BASE 要跟。
# 這已經是同一輪裡第二次手改這個數字了 —— 真正該做的是讓腳本自己跑
# arm-none-eabi-nm 去查，而不是每次改 main.c 就祈禱還記得回來改這裡。
#
# 2026-09-23 第三次位移，0x20000194 -> 0x200001A0：加了 pca_readback()
# 用的 pca_rb_* 六個全域（放在 0x74，把後面的東西全部往後推）。
# 底下八個 $OFF_* 這次完全沒動 —— 問的是同一組變數、位移相同。
# 這是第三次了：改完 main.c 一定要重跑一次
#   arm-none-eabi-nm -n -S Debug/led_blink.elf
# 再回來對 $BASE，不然讀到的會是「錯開一格但看起來很合理」的數字。
#
# 2026-09-29 第四次位移，0x200001A0 -> 0x200001B0：兩軸化把 pca_last_counts
# 從 2 bytes 長成 uint16[2]（4 bytes），pca_rb_* 六個純量換成 pca_rb_t[2]
# struct 陣列（8 -> 20 bytes），前面合計多出 14 bytes，後面每一項都跟著走。
# 這次底下八個 $OFF_* 還是沒動，因為問的仍是 imu_* 那一組、彼此相對位置不變，
# 只有 $BASE 要跟。**這是第四次手改這個數字了。** 真正該做的是讓腳本自己去跑
# arm-none-eabi-nm，而不是每次改完 main.c 就祈禱還記得回來改這裡。
$BASE = 0x200001B0
$LEN  = 0x28

# 區塊內的位移
$OFF_ROLL   = 0x00
$OFF_PITCH  = 0x04
$OFF_YAW    = 0x08
$OFF_FRAMES = 0x14
$OFF_BADSUM = 0x18
$OFF_SKIP   = 0x1C
$OFF_RXBYTE = 0x20    # imu_rx_bytes：PA10 收到的原始 byte 數（parser 之前）
$OFF_RXERR  = 0x24    # imu_rx_errs ：其中帶 UART 錯誤旗標的


# 讀回來的原始 byte 放在這裡。刻意用 byte[] 而不是 hashtable：
# hashtable 的 key 是「裝箱的數字」，Int32 和 Int64 就算數值相同，
# .Equals() 也會回 False，查不到就變 $null，再轉成 byte 就靜靜變成 0。
# 用陣列就完全沒有這個問題。
$script:buf = New-Object byte[] $LEN


function Read-Block {
    $lines = & $cli -c port=SWD mode=hotplug -r32 ("0x{0:X8}" -f $BASE) $LEN 2>&1
    for ($k = 0; $k -lt $LEN; $k++) { $script:buf[$k] = 0 }
    $got = $false

    foreach ($ln in $lines) {
        # 輸出格式：  0x20000070 : C30858F2 C1AEA4E2 4199DECD FF26FAAD
        if ($ln -match '^\s*0x([0-9A-Fa-f]{8})\s*:\s*([0-9A-Fa-f ]+?)\s*$') {
            $addr = [long][Convert]::ToUInt32($Matches[1], 16)
            $i = 0
            foreach ($wd in ($Matches[2] -split '\s+' | Where-Object { $_ })) {
                $w = [Convert]::ToUInt32($wd, 16)
                # CubeProgrammer 印的是「那個 word 的數值」，所以最低位址的
                # byte 對應最低 byte。這個位元組序在讀 uwTick 時驗證過。
                for ($b = 0; $b -lt 4; $b++) {
                    $idx = [int]($addr - $BASE) + $i * 4 + $b
                    if ($idx -ge 0 -and $idx -lt $LEN) {
                        $script:buf[$idx] = [byte](($w -shr (8 * $b)) -band 0xFF)
                    }
                }
                $i++
            }
            $got = $true
        }
    }

    if (-not $got) {
        throw "讀不到 RAM。CubeIDE 是不是正在 debug？先按紅色 Terminate 再試。"
    }
}


# 這裡刻意「不做任何算術」，直接把 4 個 byte 丟給 BitConverter。
# 原因：PowerShell 5.1 把 0xC30864AB 這種 hex 字面值當成「負的 Int32」，
# 而且 195 -shl 24 也會 Int32 溢位變成負數，於是 [uint32] 轉型會丟
# "Value was either too large or too small for a UInt32"。
# 這個錯誤只發生在「最高位元是 1」的數值上 —— 也就是負的角度幾乎全中，
# 正的角度卻沒事，非常容易漏掉。BitConverter 直接吃 byte[]，繞開整個問題。
function F32($off) {
    return [BitConverter]::ToSingle($script:buf, $off)
}

function U32($off) {
    return [BitConverter]::ToUInt32($script:buf, $off)
}


function Show-Bar([double]$deg) {
    $w = 30
    $c = [int][Math]::Floor(($w - 1) / 2)          # 中間 = 0 度
    $p = [int][Math]::Round(($deg + 180.0) / 360.0 * ($w - 1))
    $p = [Math]::Max(0, [Math]::Min($w - 1, $p))
    $s = ''
    for ($i = 0; $i -lt $w; $i++) {
        if ($i -eq $p)     { $s += '#' }
        elseif ($i -eq $c) { $s += '|' }
        else               { $s += '.' }
    }
    return $s
}


$prevFrames = -1
$prevRoll   = $null
$iter       = 0

while ($true) {
    $iter++
    try {
        Read-Block

        $roll   = F32 $OFF_ROLL
        $pitch  = F32 $OFF_PITCH
        $yaw    = F32 $OFF_YAW
        $frames = U32 $OFF_FRAMES
        $bad    = U32 $OFF_BADSUM
        $skip   = U32 $OFF_SKIP
        $rxb    = U32 $OFF_RXBYTE
        $rxe    = U32 $OFF_RXERR

        # frames 沒有增加 = 程式死了或卡住，此時角度是殘值，不可信
        $health = ''
        if ($prevFrames -ge 0 -and $frames -le $prevFrames) {
            $health = '  <<< frames 沒有增加，程式可能停了'
        }
        $prevFrames = $frames

        # 格式是 {索引:正數格式;負數格式}。
        # 不能寫成 {0,+.1} —— 逗號後面是「欄位寬度」，必須是整數，
        # 寫成 +.1 會丟 FormatException: 輸入字串的格式不正确。
        # 而且它只在第二圈之後才會被求值（第一圈 $prevRoll 還是 $null），
        # 所以症狀是「先正常顯示一次，然後才爆」。
        $delta = ''
        if ($null -ne $prevRoll) { $delta = "  (變化 {0:+0.0;-0.0} deg)" -f ($roll - $prevRoll) }
        $prevRoll = $roll

        # 對應韌體裡的 imu_link_code()。
        # frames=0 時角度欄位是 0.0，而 0.0 正是「水平」的讀數 —— 一條從沒送過
        # byte 的線，在這裡長得跟「量到水平」一模一樣。所以要把連結狀態講明，
        # 不能讓 0.0 冒充一個健康讀數。
        $link = ''
        if ($frames -gt 0)  { $link = 'OK       frame 正常解析' }
        elseif ($rxb -eq 0) { $link = 'code 1   PA10 一個 byte 都沒收到 -> 查 TX->PA10 / 3V3 / 共地' }
        elseif ($rxe -gt 0) { $link = 'code 2   有 byte 但 UART 報錯 -> 先查波特率' }
        else                { $link = 'code 3   byte 有進來且無錯誤，但解不出 frame -> 查幀格式' }

        Clear-Host
        Write-Host ("  IMU 即時角度     frames={0}   badsum={1}   skipped={2}{3}" -f $frames, $bad, $skip, $health)
        Write-Host ("  連結  rx_bytes={0}   rx_errs={1}   {2}" -f $rxb, $rxe, $link)
        Write-Host "  ------------------------------------------------------------------"
        Write-Host ("  roll  {0,8:N1} deg  {1}{2}" -f $roll,  (Show-Bar $roll),  $delta)
        Write-Host ("  pitch {0,8:N1} deg  {1}"    -f $pitch, (Show-Bar $pitch))
        Write-Host ("  yaw   {0,8:N1} deg  {1}"    -f $yaw,   (Show-Bar $yaw))
        Write-Host ""
        Write-Host "        -180                    0                    +180"

        if ($Count -gt 0 -and $iter -ge $Count) { break }
        Write-Host "        Ctrl+C 結束"
    }
    catch {
        Write-Host ""
        Write-Host ("  " + $_.Exception.Message) -ForegroundColor Red
        break
    }
}
