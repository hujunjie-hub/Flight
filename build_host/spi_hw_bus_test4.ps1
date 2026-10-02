# ADIS 总线验证 v4 (修正时序): 严格复刻驱动读时序
#   SPE=0 配 TSIZE=2 -> SPE=1 -> CS 低 -> 帧1(0x72,0x00)+CSTART+EOT
#   -> stall(telnet 间隔 ~260ms >> 16us) -> 帧2(0x00,0x00)+CSTART+EOT
#   -> CS 高 -> 逐字节读 RXDR (每步先查 SR.RXP, 空则停)
# 帧 2 的 RXDR 应为 0x40 0x79 (PROD_ID=0x4079)
$c = New-Object Net.Sockets.TcpClient('127.0.0.1', 4444)
$s = $c.GetStream(); $s.ReadTimeout = 3000
try { $null = $s.Read((New-Object byte[] 8192), 0, 8192) } catch {}
$out = ''
function Cmd($txt) {
  $b = [Text.Encoding]::ASCII.GetBytes($txt + "`n")
  $script:s.Write($b, 0, $b.Length)
  Start-Sleep -Milliseconds 200
  try {
    $buf = New-Object byte[] 8192
    $n = $s.Read($buf, 0, 8192)
    $script:out += "$txt ==> " + ([Text.Encoding]::ASCII.GetString($buf, 0, $n) -replace "`n", ' ' -replace "`r", '') + "`n"
  } catch { $script:out += "$txt ==> (timeout)`n" }
}

Cmd 'halt'
Cmd 'mww 0x40013000 0x00001000'   # SPE=0
Cmd 'mww 0x40013004 0x00000002'   # CR2 TSIZE=2
Cmd 'mww 0x40013018 0x000000F8'   # IFCR clear all flags
Cmd 'mww 0x40013000 0x00001001'   # SPE=1
Cmd 'mww 0x58020814 0x00000020'   # CS low
# ---- 帧1: 命令 0x72 0x00 ----
Cmd 'mww 0x40013020 0x00000072'
Cmd 'mww 0x40013020 0x00000000'
Cmd 'mww 0x40013000 0x00001201'   # CSTART
Cmd 'mdw 0x40013014 1'             # SR (EOT)
Cmd 'mww 0x40013018 0x00000008'   # clear EOT
# ---- stall: telnet 间隔即 ~200ms ----
# ---- 帧2: dummy 0x00 0x00 ----
Cmd 'mww 0x40013020 0x00000000'
Cmd 'mww 0x40013020 0x00000000'
Cmd 'mww 0x40013000 0x00001201'   # CSTART
Cmd 'mdw 0x40013014 1'             # SR
Cmd 'mww 0x58020814 0x00000030'   # CS high
Cmd 'mww 0x40013000 0x00001000'   # SPE=0
# ---- 读 RX FIFO (RXP=bit0) ----
Cmd 'mdw 0x40013014 1'
Cmd 'mdw 0x40013030 1'
Cmd 'mdw 0x40013014 1'
Cmd 'mdw 0x40013030 1'
Cmd 'mdw 0x40013014 1'
Cmd 'mdw 0x40013030 1'
Cmd 'mdw 0x40013014 1'
Cmd 'resume'
$c.Close()
Write-Output $out
