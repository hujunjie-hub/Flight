# ADIS 总线验证 v5: v4 修正 —— CS 高后立即读 RX FIFO, 读完才 SPE=0
# (SPE 1->0 会清空 FIFO, v4 因此误读全零)
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
Cmd 'mww 0x40013004 0x00000002'   # TSIZE=2
Cmd 'mww 0x40013018 0x000000F8'   # clear flags
Cmd 'mww 0x40013000 0x00001001'   # SPE=1
Cmd 'mww 0x58020814 0x00000020'   # CS low
# 帧1: 0x72 0x00
Cmd 'mww 0x40013020 0x00000072'
Cmd 'mww 0x40013020 0x00000000'
Cmd 'mww 0x40013000 0x00001201'   # CSTART
Cmd 'mww 0x40013018 0x00000008'   # clear EOT
# stall (telnet ~200ms)
# 帧2: 0x00 0x00
Cmd 'mww 0x40013020 0x00000000'
Cmd 'mww 0x40013020 0x00000000'
Cmd 'mww 0x40013000 0x00001201'   # CSTART
Cmd 'mww 0x58020814 0x00000030'   # CS high
# ---- 立即读 FIFO (此时 SPE 仍 =1) ----
Cmd 'mdw 0x40013014 1'             # SR
Cmd 'mdw 0x40013030 1'             # RXDR (帧1 byte1)
Cmd 'mdw 0x40013030 1'             # RXDR (帧1 byte2)
Cmd 'mdw 0x40013030 1'             # RXDR (帧2 byte1 = PROD_ID MSB?)
Cmd 'mdw 0x40013030 1'             # RXDR (帧2 byte2 = PROD_ID LSB?)
# 恢复
Cmd 'mww 0x40013000 0x00001000'   # SPE=0 (FIFO 此后清空无所谓)
Cmd 'resume'
$c.Close()
Write-Output $out
