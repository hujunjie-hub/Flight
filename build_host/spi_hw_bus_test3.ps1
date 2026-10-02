# ADIS 总线硬件级验证 v3: TSIZE=5 批量传输 (H7 标准 TSIZE 模式, 一次 CSTART)
# 固件 halt, SWD 直控。数据帧 (第 3/4 字节) RXDR 应为 0x40 0x79 (PROD_ID)
$c = New-Object Net.Sockets.TcpClient('127.0.0.1', 4444)
$s = $c.GetStream(); $s.ReadTimeout = 3000
try { $null = $s.Read((New-Object byte[] 8192), 0, 8192) } catch {}

$out = ''
function Cmd($txt) {
  $b = [Text.Encoding]::ASCII.GetBytes($txt + "`n")
  $script:s.Write($b, 0, $b.Length)
  Start-Sleep -Milliseconds 260
  try {
    $buf = New-Object byte[] 8192
    $n = $s.Read($buf, 0, 8192)
    $script:out += "$txt ==> " + ([Text.Encoding]::ASCII.GetString($buf, 0, $n) -replace "`n", ' ' -replace "`r", '') + "`n"
  } catch { $script:out += "$txt ==> (timeout)`n" }
}

Cmd 'halt'
Cmd 'mdw 0x40013004 1'             # CR2 backup
Cmd 'mdw 0x58020814 1'             # GPIOC ODR backup
Cmd 'mww 0x40013004 0x00000005'   # CR2: TSIZE=5 bytes
Cmd 'mww 0x40013000 0x00001001'   # SPE=1
Cmd 'mww 0x58020814 0x00000020'   # CS low
Cmd 'mdw 0x58020010 1'             # MISO line level (CS active)
Cmd 'mww 0x40013020 0x00000072'   # TXDR[0]: read cmd 0x72
Cmd 'mww 0x40013020 0x00000000'   # TXDR[1]: 0x00
Cmd 'mww 0x40013020 0x00000000'   # TXDR[2]: dummy -> RX = PROD_ID MSB
Cmd 'mww 0x40013020 0x00000000'   # TXDR[3]: dummy -> RX = PROD_ID LSB
Cmd 'mww 0x40013020 0x00000000'   # TXDR[4]: spare
Cmd 'mww 0x40013000 0x00001201'   # CSTART (one-shot, TSIZE=5 auto burst)
Cmd 'mdw 0x40013014 1'             # SR (expect EOT)
Cmd 'mdw 0x58020010 1'             # MISO after burst
Cmd 'mdw 0x40013030 1'             # RXDR[0]
Cmd 'mdw 0x40013030 1'             # RXDR[1]
Cmd 'mdw 0x40013030 1'             # RXDR[2]  <- PROD_ID MSB?
Cmd 'mdw 0x40013030 1'             # RXDR[3]  <- PROD_ID LSB?
Cmd 'mdw 0x40013030 1'             # RXDR[4]
Cmd 'mww 0x58020814 0x00000030'   # CS high
Cmd 'mww 0x40013000 0x00001000'   # SPE=0
Cmd 'mww 0x40013004 0x00000002'   # CR2 restore
Cmd 'resume'
$c.Close()
Write-Output $out
