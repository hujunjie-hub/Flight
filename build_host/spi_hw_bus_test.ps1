# ADIS 总线硬件级验证 (2026-10-01): 完全绕开固件, SWD 直控 SPI1 + CS
# halt CPU (adisretry 干扰隔离) -> SPE 使能 -> CS 低 -> MISO 线电平采样 ->
# 手动发 0x72 命令 + dummy -> RXDR 收数 -> MISO 复采样 -> 恢复 -> resume
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
Cmd 'mdw 0x40013000 1'     # CR1 backup
Cmd 'mdw 0x58020814 1'     # GPIOC ODR backup (CS=PC4)
Cmd 'mww 0x40013000 0x00001001'   # SPE=1
Cmd 'mdw 0x58020010 1'     # GPIOA IDR: MISO(PA6) before clock, CS still high
Cmd 'mww 0x58020814 0x00000020'   # CS low (ODR bit4 clear, keep RST PC5 high)
Cmd 'mdw 0x58020010 1'     # MISO after CS low (chip should drive it)
Cmd 'mww 0x40013020 0x00000072'   # TXDR: 0x72 (PROD_ID read cmd)
Cmd 'mdw 0x40013014 1'     # SR
Cmd 'mww 0x40013020 0x00000000'   # TXDR: 0x00 (cmd frame byte 2)
Cmd 'mww 0x40013020 0x00000000'   # TXDR: dummy (data frame byte 1)
Cmd 'mww 0x40013020 0x00000000'   # TXDR: dummy (data frame byte 2)
Cmd 'mww 0x40013020 0x00000000'   # TXDR: spare
Cmd 'mdw 0x40013014 1'     # SR (TXP/RXP)
Cmd 'mdw 0x58020010 1'     # MISO mid/after transfers
Cmd 'mdw 0x40013030 1'     # RXDR 1
Cmd 'mdw 0x40013030 1'     # RXDR 2
Cmd 'mdw 0x40013030 1'     # RXDR 3
Cmd 'mdw 0x40013030 1'     # RXDR 4
Cmd 'mww 0x58020814 0x00000030'   # CS high, RST high
Cmd 'mww 0x40013000 0x00001000'   # CR1 restore (SPE=0)
Cmd 'resume'
$c.Close()
Write-Output $out
