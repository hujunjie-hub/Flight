# ADIS 总线硬件级验证 v2: H7 SPI 主模式每字节 TXDR -> CR1.CSTART -> SR -> RXDR
# 固件 halt, 纯 SWD 直控。期望 (芯片在): 第 3/4 字节 RXDR = 0x40 0x79
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
Cmd 'mdw 0x58020814 1'             # GPIOC ODR backup
Cmd 'mww 0x40013000 0x00001001'   # SPE=1 (keep MasterKeepIOState bit12)
Cmd 'mww 0x58020814 0x00000020'   # CS low
Cmd 'mdw 0x58020010 1'             # GPIOA IDR (MISO PA6, CS active)

foreach ($byte in @(0x72, 0x00, 0x00, 0x00, 0x00)) {
  Cmd ('mww 0x40013020 0x{0:x8}' -f $byte)     # TXDR
  Cmd 'mww 0x40013000 0x00001201'              # CSTART pulse (SPE|CSTART)
  Cmd 'mdw 0x40013014 1'                       # SR
  Cmd 'mdw 0x40013030 1'                       # RXDR
  Cmd 'mww 0x40013018 0x00000008'              # IFCR clear EOT
}

Cmd 'mdw 0x58020010 1'             # MISO after
Cmd 'mww 0x58020814 0x00000030'   # CS high
Cmd 'mww 0x40013000 0x00001000'   # SPE=0 restore
Cmd 'resume'
$c.Close()
Write-Output $out
