# SWD: 连续读 GPIO IDR (线上真实电平) — ADIS 取证
# GPIOA: PA5=SCK PA6=MISO PA7=MOSI ; GPIOC: PC4=CS PC5=RST ; DR=EXTI4=PE4
$c = New-Object Net.Sockets.TcpClient('127.0.0.1', 4444)
$s = $c.GetStream(); $s.ReadTimeout = 2500
try { $null = $s.Read((New-Object byte[] 4096), 0, 4096) } catch {}
$out = ''
foreach ($i in 1..8) {
  $b = [Text.Encoding]::ASCII.GetBytes("mdw 0x58020010 1`nmdw 0x58020810 1`nmdw 0x58021010 1`n")
  $s.Write($b, 0, $b.Length)
  Start-Sleep -Milliseconds 60
  try {
    $buf = New-Object byte[] 4096
    $n = $s.Read($buf, 0, 4096)
    $out += "[$i] " + ([Text.Encoding]::ASCII.GetString($buf, 0, $n) -replace "`n", ' ' -replace 'mdw', '') + "`n"
  } catch {}
}
$c.Close()
Write-Output $out
