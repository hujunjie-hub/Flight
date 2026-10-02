# SWD dump: SPI1 + GPIOA/GPIOC 寄存器 (ADIS 探测诊断)
$c = New-Object Net.Sockets.TcpClient('127.0.0.1', 4444)
$s = $c.GetStream(); $s.ReadTimeout = 2500
try { $null = $s.Read((New-Object byte[] 4096), 0, 4096) } catch {}
$cmds = @(
  'mdw 0x58020000 2',   # GPIOA MODER/OTYPER (PA5=SCK PA6=MISO PA7=MOSI)
  'mdw 0x58020024 1',   # GPIOA OSPEEDR (部分)
  'mdw 0x58020020 2',   # GPIOA AFRL/AFRH
  'mdw 0x58020800 2',   # GPIOC MODER/OTYPER (PC4=CS PC5=RST)
  'mdw 0x40013000 4',   # SPI1 CR1/CR2/SR/DR
  'mdw 0x58024428 1'    # RCC APB2LPENR? 占位
)
$out = ''
foreach ($cmd in $cmds) {
  $b = [Text.Encoding]::ASCII.GetBytes($cmd + "`n")
  $s.Write($b, 0, $b.Length)
  Start-Sleep -Milliseconds 250
  try {
    $buf = New-Object byte[] 4096
    $n = $s.Read($buf, 0, 4096)
    $out += [Text.Encoding]::ASCII.GetString($buf, 0, $n)
  } catch {}
}
$c.Close()
Write-Output $out
