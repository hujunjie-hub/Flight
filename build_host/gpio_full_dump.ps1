# 全 GPIO 配置 dump: 按 Flight.xlsx 引脚表核对 5.3.0 固件运行态
# GPIOA=0x58020000 GPIOC=0x58020800 GPIOE=0x58021000 GPIOF=0x58021400 GPIOG=0x58021800
# 每端口: MODER(+0x00) PUPDR(+0x0C) AFRL(+0x20) AFRH(+0x24)
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
Cmd 'mdw 0x58020000 4'    # A: MODER OTYPER OSPEEDR PUPDR
Cmd 'mdw 0x58020020 2'    # A: AFRL AFRH
Cmd 'mdw 0x58020800 4'    # C
Cmd 'mdw 0x58020820 2'
Cmd 'mdw 0x58021000 4'    # E
Cmd 'mdw 0x58021020 2'
Cmd 'mdw 0x58021400 4'    # F (W25Q64 OCTOSPI1: PF6/7/8/9/10)
Cmd 'mdw 0x58021420 2'
Cmd 'mdw 0x58021800 4'    # G (PG6 NCS, PG7 LED)
Cmd 'mdw 0x58021820 2'
Cmd 'resume'
$c.Close()
Write-Output $out
