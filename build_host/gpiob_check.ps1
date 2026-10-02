# 查 GPIOB: PB3/PB4/PB5 (SPI1 的第二组 AF5 引脚) 是否被 5.3.0 drv_spi 配置
# 若 PB4(SPI1_MISO alt)=AF5 且悬空 -> 污染 SPI1 MISO 输入 -> 读恒 0
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
Cmd 'mdw 0x58020400 4'    # GPIOB MODER OTYPER OSPEEDR PUPDR
Cmd 'mdw 0x58020420 2'    # GPIOB AFRL AFRH
Cmd 'mdw 0x58020410 1'    # GPIOB IDR
Cmd 'resume'
$c.Close()
Write-Output $out
