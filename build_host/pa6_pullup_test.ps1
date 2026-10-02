# MISO(PA6) 输入路径电气诊断:
# 1. dump GPIOA 全配置 (MODER/OTYPER/OSPEEDR/PUPDR/IDR/ODR/AFRL/AFRH)
# 2. PA6 加内部上拉 (PUPDR[13:12]=01) 后读 IDR:
#    - IDR bit6 变 1 -> 线悬空/高阻 (AF 输入连到的点无驱动) -> 指向断线或芯片高阻
#    - IDR bit6 仍 0 -> 线被外部强拉到地 (短路/外部下拉) -> 查短路
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
Cmd 'mdw 0x58020000 10'              # MODER..AFRH (MODER OTYPER OSPEEDR PUPDR IDR ODR .. AFRL AFRH)
Cmd 'mdw 0x5802000C 1'               # PUPDR
Cmd 'mww 0x5802000C 0x00001800'      # PA6 pull-up: bits[13:12]=01 -> 0x1000 (only PA6)
Cmd 'mdw 0x58020010 1'               # IDR with pull-up
Cmd 'mdw 0x58020010 1'
Cmd 'mww 0x5802000C 0x00000000'      # PUPDR restore none
Cmd 'mdw 0x58020010 1'               # IDR after restore
Cmd 'resume'
$c.Close()
Write-Output $out
