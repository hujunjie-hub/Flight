# 完整流程: 烧录 -> verify -> reset run -> 等 boot -> 串口 adisdbg reg -> 收结果
$ErrorActionPreference = 'Continue'

# 1. 烧录 (OpenOCD telnet, 等 banner)
$c = New-Object Net.Sockets.TcpClient('127.0.0.1', 4444)
$s = $c.GetStream(); $s.ReadTimeout = 4000
Start-Sleep -Milliseconds 500
try { $null = $s.Read((New-Object byte[] 8192), 0, 8192) } catch {}
$b = [Text.Encoding]::ASCII.GetBytes("program D:/STM32Project/Flight/cmake-build-debug/Flight.elf verify`n")
$s.Write($b, 0, $b.Length)
$prog = ''
$sw = [Diagnostics.Stopwatch]::StartNew()
while ($sw.Elapsed.TotalSeconds -lt 60) {
  try {
    $buf = New-Object byte[] 8192
    $n = $s.Read($buf, 0, 8192)
    $prog += [Text.Encoding]::ASCII.GetString($buf, 0, $n)
    if ($prog -match '\*\* Verified OK \*\*') { break }
  } catch { Start-Sleep -Milliseconds 400 }
}
if ($prog -match '\*\* Verified OK \*\*') { Write-Output 'FLASH: Verified OK' }
else { Write-Output ("FLASH FAILED: " + $prog.Substring([Math]::Max(0,$prog.Length-200))) }

# 2. reset run
$b2 = [Text.Encoding]::ASCII.GetBytes("reset run`n")
$s.Write($b2, 0, $b2.Length)
Start-Sleep -Seconds 1
$c.Close()
Write-Output 'RESET: sent'

# 3. 等 boot + 采集
Start-Sleep -Seconds 10
$p = New-Object IO.Ports.SerialPort('COM9',460800,'None',8,'One')
$p.ReadTimeout = 500
$p.Open()
Start-Sleep -Milliseconds 300
$p.DiscardInBuffer()
$p.Write("adisdbg reg`r`n")
Start-Sleep -Seconds 10
$r = $p.ReadExisting()
$p.Close()
[IO.File]::WriteAllBytes('D:\STM32Project\Flight\build_host\data\regtest2.bin',
                         [Text.Encoding]::UTF8.GetBytes($r))
Write-Output ("SERIAL: captured " + $r.Length + " chars")
