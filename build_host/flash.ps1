$c = New-Object Net.Sockets.TcpClient('127.0.0.1', 4444)
$s = $c.GetStream(); $s.ReadTimeout = 3000
try { $null = $s.Read((New-Object byte[] 4096), 0, 4096) } catch {}
$b = [Text.Encoding]::ASCII.GetBytes("program D:/STM32Project/Flight/cmake-build-debug/Flight.elf verify" + "`n")
$s.Write($b, 0, $b.Length)
$sw = [Diagnostics.Stopwatch]::StartNew()
while ($sw.Elapsed.TotalSeconds -lt 45) {
  try {
    $buf = New-Object byte[] 8192
    $n = $s.Read($buf, 0, 8192)
    Write-Output ([Text.Encoding]::ASCII.GetString($buf, 0, $n))
  } catch { Start-Sleep -Milliseconds 300 }
}
$c.Close()
