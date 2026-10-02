$c = New-Object Net.Sockets.TcpClient('127.0.0.1', 4444)
$s = $c.GetStream(); $s.ReadTimeout = 2000
try { $null = $s.Read((New-Object byte[] 4096), 0, 4096) } catch {}
foreach ($cmd in @('reset halt', 'program D:/STM32Project/Flight/cmake-build-debug/Flight.elf verify', 'reset run')) {
  $b = [Text.Encoding]::ASCII.GetBytes($cmd + "`n")
  $s.Write($b, 0, $b.Length)
  $sw = [Diagnostics.Stopwatch]::StartNew()
  $wait = if ($cmd -like 'program*') { 60 } else { 5 }
  while ($sw.Elapsed.TotalSeconds -lt $wait) {
    try {
      $buf = New-Object byte[] 8192
      $n = $s.Read($buf, 0, 8192)
      Write-Output ([Text.Encoding]::ASCII.GetString($buf, 0, $n))
    } catch { Start-Sleep -Milliseconds 300 }
  }
  Write-Output ("---- done: " + $cmd)
}
$c.Close()
