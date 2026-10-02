$c = New-Object Net.Sockets.TcpClient('127.0.0.1', 4444)
$s = $c.GetStream(); $s.ReadTimeout = 2500
try { $null = $s.Read((New-Object byte[] 8192), 0, 8192) } catch {}
foreach ($cmd in @('adapter speed', 'mdw 0x08000000 8', 'mdw 0x24000000 8', 'targets')) {
  $b = [Text.Encoding]::ASCII.GetBytes($cmd + "`n")
  $s.Write($b, 0, $b.Length)
  Start-Sleep -Milliseconds 600
  try {
    $buf = New-Object byte[] 8192
    $n = $s.Read($buf, 0, 8192)
    Write-Output ([Text.Encoding]::ASCII.GetString($buf, 0, $n))
  } catch { Write-Output ("no reply: " + $cmd) }
  Write-Output "----"
}
# read-only full-image stress (~240KB over DAP @10MHz)
$b = [Text.Encoding]::ASCII.GetBytes("verify_image D:/STM32Project/Flight/cmake-build-debug/Flight.elf`n")
$s.Write($b, 0, $b.Length)
$sw = [Diagnostics.Stopwatch]::StartNew()
while ($sw.Elapsed.TotalSeconds -lt 40) {
  try {
    $buf = New-Object byte[] 8192
    $n = $s.Read($buf, 0, 8192)
    Write-Output ([Text.Encoding]::ASCII.GetString($buf, 0, $n))
  } catch { Start-Sleep -Milliseconds 300 }
}
Write-Output "---- verify done"
$b = [Text.Encoding]::ASCII.GetBytes("targets`n")
$s.Write($b, 0, $b.Length)
Start-Sleep -Milliseconds 800
try {
  $buf = New-Object byte[] 8192
  $n = $s.Read($buf, 0, 8192)
  Write-Output ([Text.Encoding]::ASCII.GetString($buf, 0, $n))
} catch {}
$c.Close()
