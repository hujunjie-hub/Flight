$c = New-Object Net.Sockets.TcpClient('127.0.0.1', 4444)
$s = $c.GetStream(); $s.ReadTimeout = 1500
Start-Sleep -Milliseconds 300
try { $null = $s.Read((New-Object byte[] 4096), 0, 4096) } catch {}
$b = [Text.Encoding]::ASCII.GetBytes("reset run`n")
$s.Write($b, 0, $b.Length)
Start-Sleep -Milliseconds 500
$c.Close()
