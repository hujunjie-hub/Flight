$c = New-Object Net.Sockets.TcpClient('127.0.0.1', 4444)
$s = $c.GetStream()
$s.ReadTimeout = 1500
Start-Sleep -Milliseconds 300
try { $null = $s.Read((New-Object byte[] 4096), 0, 4096) } catch {}
$cmd = 'resume'
$b = [Text.Encoding]::ASCII.GetBytes($cmd + "`n")
$s.Write($b, 0, $b.Length)
Start-Sleep -Milliseconds 800
$buf = New-Object byte[] 4096
try {
    $n = $s.Read($buf, 0, 4096)
    Write-Output ([Text.Encoding]::ASCII.GetString($buf, 0, $n))
} catch { Write-Output "no reply" }
$c.Close()
