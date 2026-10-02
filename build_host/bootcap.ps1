param(
    [string]$Port = 'COM9',
    [int]$Seconds = 14
)
$p = New-Object System.IO.Ports.SerialPort($Port, 460800, 'None', 8, 'One')
$p.ReadTimeout = 100
try { $p.Open() } catch { Write-Output ('OPEN FAIL: ' + $_.Exception.Message); exit 1 }

# OpenOCD 复位目标 (板子重新启动, 抓完整 boot log)
$c = New-Object Net.Sockets.TcpClient('127.0.0.1', 4444)
$s = $c.GetStream(); $s.ReadTimeout = 1500
try { $null = $s.Read((New-Object byte[] 4096), 0, 4096) } catch {}
$b = [Text.Encoding]::ASCII.GetBytes("reset run" + "`n")
$s.Write($b, 0, $b.Length)
Start-Sleep -Milliseconds 400
try { $null = $s.Read((New-Object byte[] 4096), 0, 4096) } catch {}
$c.Close()

$sw = [Diagnostics.Stopwatch]::StartNew()
$sb = New-Object System.Text.StringBuilder
while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    try { $null = $sb.Append($p.ReadExisting()) } catch {}
    Start-Sleep -Milliseconds 20
}
$p.Close()
$clean = ($sb.ToString() -replace '[^\x20-\x7E\r\n]', '')
($clean -split "`n") | Where-Object { $_ -match '\S' } |
    ForEach-Object { $_.TrimEnd("`r") } | Select-Object -First 60
