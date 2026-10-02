param(
    [string]$Port = 'COM9',
    [int]$Seconds = 10
)
$p = New-Object System.IO.Ports.SerialPort($Port, 460800, 'None', 8, 'One')
$p.ReadTimeout = 300
try { $p.Open() } catch { Write-Output ('OPEN FAIL: ' + $_.Exception.Message); exit 1 }
$sw = [Diagnostics.Stopwatch]::StartNew()
$sb = New-Object System.Text.StringBuilder
while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    try { $null = $sb.Append($p.ReadExisting()) } catch {}
    Start-Sleep -Milliseconds 30
}
$p.Close()
$clean = ($sb.ToString() -replace '[^\x20-\x7E\r\n]', '')
$lines = @(($clean -split "`n") | Where-Object { $_ -match '\S' } |
    ForEach-Object { $_.TrimEnd("`r") })
Write-Output ('total lines: ' + $lines.Count)
Write-Output ('ready lines: ' + @(($lines | Where-Object { $_ -match 'ready:' })).Count)
Write-Output ('vofa log/natt: ' + @(($lines | Where-Object { $_ -match 'natt' })).Count)
Write-Output ('mag_calib lines: ' + @(($lines | Where-Object { $_ -match 'mag_calib_data' })).Count)
Write-Output '--- last 12 lines ---'
$lines | Select-Object -Last 12
