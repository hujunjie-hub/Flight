param(
    [string]$Port = 'COM9',
    [int]$Seconds = 60
)
$p = New-Object System.IO.Ports.SerialPort($Port, 460800, 'None', 8, 'One')
$p.ReadTimeout = 200
try { $p.Open() } catch { Write-Output ('OPEN FAIL: ' + $_.Exception.Message); exit 1 }
$sw = [Diagnostics.Stopwatch]::StartNew()
$sb = New-Object System.Text.StringBuilder
while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    try { $null = $sb.Append($p.ReadExisting()) } catch {}
    Start-Sleep -Milliseconds 50
}
$p.Close()
$t = $sb.ToString()
$lines = @($t -split "`n")
$gnss = @($lines | Where-Object { $_ -match 'fix:\d+ sats:' })
$mag  = @($lines | Where-Object { $_ -match 'rx:' })
$natt = @($lines | Where-Object { $_ -match 'natt ' })
Write-Output ("TOTAL: " + $t.Length + " chars, " + $lines.Count + " lines in " + $Seconds + "s")
Write-Output ("gnssout lines: " + $gnss.Count + " (~" + [math]::Round($gnss.Count / $Seconds, 2) + " Hz)")
Write-Output ("magout  lines: " + $mag.Count)
Write-Output ("vofa natt lines: " + $natt.Count)
Write-Output "--- last 3 gnssout lines ---"
$gnss | Select-Object -Last 3 | ForEach-Object { $_.TrimEnd("`r") }
