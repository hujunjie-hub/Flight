param(
    [string]$Port = 'COM9',
    [int]$Seconds = 22
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
$lines = @(($sb.ToString() -split "`n") | Where-Object { $_ -match 'ready:' })
$gnss  = @(($sb.ToString() -split "`n") | Where-Object { $_ -match 'fix:' })
Write-Output ('ginsout lines: ' + $lines.Count + '  gnssout lines: ' + $gnss.Count)
Write-Output '--- ginsout samples ---'
$lines | Select-Object -First 6 | ForEach-Object { ($_ -replace '[^\x20-\x7E]','').TrimEnd("`r") }
Write-Output '--- gnssout sample ---'
$gnss | Select-Object -First 1 | ForEach-Object { ($_ -replace '[^\x20-\x7E]','').TrimEnd("`r") }
