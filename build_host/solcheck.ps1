param(
    [string]$Port = 'COM9',
    [int]$Seconds = 8
)
$p = New-Object System.IO.Ports.SerialPort($Port, 460800, 'None', 8, 'One')
$p.ReadTimeout = 300
try { $p.Open() } catch { Write-Output ('OPEN FAIL: ' + $_.Exception.Message); exit 1 }
$sw = [Diagnostics.Stopwatch]::StartNew()
$sb = New-Object System.Text.StringBuilder
while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    try { $null = $sb.Append($p.ReadExisting()) } catch {}
    Start-Sleep -Milliseconds 50
}
$p.Close()
$lines = @(($sb.ToString() -split "`n") | Where-Object { $_ -match 'ready:1' })
Write-Output ('ready:1 lines in window: ' + $lines.Count)
$lines | Select-Object -Last 3 | ForEach-Object { ($_ -replace '[^\x20-\x7E]', '').TrimEnd("`r") }
