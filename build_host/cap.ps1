param(
    [string]$Port = 'COM9',
    [int]$Seconds = 15
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
Write-Output ('TOTAL CHARS: ' + $t.Length)
$lines = @($t -split "`n" | Where-Object { $_ -match '[!-~]' })
Write-Output ('TEXT LINES: ' + $lines.Count)
$lines | Select-Object -First 40 | ForEach-Object { $_.TrimEnd("`r") }
