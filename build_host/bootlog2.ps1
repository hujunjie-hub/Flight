param(
    [string]$Port = 'COM9',
    [int]$Seconds = 5
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
($clean -split "`n") | Where-Object { $_ -match 'Flight|vofa:|gnssout:|magout:|ginsout:|imuout:|main|aligned|assert|OOM|I/gins|E/' } | Select-Object -First 20
