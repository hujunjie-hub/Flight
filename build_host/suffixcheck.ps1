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
    Start-Sleep -Milliseconds 30
}
$p.Close()
$clean = ($sb.ToString() -replace '[^\x20-\x7E\r\n]', '')
$mag  = @(($clean -split "`n") | Where-Object { $_ -match 'mag_calib_data' })
$fuse = @(($clean -split "`n") | Where-Object { $_ -match 'fused_data' })
$baro = @(($clean -split "`n") | Where-Object { $_ -match 'baro_calib_data' })
Write-Output ("mag_calib_data: " + $mag.Count + "   fused_data: " + $fuse.Count + "   baro_calib_data: " + $baro.Count)
Write-Output '--- mag sample ---'
$mag | Select-Object -Last 1
Write-Output '--- fused sample ---'
$fuse | Select-Object -Last 1
