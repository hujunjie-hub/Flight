param([int]$N = 40, [int]$WaitMs = 400)
$p = New-Object System.IO.Ports.SerialPort('COM9', 460800, 'None', 8, 'One')
$p.ReadTimeout = 300
try { $p.Open() } catch { Write-Output ('OPEN FAIL: ' + $_.Exception.Message); exit 1 }
Start-Sleep -Milliseconds 300
$null = $p.ReadExisting()
$p.Write("`r`n")
Start-Sleep -Milliseconds 400
$null = $p.ReadExisting()
for ($i = 0; $i -lt $N; $i++) {
    $p.Write("imudata`r`n")
    Start-Sleep -Milliseconds $WaitMs
    $resp = $p.ReadExisting()
    $m = [regex]::Match($resp, 'gyro  : (-?[\d.]+) (-?[\d.]+) (-?[\d.]+) rad/s')
    if ($m.Success) { Write-Output ($m.Groups[1].Value + ' ' + $m.Groups[2].Value + ' ' + $m.Groups[3].Value) }
}
$p.Close()
