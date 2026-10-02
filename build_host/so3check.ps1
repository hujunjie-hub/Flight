param(
    [string]$Port = 'COM9',
    [string]$Cmd = 'so3',
    [int]$WaitMs = 2500
)
$p = New-Object System.IO.Ports.SerialPort($Port, 460800, 'None', 8, 'One')
$p.ReadTimeout = 200
try { $p.Open() } catch { Write-Output ('OPEN FAIL: ' + $_.Exception.Message); exit 1 }
Start-Sleep -Milliseconds 300
$null = $p.ReadExisting()
$p.Write($Cmd + "`r`n")
Start-Sleep -Milliseconds $WaitMs
$resp = $p.ReadExisting()
$p.Close()
$clean = ($resp -replace '[^\x20-\x7E\r\n]', '')
($clean -split "`r?`n") | Where-Object { $_ -match 'SO3|cur|target|angle|e_b|e_n|fused' } | Select-Object -First 8
