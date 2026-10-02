param(
    [string]$Port = 'COM9',
    [int]$WaitMs = 1500
)
$p = New-Object System.IO.Ports.SerialPort($Port, 460800, 'None', 8, 'One')
$p.ReadTimeout = 200
try { $p.Open() } catch { Write-Output ('OPEN FAIL: ' + $_.Exception.Message); exit 1 }

function Send-Cmd([string]$cmd, [int]$waitMs) {
    $script:p.Write("`r`n")
    Start-Sleep -Milliseconds 200
    $null = $script:p.ReadExisting()
    $script:p.Write($cmd + "`r`n")
    Start-Sleep -Milliseconds $waitMs
    $resp = $script:p.ReadExisting()
    $clean = ($resp -replace '[^\x20-\x7E\r\n]', '')
    Write-Output ("===== " + $cmd + " =====")
    ($clean -split "`r?`n") | Where-Object { $_ -match '\S' } | ForEach-Object { $_.TrimEnd() }
}

Send-Cmd 'help' 800 | Select-Object -First 8
Send-Cmd 'ps' 800 | Select-Object -First 40
$p.Close()
