param(
    [string]$Port = 'COM9'
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
    ($clean -split "`r?`n") | Where-Object { $_ -match '\S' } | ForEach-Object { $_.TrimEnd() }
}

Write-Output '##### silence text links #####'
Send-Cmd 'gins_fused_data off' 400 | Out-Null
Send-Cmd 'magout off' 400 | Out-Null
Send-Cmd 'gnssout off' 400 | Out-Null
Send-Cmd 'barout off' 400 | Out-Null
Send-Cmd 'vofa log off' 400 | Out-Null

Write-Output '##### gins #####'
Send-Cmd 'gins' 1500
Write-Output '##### magdata #####'
Send-Cmd 'magdata' 800
Write-Output '##### ps (data threads) #####'
Send-Cmd 'ps' 1200 | Where-Object { $_ -match 'gins|mag|baro|vofa|gnss|um982|finsh|tshell|main' }
$p.Close()
