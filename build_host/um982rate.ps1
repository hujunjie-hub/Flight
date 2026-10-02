param(
    [string]$Port = 'COM9',
    [int]$GapSec = 10
)
$p = New-Object System.IO.Ports.SerialPort($Port, 460800, 'None', 8, 'One')
$p.ReadTimeout = 200
try { $p.Open() } catch { Write-Output ('OPEN FAIL: ' + $_.Exception.Message); exit 1 }

function Send-Cmd([string]$cmd, [int]$waitMs) {
    $script:p.Write("`r`n")
    Start-Sleep -Milliseconds 300
    $null = $script:p.ReadExisting()
    $script:p.Write($cmd + "`r`n")
    Start-Sleep -Milliseconds $waitMs
    $resp = $script:p.ReadExisting()
    $clean = ($resp -replace '[^\x20-\x7E\r\n]', '')
    ($clean -split "`r?`n") | Where-Object {
        $_ -notmatch '^(time:|natt |ready:)' -and $_ -notmatch 'rmag:' -and $_ -match '\S'
    } | ForEach-Object { $_.TrimEnd() }
}

Write-Output "--- um982 (t=0) ---"
Send-Cmd 'um982' 1500
Start-Sleep -Seconds $GapSec
Write-Output "--- um982 (t=+10s, 看增量) ---"
Send-Cmd 'um982' 1500
Write-Output "--- gnss raw (当前语句内容) ---"
Send-Cmd 'gnss raw' 800
$p.Close()
