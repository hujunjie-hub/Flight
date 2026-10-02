param(
    [string]$Port = 'COM9',
    [int]$SampleSec = 12
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
        $_ -notmatch '^(time:|natt )' -and $_ -notmatch 'rmag:' -and $_ -match '\S'
    } | ForEach-Object { $_.TrimEnd() }
}

Write-Output "===== TEST @460800 (firmware default) ====="
Send-Cmd 'gnss baud 460800' 500 | Out-Null
Send-Cmd 'gnss reset' 500 | Out-Null
Start-Sleep -Seconds $SampleSec
Write-Output "--- gnss (12s counters) ---"
Send-Cmd 'gnss' 800
Write-Output "--- gnss raw (last 64 bytes) ---"
Send-Cmd 'gnss raw' 800
Write-Output "--- um982 ---"
Send-Cmd 'um982' 1200
Write-Output "--- gnssdata ---"
Send-Cmd 'gnssdata' 800
$p.Close()
