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
    ($clean -split "`r?`n") | Where-Object {
        $_ -notmatch '^(time:|natt |ready:)' -and $_ -notmatch 'rmag:' -and $_ -notmatch 'baro_calib_data' -and $_ -match '\S'
    } | ForEach-Object { $_.TrimEnd() }
}

Send-Cmd 'gins' $WaitMs
Send-Cmd 'gnss' 800
Send-Cmd 'ps' 800
$p.Close()
