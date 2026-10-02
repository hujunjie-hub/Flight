param(
    [string]$Port = 'COM9'
)
$p = New-Object System.IO.Ports.SerialPort($Port, 460800, 'None', 8, 'One')
$p.ReadTimeout = 200
try { $p.Open() } catch { Write-Output ('OPEN FAIL: ' + $_.Exception.Message); exit 1 }

function Send-Cmd([string]$cmd, [int]$waitMs) {
    $script:p.Write("`r`n")
    Start-Sleep -Milliseconds 250
    $null = $script:p.ReadExisting()
    $script:p.Write($cmd + "`r`n")
    Start-Sleep -Milliseconds $waitMs
    $resp = $script:p.ReadExisting()
    $clean = ($resp -replace '[^\x20-\x7E\r\n]', '')
    ($clean -split "`r?`n") | Where-Object {
        $_ -notmatch '^(time:|natt |ready:)' -and $_ -notmatch 'rmag:' -and $_ -match '\S'
    } | ForEach-Object { $_.TrimEnd() }
}

foreach ($b in @('921600', '115200', '230400', '57600', '38400', '19200')) {
    Write-Output ("##### TRY " + $b + " #####")
    Send-Cmd ('gnss baud ' + $b) 400 | Out-Null
    Start-Sleep -Milliseconds 2600
    Send-Cmd 'gnss raw' 600 | Where-Object { $_ -match '\||raw bytes' } | Select-Object -First 5
}
$p.Close()
