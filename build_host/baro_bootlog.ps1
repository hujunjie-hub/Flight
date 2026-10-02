param(
    [string]$Port = 'COM9',
    [int]$BootSeconds = 8
)
# 重启板子并抓启动日志: 观察 BMP585 初始化输出 (探测/软复位/NVM)
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
    Write-Output ("===== " + $cmd + " =====")
    ($clean -split "`r?`n") | Where-Object {
        $_ -match '\S' -and $_ -notmatch '^\?'
    } | ForEach-Object { $_.TrimEnd() }
}

Send-Cmd 'vofa log off' 600
Send-Cmd 'reboot' 500

# 抓启动日志
$sw = [Diagnostics.Stopwatch]::StartNew()
$sb = New-Object System.Text.StringBuilder
while ($sw.Elapsed.TotalSeconds -lt $BootSeconds) {
    try { $null = $sb.Append($p.ReadExisting()) } catch {}
    Start-Sleep -Milliseconds 50
}
$p.Close()
$t = $sb.ToString()
$clean = ($t -replace '[^\x20-\x7E\r\n]', '')
Write-Output "===== BOOT LOG ====="
($clean -split "`r?`n") | Where-Object {
    $_ -match '\S' -and $_ -notmatch '^\?'
} | ForEach-Object { $_.TrimEnd() }
