param(
    [string]$Port = 'COM9'
)
# BMP585 状态检测: barodata 两次采样间隔看 pushed 增速, list_device 看注册
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
        $_ -match '\S' -and
        $_ -notmatch '^(time:|ready:|natt )' -and
        $_ -notmatch 'fused_data|mag_calib_data|baro_calib_data$'
    } | ForEach-Object { $_.TrimEnd() }
}

Send-Cmd 'barodata' 800
Start-Sleep -Seconds 3          # 100Hz x 3s ≈ +300 pushed 若采集正常
Send-Cmd 'barodata' 800
Send-Cmd 'list_device' 800
Send-Cmd 'gins' 800
$p.Close()
