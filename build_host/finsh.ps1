param(
    [string]$Port = 'COM9',
    [string[]]$Commands = @('gnssout', 'gnssraw', 'gnssdata', 'um982')
)
$p = New-Object System.IO.Ports.SerialPort($Port, 460800, 'None', 8, 'One')
$p.ReadTimeout = 200
$p.NewLine = "`n"
try { $p.Open() } catch { Write-Output ('OPEN FAIL: ' + $_.Exception.Message); exit 1 }

# 清掉积压的流式数据
Start-Sleep -Milliseconds 300
$null = $p.ReadExisting()

# 唤醒 FinSH 提示符
$p.Write("`r`n")
Start-Sleep -Milliseconds 800
$null = $p.ReadExisting()

foreach ($cmd in $Commands) {
    $p.Write($cmd + "`r`n")
    Start-Sleep -Milliseconds 1500
    $resp = $p.ReadExisting()
    Write-Output ('===== CMD: ' + $cmd + ' =====')
    $clean = ($resp -replace '[^\x20-\x7E\r\n]', '')
    Write-Output $clean
}
$p.Close()
