param(
    [string]$Port = 'COM9',
    [string[]]$Commands = @('gnssraw', 'um982', 'gnssdata')
)
$p = New-Object System.IO.Ports.SerialPort($Port, 460800, 'None', 8, 'One')
$p.ReadTimeout = 200
try { $p.Open() } catch { Write-Output ('OPEN FAIL: ' + $_.Exception.Message); exit 1 }

Start-Sleep -Milliseconds 300
$null = $p.ReadExisting()
$p.Write("`r`n")
Start-Sleep -Milliseconds 800
$null = $p.ReadExisting()

foreach ($cmd in $Commands) {
    $p.Write($cmd + "`r`n")
    Start-Sleep -Milliseconds 1500
    $resp = $p.ReadExisting()
    # 去掉二进制帧字节, 只留可打印字符
    $clean = ($resp -replace '[^\x20-\x7E\r\n]', '')
    # 过滤掉流式数据行, 只留命令响应
    $keep = $clean -split "`r?`n" | Where-Object {
        $_ -notmatch '^(time:|natt )' -and $_ -notmatch 'rmag:' -and $_ -match '\S'
    }
    Write-Output ('===== ' + $cmd + ' =====')
    $keep | ForEach-Object { $_.TrimEnd() }
}
$p.Close()
