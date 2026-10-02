param(
    [string]$Port = 'COM9',
    [int]$SampleSec = 10
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

foreach ($baud in @('460800', '115200')) {
    Write-Output ("########## TEST A/B: baud " + $baud + " ##########")
    Send-Cmd ('gnss baud ' + $baud) 500 | Out-Null
    Send-Cmd 'gnss reset' 500 | Out-Null
    Start-Sleep -Seconds $SampleSec
    Write-Output ('--- after ' + $SampleSec + 's @ ' + $baud + ' ---')
    Send-Cmd 'gnss' 800
    Send-Cmd 'gnss raw' 800
}
# 115200 下的协议层/数据层验证
Write-Output "########## um982 / gnssdata @115200 ##########"
Send-Cmd 'um982' 1500
Send-Cmd 'gnssdata' 800
$p.Close()
