# param_calib 上板验收: COM9 console + OpenOCD(4444) 复位/FinSH 交互
# 串口读取走 console_lib.ps1 (字节级 UTF-8, 中文日志不花字)
# 用法: powershell -ExecutionPolicy Bypass -File build_host/param_onboard_test.ps1
param(
    [string]$Port = "COM9",
    [int]$Baud = 460800
)
. (Join-Path $PSScriptRoot "console_lib.ps1")

function Ocd([string]$cmd) {
    $c = New-Object Net.Sockets.TcpClient('127.0.0.1', 4444)
    $s = $c.GetStream(); $s.ReadTimeout = 2000
    try { $null = $s.Read((New-Object byte[] 4096), 0, 4096) } catch {}
    $b = [Text.Encoding]::ASCII.GetBytes($cmd + "`n")
    $s.Write($b, 0, $b.Length)
    Start-Sleep -Milliseconds 500
    $resp = New-Object Text.StringBuilder
    $deadline = [DateTime]::Now.AddSeconds(6)
    while ([DateTime]::Now -lt $deadline) {
        try {
            $buf = New-Object byte[] 8192
            $n = $s.Read($buf, 0, 8192)
            [void]$resp.Append([Text.Encoding]::ASCII.GetString($buf, 0, $n))
        } catch { Start-Sleep -Milliseconds 200 }
    }
    $c.Close()
    return $resp.ToString()
}
function Finsh([string]$cmd) {
    $null = Read-SerialWindow -Ms 200
    Send-SerialLine $cmd
    return (Read-SerialWindow -Ms 1500)
}
try {
    Open-Serial -Port $Port -Baud $Baud

    Write-Output "===== BOOT 1: reset run ====="
    $null = Ocd "reset run"
    Write-Output (Remove-Ansi (Read-SerialWindow -Ms 10000))

    Write-Output "===== param ====="
    Write-Output (Remove-Ansi (Finsh "param"))

    Write-Output "===== sysinfo ====="
    Write-Output (Remove-Ansi (Finsh "sysinfo"))

    Write-Output "===== nav ====="
    Write-Output (Remove-Ansi (Finsh "nav"))

    Write-Output "===== nav set alt 123.0 + nav save ====="
    Write-Output (Remove-Ansi (Finsh "nav set alt 123.0"))
    Write-Output (Remove-Ansi (Finsh "nav save"))

    Write-Output "===== BOOT 2: reset run (persistence check) ====="
    $null = Ocd "reset run"
    Write-Output (Remove-Ansi (Read-SerialWindow -Ms 10000))

    Write-Output "===== nav (persisted) ====="
    Write-Output (Remove-Ansi (Finsh "nav"))

    Write-Output "===== param erase nav + nav load (factory reset check) ====="
    Write-Output (Remove-Ansi (Finsh "param erase nav"))
    Write-Output (Remove-Ansi (Finsh "nav load"))
    Write-Output (Remove-Ansi (Finsh "nav"))

    Write-Output "===== DONE ====="
}
finally {
    Close-Serial
}
