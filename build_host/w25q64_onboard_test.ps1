# W25Q64 QSPI 框架驱动上板验收: COM9 console + OpenOCD(4444) 复位
# 用法: powershell -File build_host/w25q64_onboard_test.ps1
param(
    [string]$Port = "COM9",
    [int]$Baud = 460800
)
$sp = New-Object System.IO.Ports.SerialPort($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$sp.DtrEnable = $true
$sp.RtsEnable = $true
function Read-Window([int]$ms) {
    $out = ""
    $deadline = [DateTime]::Now.AddMilliseconds($ms)
    while ([DateTime]::Now -lt $deadline) {
        $chunk = $sp.ReadExisting()
        if ($chunk) { $out += $chunk }
        Start-Sleep -Milliseconds 30
    }
    return $out
}
function Ocd([string]$cmd) {
    $c = New-Object Net.Sockets.TcpClient('127.0.0.1', 4444)
    $s = $c.GetStream(); $s.ReadTimeout = 2000
    try { $null = $s.Read((New-Object byte[] 4096), 0, 4096) } catch {}
    $b = [Text.Encoding]::ASCII.GetBytes($cmd + "`n")
    $s.Write($b, 0, $b.Length)
    Start-Sleep -Milliseconds 500
    $resp = ""
    try { $resp = (New-Object Text.StringBuilder) } catch {}
    $deadline = [DateTime]::Now.AddSeconds(4)
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
try {
    $sp.Open()
    $null = Read-Window 500
    Write-Output "=== reset run (capture boot) ==="
    $null = Ocd "reset run"
    $boot = Read-Window 9000
    Write-Output $boot

    foreach ($cmd in @(
        "w25q64 id",
        "list_device",
        "w25q64 write 0x1000 AA 55 12 34 89 AB",
        "w25q64 read 0x1000 16",
        "w25q64 erase 0x1000 4k",
        "w25q64 read 0x1000 16",
        "w25q64 write 0x2000 CD EF",
        "w25q64 read 0x2000 8",
        "w25q64 erase 0x2000 4k",
        "w25q64 read 0x2000 8"
    )) {
        $sp.Write($cmd + "`r")
        $out = Read-Window 2500
        Write-Output "=== [$cmd] ==="
        Write-Output $out
        Start-Sleep -Milliseconds 150
    }
} finally {
    if ($sp.IsOpen) { $sp.Close() }
}
