# 复位板子并抓取上电日志: reboot 后持续采集 Sec 秒
param(
    [string]$Port = "COM9",
    [int]$Baud = 460800,
    [string]$OutFile = "boot_log.txt",
    [double]$Sec = 30,
    [string]$ResetCmd = "reboot"
)
$sp = New-Object System.IO.Ports.SerialPort $Port,$Baud,None,8,One
$sp.ReadTimeout = 200
try { $sp.Open() } catch { Write-Host "OPEN_FAIL: $($_.Exception.Message)"; exit 1 }
Start-Sleep -Milliseconds 300
try { $sp.DiscardInBuffer() } catch {}
$sp.Write($ResetCmd + "`r`n")
$fs = [System.IO.File]::CreateText($OutFile)
$deadline = [DateTime]::UtcNow.AddSeconds($Sec)
while ([DateTime]::UtcNow -lt $deadline) {
    try {
        if ($sp.BytesToRead -gt 0) { $fs.Write($sp.ReadExisting()) }
        else { Start-Sleep -Milliseconds 20 }
    } catch { Start-Sleep -Milliseconds 20 }
}
$fs.Close(); $sp.Close()
Write-Host "CAPTURED -> $OutFile"
