# 无GNSS融合全流程采集: reboot -> gins_fused_data 10Hz 流 -> 周期 gins 快照
param(
    [string]$Port = "COM9",
    [int]$Baud = 460800,
    [string]$OutFile = "run1.bin",
    [double]$Seconds = 330
)
$sp = New-Object System.IO.Ports.SerialPort $Port,$Baud,None,8,One
$sp.ReadTimeout = 100
try { $sp.Open() } catch { Write-Output ("OPEN FAIL: " + $_.Exception.Message); exit 1 }
Start-Sleep -Milliseconds 200
try { $sp.DiscardInBuffer() } catch {}

$fs = [System.IO.File]::CreateText($OutFile)
$cmds = @(
    @{ t = 0.0;   s = "reboot`r`n" },
    @{ t = 6.0;   s = "gins_fused_data on`r`n" },
    @{ t = 60.0;  s = "gins`r`n" },
    @{ t = 150.0; s = "gins`r`n" },
    @{ t = 240.0; s = "gins`r`n" },
    @{ t = 320.0; s = "gins_fused_data off`r`n" },
    @{ t = 322.0; s = "gins`r`n" }
)
$t0 = [Diagnostics.Stopwatch]::StartNew()
$ci = 0
while ($t0.Elapsed.TotalSeconds -lt $Seconds) {
    while ($ci -lt $cmds.Count -and $t0.Elapsed.TotalSeconds -ge $cmds[$ci].t) {
        $sp.Write($cmds[$ci].s); $ci++
    }
    try {
        if ($sp.BytesToRead -gt 0) { $fs.Write($sp.ReadExisting()) }
        else { Start-Sleep -Milliseconds 10 }
    } catch { Start-Sleep -Milliseconds 10 }
}
$fs.Close(); $sp.Close()
Write-Output ("CAPTURED " + $Seconds + "s -> " + $OutFile)
