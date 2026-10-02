# 采集原始数据流: 先开 magout/barout, 采 Sec 秒, 再关掉
param(
    [string]$Port = "COM9",
    [int]$Baud = 460800,
    [string]$OutFile = "raw90.bin",
    [double]$Sec = 90,
    [ValidateSet("mag","baro")]
    [string]$Stream = "mag"
)
$sp = New-Object System.IO.Ports.SerialPort $Port,$Baud,None,8,One
$sp.ReadTimeout = 100
try { $sp.Open() } catch { Write-Output ("OPEN FAIL: " + $_.Exception.Message); exit 1 }
Start-Sleep -Milliseconds 200
try { $sp.DiscardInBuffer() } catch {}
# 单写者原则: 两个都先关, 再只开目标流
$sp.Write("magout off`r`n")
$sp.Write("barout off`r`n")
Start-Sleep -Milliseconds 800
try { $sp.DiscardInBuffer() } catch {}
if ($Stream -eq "mag") { $sp.Write("magout on`r`n") } else { $sp.Write("barout on`r`n") }
$fs = [System.IO.File]::CreateText($OutFile)
$dl = [DateTime]::UtcNow.AddSeconds($Sec)
while ([DateTime]::UtcNow -lt $dl) {
    try {
        if ($sp.BytesToRead -gt 0) { $fs.Write($sp.ReadExisting()) }
        else { Start-Sleep -Milliseconds 10 }
    } catch {}
}
$fs.Close()
$sp.Write("magout off`r`n")
$sp.Write("barout off`r`n")
Start-Sleep -Milliseconds 600
$sp.Close()
Write-Output "DONE -> $OutFile"
