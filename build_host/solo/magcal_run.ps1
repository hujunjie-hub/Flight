# 启动磁标定并全程监听 console 直到拟合完成
param(
    [string]$Port = "COM9",
    [int]$Baud = 460800,
    [string]$OutFile = "magcal_live.txt",
    [double]$Sec = 100,
    [int]$CalSec = 75
)
$sp = New-Object System.IO.Ports.SerialPort $Port,$Baud,None,8,One
$sp.ReadTimeout = 100
try { $sp.Open() } catch { Write-Output ("OPEN FAIL: " + $_.Exception.Message); exit 1 }
Start-Sleep -Milliseconds 200
try { $sp.DiscardInBuffer() } catch {}
$sp.Write("magcal start $CalSec`r`n")
$fs = [System.IO.File]::CreateText($OutFile)
$dl = [DateTime]::UtcNow.AddSeconds($Sec)
while ([DateTime]::UtcNow -lt $dl) {
    try {
        if ($sp.BytesToRead -gt 0) { $fs.Write($sp.ReadExisting()) }
        else { Start-Sleep -Milliseconds 10 }
    } catch {}
}
$fs.Close()
$sp.Write("magcal`r`n")
Start-Sleep -Milliseconds 2500
try { if ($sp.BytesToRead -gt 0) { $tail = $sp.ReadExisting() } else { $tail = "" } } catch { $tail = "" }
$sp.Close()
[System.IO.File]::AppendAllText($OutFile, $tail)
Write-Output "DONE -> $OutFile"
