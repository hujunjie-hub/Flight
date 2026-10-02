# 爆炸狩猎: N 轮 reboot, 每轮 70s, 全程监听 console 存文件
param(
    [string]$Port = "COM9",
    [int]$Baud = 460800,
    [string]$OutFile = "hunt.txt",
    [int]$Cycles = 6,
    [double]$PerCycleSec = 70
)
$sp = New-Object System.IO.Ports.SerialPort $Port,$Baud,None,8,One
$sp.ReadTimeout = 100
try { $sp.Open() } catch { Write-Output ("OPEN FAIL: " + $_.Exception.Message); exit 1 }
$fs = [System.IO.File]::CreateText($OutFile)
for ($c = 1; $c -le $Cycles; $c++) {
    $fs.Write("`r`n===== CYCLE $c =====`r`n")
    $sp.Write("reboot`r`n")
    $dl = [DateTime]::UtcNow.AddSeconds($PerCycleSec)
    while ([DateTime]::UtcNow -lt $dl) {
        try {
            if ($sp.BytesToRead -gt 0) { $fs.Write($sp.ReadExisting()) }
            else { Start-Sleep -Milliseconds 15 }
        } catch {}
    }
}
$fs.Close(); $sp.Close()
Write-Output "DONE -> $OutFile"
