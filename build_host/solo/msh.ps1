# 独立测试串口控制台工具: 向板子 MSH 发命令并采集输出
# 用法: powershell -File msh.ps1 -Port COM9 -Baud 460800 -Cmds "cmd1;cmd2" -WaitSec 3 [-OutFile x.txt]
param(
    [string]$Port = "COM9",
    [int]$Baud = 460800,
    [string]$Cmds = "help",
    [double]$PerCmdSec = 2.0,
    [double]$TailSec = 1.0,
    [string]$OutFile = ""
)

$sp = New-Object System.IO.Ports.SerialPort $Port,$Baud,None,8,One
$sp.ReadTimeout = 200
$sp.NewLine = "`n"
try { $sp.Open() } catch { Write-Host "OPEN_FAIL: $($_.Exception.Message)"; exit 1 }
Start-Sleep -Milliseconds 400
try { $sp.DiscardInBuffer() } catch {}

function Read-Avail([double]$sec) {
    $sb = New-Object System.Text.StringBuilder
    $deadline = [DateTime]::UtcNow.AddSeconds($sec)
    while ([DateTime]::UtcNow -lt $deadline) {
        try {
            $n = $sp.BytesToRead
            if ($n -gt 0) { [void]$sb.Append($sp.ReadExisting()) }
            else { Start-Sleep -Milliseconds 20 }
        } catch { Start-Sleep -Milliseconds 20 }
    }
    return $sb.ToString()
}

# 敲回车唤醒 shell
$sp.Write("`r`n")
[void](Read-Avail 0.4)

foreach ($c in $Cmds.Split(';')) {
    $c = $c.Trim()
    if ($c -eq "") { continue }
    $sp.Write($c + "`r`n")
    $out = Read-Avail $PerCmdSec
    Write-Output "===== CMD: $c ====="
    Write-Output $out
}
Write-Output "===== TAIL ====="
Write-Output (Read-Avail $TailSec)
$sp.Close()
