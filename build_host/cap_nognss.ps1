param(
    [string]$Port = 'COM9',
    [int]$Seconds = 330,
    [string]$OutFile = 'D:\STM32Project\Flight\build_host\data\nognss_quality_20261001.bin'
)
$p = New-Object System.IO.Ports.SerialPort($Port, 460800, 'None', 8, 'One')
$p.ReadTimeout = 100
try { $p.Open() } catch { Write-Output ('OPEN FAIL: ' + $_.Exception.Message); exit 1 }

$ms = New-Object System.IO.MemoryStream
$buf = New-Object byte[] 16384
function Sink-Read($p, $ms, $buf) {
    try {
        $n = $p.Read($buf, 0, $buf.Length)
        if ($n -gt 0) { $ms.Write($buf, 0, $n) }
    } catch {}
}

# t=0   发 reboot，板子从干净状态启动（boot 日志 + 对准 + 30s 无GNSS播种全程可见）
$p.Write("reboot`r`n")
Start-Sleep -Milliseconds 300
Sink-Read $p $ms $buf

# t=20s 开启 GINS 文本输出（此时还在对准/等待窗，1Hz 心跳可见 ready:0->1 翻转）
# t=Seconds-10s 发 gins 命令拿最终统计
$cmds = @(
    @{ t = 20.0;                s = "gins_fused_data on`r`n" },
    @{ t = ($Seconds - 10.0);   s = "gins`r`n" }
)
$t0 = [Diagnostics.Stopwatch]::StartNew()
while ($t0.Elapsed.TotalSeconds -lt $Seconds) {
    Sink-Read $p $ms $buf
    foreach ($c in $cmds) {
        if ($c.t -ge 0 -and $t0.Elapsed.TotalSeconds -ge $c.t) {
            $p.Write($c.s)
            $c.t = -1.0
        }
    }
    Start-Sleep -Milliseconds 15
}
$p.Close()
[IO.File]::WriteAllBytes($OutFile, $ms.ToArray())
Write-Output ("captured {0} bytes -> {1}" -f $ms.Length, $OutFile)
