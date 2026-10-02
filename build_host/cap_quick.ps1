param(
    [string]$Port = 'COM9',
    [int]$Seconds = 120,
    [string]$OutFile = 'D:\STM32Project\Flight\build_host\data\quick_cap.bin'
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
$p.Write("reboot`r`n")
Start-Sleep -Milliseconds 300
Sink-Read $p $ms $buf
$cmds = @(
    @{ t = 20.0;              s = "gins_fused_data on`r`n" },
    @{ t = ($Seconds - 8.0);  s = "gins`r`n" }
)
$t0 = [Diagnostics.Stopwatch]::StartNew()
while ($t0.Elapsed.TotalSeconds -lt $Seconds) {
    Sink-Read $p $ms $buf
    foreach ($c in $cmds) {
        if ($c.t -ge 0 -and $t0.Elapsed.TotalSeconds -ge $c.t) {
            $p.Write($c.s); $c.t = -1.0
        }
    }
    Start-Sleep -Milliseconds 15
}
$p.Close()
[IO.File]::WriteAllBytes($OutFile, $ms.ToArray())
Write-Output ("captured {0} bytes -> {1}" -f $ms.Length, $OutFile)
