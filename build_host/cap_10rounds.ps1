param(
    [string]$Port = 'COM9',
    [int]$Rounds = 10,
    [int]$SecondsPerRound = 220,
    [string]$OutDir = 'D:\STM32Project\Flight\build_host\test10_nognss'
)
if (-not (Test-Path $OutDir)) { New-Item -ItemType Directory -Path $OutDir | Out-Null }

for ($r = 1; $r -le $Rounds; $r++) {
    $out = Join-Path $OutDir ("round_{0:D2}.bin" -f $r)
    Write-Output ("=== round {0}/{1} start {2} ===" -f $r, $Rounds, (Get-Date -Format 'HH:mm:ss'))

    $p = New-Object System.IO.Ports.SerialPort($Port, 460800, 'None', 8, 'One')
    $p.ReadTimeout = 100
    try { $p.Open() } catch {
        Write-Output ('OPEN FAIL round ' + $r + ': ' + $_.Exception.Message)
        Start-Sleep -Seconds 3
        continue
    }

    $ms = New-Object System.IO.MemoryStream
    $buf = New-Object byte[] 16384
    function Sink-Read($p, $ms, $buf) {
        try {
            $n = $p.Read($buf, 0, $buf.Length)
            if ($n -gt 0) { $ms.Write($buf, 0, $n) }
        } catch {}
    }

    $p.Write("reboot`r`n")
    Start-Sleep -Milliseconds 400
    Sink-Read $p $ms $buf

    $cmds = @(
        @{ t = 15.0;              s = "gins_fused_data on`r`n" },
        @{ t = ($SecondsPerRound - 8.0); s = "gins`r`n" }
    )
    $t0 = [Diagnostics.Stopwatch]::StartNew()
    while ($t0.Elapsed.TotalSeconds -lt $SecondsPerRound) {
        Sink-Read $p $ms $buf
        foreach ($c in $cmds) {
            if ($c.t -ge 0 -and $t0.Elapsed.TotalSeconds -ge $c.t) {
                $p.Write($c.s); $c.t = -1.0
            }
        }
        Start-Sleep -Milliseconds 15
    }
    $p.Close()
    [IO.File]::WriteAllBytes($out, $ms.ToArray())
    Write-Output ("round {0}: {1} bytes -> {2}" -f $r, $ms.Length, $out)
    Start-Sleep -Seconds 2
}
Write-Output ("ALL {0} ROUNDS DONE {1}" -f $Rounds, (Get-Date -Format 'HH:mm:ss'))
