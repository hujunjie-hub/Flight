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
try {
    $sp.Open()
    $idle = Read-Window 500
    if ($idle.Length -gt 0) { Write-Output "=== idle output ==="; Write-Output $idle }

    # send a bare newline first so msh prompt shows and any half-line is flushed
    $sp.Write("`r")
    $idle2 = Read-Window 300
    if ($idle2.Length -gt 0) { Write-Output "=== after CR ==="; Write-Output $idle2 }

    foreach ($cmd in @("imudata", "imudata", "ps")) {
        $sp.Write($cmd + "`r")
        $out = Read-Window 1500
        Write-Output "=== [$cmd] ==="
        Write-Output $out
        Start-Sleep -Milliseconds 150
    }
} finally {
    if ($sp.IsOpen) { $sp.Close() }
}
