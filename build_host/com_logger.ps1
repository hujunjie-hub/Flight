param(
    [string]$Port = "COM9",
    [int]$Baud = 460800,
    [int]$Seconds = 12,
    [string]$OutFile = "D:\STM32Project\Flight\build_host\boot_log.txt"
)
$sp = New-Object System.IO.Ports.SerialPort($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$sp.DtrEnable = $true
$sp.RtsEnable = $true
$buf = New-Object System.Text.StringBuilder
try {
    $sp.Open()
    $deadline = [DateTime]::Now.AddSeconds($Seconds)
    while ([DateTime]::Now -lt $deadline) {
        $chunk = $sp.ReadExisting()
        if ($chunk) { [void]$buf.Append($chunk) }
        Start-Sleep -Milliseconds 20
    }
} finally {
    if ($sp.IsOpen) { $sp.Close() }
}
[System.IO.File]::WriteAllText($OutFile, $buf.ToString())
Write-Output ("captured {0} bytes -> {1}" -f $buf.Length, $OutFile)
