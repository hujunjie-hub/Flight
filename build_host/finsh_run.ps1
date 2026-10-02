param(
    [string]$Port = 'COM9',
    [string[]]$Commands = @('gins'),
    [int]$WaitMs = 1500,
    [int]$TailMs = 800,
    [string]$OutFile = ''
)
$p = New-Object System.IO.Ports.SerialPort($Port, 460800, 'None', 8, 'One')
$p.ReadTimeout = 200
$p.NewLine = "`n"
try { $p.Open() } catch { Write-Output ('OPEN FAIL: ' + $_.Exception.Message); exit 1 }

Start-Sleep -Milliseconds 300
$null = $p.ReadExisting()
$p.Write("`r`n")
Start-Sleep -Milliseconds 500
$null = $p.ReadExisting()

foreach ($cmd in $Commands) {
    $p.Write($cmd + "`r`n")
    Start-Sleep -Milliseconds $WaitMs
}
Start-Sleep -Milliseconds $TailMs
$resp = $p.ReadExisting()
$p.Close()

$clean = ($resp -replace '[^\x20-\x7E\r\n]', '')
Write-Output $clean
if ($OutFile -ne '') { [IO.File]::WriteAllText($OutFile, $clean) }
