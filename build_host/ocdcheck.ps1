$c = New-Object Net.Sockets.TcpClient('127.0.0.1', 4444)
$s = $c.GetStream()
$s.ReadTimeout = 2000
function Send-Ocd([string]$cmd, [int]$waitMs) {
    $b = [Text.Encoding]::ASCII.GetBytes($cmd + "`n")
    $s.Write($b, 0, $b.Length)
    Start-Sleep -Milliseconds $waitMs
    $buf = New-Object byte[] 8192
    try {
        $n = $s.Read($buf, 0, 8192)
        Write-Output ([Text.Encoding]::ASCII.GetString($buf, 0, $n))
    } catch { Write-Output "<<timeout>>" }
}
try { $null = $s.Read((New-Object byte[] 4096), 0, 4096) } catch {}
Send-Ocd 'version' 400
Send-Ocd 'targets' 600
Send-Ocd 'halt' 600
Send-Ocd 'reg pc' 500
Send-Ocd 'mdw 0x08000000 4' 500
Send-Ocd 'mdw 0x0802cefc 2' 500
Send-Ocd 'resume' 500
$c.Close()
