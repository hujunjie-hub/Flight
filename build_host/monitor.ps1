param(
    [string]$Port = 'COM9',
    [int]$MaxSec = 300
)
$p = New-Object System.IO.Ports.SerialPort($Port, 460800, 'None', 8, 'One')
$p.ReadTimeout = 500
try { $p.Open() } catch { Write-Output ('OPEN FAIL: ' + $_.Exception.Message); exit 1 }

$sw = [Diagnostics.Stopwatch]::StartNew()
$sb = New-Object System.Text.StringBuilder
$lastReport = 0
$charsLast10 = 0
Write-Output ("MONITOR START")
while ($sw.Elapsed.TotalSeconds -lt $MaxSec) {
    $chunk = ''
    try { $chunk = $p.ReadExisting() } catch {}
    $charsLast10 += $chunk.Length
    $null = $sb.Append($chunk)
    if ($chunk.Length -gt 0) { $script:lastData = $sw.Elapsed.TotalSeconds }

    $now = [int]$sw.Elapsed.TotalSeconds
    if ($now -ge $lastReport + 10) {
        Write-Output ("T+" + $now.ToString('D4') + "s: " + $charsLast10 + " chars/10s " +
                      $(if ($charsLast10 -eq 0) {'<<< SILENT'} else {''}))
        if ($charsLast10 -eq 0) {
            Write-Output ("HANG DETECTED at T+" + $now + "s, last data at ~T+" +
                         [int]($now - 10) + "s")
            break
        }
        $charsLast10 = 0
        $lastReport = $now
    }
    Start-Sleep -Milliseconds 100
}
# 保留最后 2KB 原始输出找最后的话
Write-Output "--- LAST 800 chars before end (cleaned) ---"
$t = $sb.ToString()
if ($t.Length -gt 800) { $t = $t.Substring($t.Length - 800) }
$clean = ($t -replace '[^\x20-\x7E\r\n]', '')
Write-Output $clean
$p.Close()
