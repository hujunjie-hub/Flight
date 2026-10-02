# 动态监测 GPIOA/C MODER: 13s 连续采样 (~0.26s/次, 覆盖一个 adisretry 周期)
# 若 PA5/6/7 (SPI1) 或 PC4/5 (CS/RST) 在某瞬间离开 AF/输出模式 -> 有代码周期性改写引脚
$c = New-Object Net.Sockets.TcpClient('127.0.0.1', 4444)
$s = $c.GetStream(); $s.ReadTimeout = 3000
try { $null = $s.Read((New-Object byte[] 8192), 0, 8192) } catch {}

$sw = [Diagnostics.Stopwatch]::StartNew()
$seen = @{}
$n = 0
while ($sw.Elapsed.TotalSeconds -lt 13) {
  foreach ($cmd in @('mdw 0x58020000 1', 'mdw 0x58020800 1')) {
    $b = [Text.Encoding]::ASCII.GetBytes($cmd + "`n")
    $s.Write($b, 0, $b.Length)
    try {
      $buf = New-Object byte[] 4096
      $n2 = $s.Read($buf, 0, 4096)
      $t = [Text.Encoding]::ASCII.GetString($buf, 0, $n2)
      if ($t -match '(0x[0-9a-f]{8}):\s+([0-9a-f]{8})') {
        $key = "$($Matches[1])=$($Matches[2])"
        if (-not $seen.ContainsKey($key)) {
          $seen[$key] = $true
          Write-Output ("t={0:f1}s {1}" -f $sw.Elapsed.TotalSeconds, $key)
        }
      }
    } catch {}
  }
  $n++
}
$c.Close()
Write-Output "samples: $n, distinct states: $($seen.Count)"
