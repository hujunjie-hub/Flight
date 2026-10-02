# 复发验证: 连续 N 次硬复位, 每次抓 boot 日志统计 ADIS 探测结果
# 全部成功 = 一次性事件; 再次失败 = 固件行为会触发芯片卡死 (需查根因)
param([int]$N = 5)

$c = New-Object Net.Sockets.TcpClient('127.0.0.1', 4444)
$s = $c.GetStream(); $s.ReadTimeout = 3000
try { $null = $s.Read((New-Object byte[] 8192), 0, 8192) } catch {}

for ($i = 1; $i -le $N; $i++) {
  # reset run
  $b = [Text.Encoding]::ASCII.GetBytes("reset run`n")
  $s.Write($b, 0, $b.Length)
  Start-Sleep -Milliseconds 500
  try { $null = $s.Read((New-Object byte[] 8192), 0, 8192) } catch {}

  # 抓 boot 日志 (6s 覆盖 1s 内的传感器 init)
  $p = New-Object IO.Ports.SerialPort('COM9',460800,'None',8,'One')
  $p.ReadTimeout = 200
  $p.Open()
  $p.DiscardInBuffer()
  $buf = New-Object System.Text.StringBuilder
  $sw = [Diagnostics.Stopwatch]::StartNew()
  while ($sw.Elapsed.TotalSeconds -lt 6) {
    try { $d = $p.ReadExisting(); if ($d) { [void]$buf.Append($d) } } catch {}
    Start-Sleep -Milliseconds 40
  }
  $p.Close()
  $txt = $buf.ToString()
  $adis_ok = $txt -match 'ADIS16505 found'
  $adis_bad = $txt -match 'not found|probe xfr failed'
  Write-Output ("[reset {0}] ADIS: {1}  (found={2} failed={3} len={4})" -f `
    $i, $(if ($adis_ok) {'ONLINE'} elseif ($adis_bad) {'FAIL'} else {'?'}), $adis_ok, $adis_bad, $txt.Length)
}
$c.Close()
