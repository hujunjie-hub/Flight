# ADIS 总线验证 v6 (最终): 逐字节读 FIFO 并每步核对 SR.RXP (bit0)
# 收到的真实字节数 = RQP 保持 1 的读次数; 字节 3/4 = PROD_ID 数据帧
$c = New-Object Net.Sockets.TcpClient('127.0.0.1', 4444)
$s = $c.GetStream(); $s.ReadTimeout = 3000
try { $null = $s.Read((New-Object byte[] 8192), 0, 8192) } catch {}
$out = ''
function Cmd($txt) {
  $b = [Text.Encoding]::ASCII.GetBytes($txt + "`n")
  $script:s.Write($b, 0, $b.Length)
  Start-Sleep -Milliseconds 200
  try {
    $buf = New-Object byte[] 8192
    $n = $s.Read($buf, 0, 8192)
    $script:out += "$txt ==> " + ([Text.Encoding]::ASCII.GetString($buf, 0, $n) -replace "`n", ' ' -replace "`r", '') + "`n"
  } catch { $script:out += "$txt ==> (timeout)`n" }
}

Cmd 'halt'
Cmd 'mww 0x40013000 0x00001000'
Cmd 'mww 0x40013004 0x00000002'
Cmd 'mww 0x40013018 0x000000F8'
Cmd 'mww 0x40013000 0x00001001'
Cmd 'mww 0x58020814 0x00000020'   # CS low
Cmd 'mww 0x40013020 0x00000072'
Cmd 'mww 0x40013020 0x00000000'
Cmd 'mww 0x40013000 0x00001201'
Cmd 'mww 0x40013018 0x00000008'
Cmd 'mww 0x40013020 0x00000000'
Cmd 'mww 0x40013020 0x00000000'
Cmd 'mww 0x40013000 0x00001201'
Cmd 'mww 0x58020814 0x00000030'   # CS high
# 逐字节: RXP -> RXDR 交替
Cmd 'mdw 0x40013014 1'
Cmd 'mdw 0x40013030 1'    # byte1
Cmd 'mdw 0x40013014 1'
Cmd 'mdw 0x40013030 1'    # byte2
Cmd 'mdw 0x40013014 1'
Cmd 'mdw 0x40013030 1'    # byte3 (data MSB?)
Cmd 'mdw 0x40013014 1'
Cmd 'mdw 0x40013030 1'    # byte4 (data LSB?)
Cmd 'mdw 0x40013014 1'
Cmd 'mdw 0x40013030 1'    # byte5 (must be empty)
Cmd 'mdw 0x40013014 1'
Cmd 'mww 0x40013000 0x00001000'
Cmd 'resume'
$c.Close()
Write-Output $out
