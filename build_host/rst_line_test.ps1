# RST(PC5) 线连通性测试: 释放为输入读线上电平。
# ADIS16505 RST 输入带内部上拉 -> 芯片有电且线通时读 1; 读 0 = 芯片无电或线断。
# GPIOC_MODER=0x58020800, PC5 = bits[11:10]; IDR=0x58020810 bit5
$c = New-Object Net.Sockets.TcpClient('127.0.0.1', 4444)
$s = $c.GetStream(); $s.ReadTimeout = 3000
try { $null = $s.Read((New-Object byte[] 8192), 0, 8192) } catch {}

$out = ''
function Cmd($txt) {
  $b = [Text.Encoding]::ASCII.GetBytes($txt + "`n")
  $script:s.Write($b, 0, $b.Length)
  Start-Sleep -Milliseconds 260
  try {
    $buf = New-Object byte[] 8192
    $n = $s.Read($buf, 0, 8192)
    $script:out += "$txt ==> " + ([Text.Encoding]::ASCII.GetString($buf, 0, $n) -replace "`n", ' ' -replace "`r", '') + "`n"
  } catch { $script:out += "$txt ==> (timeout)`n" }
}

Cmd 'halt'
Cmd 'mdw 0x58020800 1'                    # MODER backup
Cmd 'mww 0x58020800 0xF3FFD5FF'          # PC5 -> input (bits[11:10]=00), PC4 keep output
Cmd 'mdw 0x58020810 1'                    # IDR: PC5 line level (pull-up present?)
Cmd 'mdw 0x58020810 1'                    # IDR again (stability)
Cmd 'mww 0x58020800 0xF3FFF5FF'          # PC5 -> output again
Cmd 'mww 0x58020814 0x00000030'          # ODR: CS/RST high
Cmd 'resume'
$c.Close()
Write-Output $out
