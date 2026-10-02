# console_lib.ps1 — 串口 console 公共库: 字节级读取 + 持久 UTF-8 解码
#
# 背景: 固件 ulog/kprintf 输出 UTF-8 原始字节, 而 .NET SerialPort 默认
# ASCII 编码解码 —— 所有 >0x7F 字节 (全部中文) 被 ReadExisting 替换成
# '?'。本库以字节读端口, 用持久 Decoder 按 UTF-8 解码 (Decoder 跨调用
# 保持状态, 多字节序列被读块截断也不会花字), 并把控制台输出编码切到
# UTF-8, 使 Git Bash/终端管道直接可读。
#
# 用法 (调用脚本 dot-source 后):
#   . "$PSScriptRoot/console_lib.ps1"
#   Open-Serial -Port COM9 -Baud 460800
#   $boot = Read-SerialWindow -Ms 9000        # 读窗口内到达的字节
#   Send-SerialLine "param"                   # 发 FinSH 命令 (自动 \r)
#   $resp = Read-SerialWindow -Ms 1500
#   $resp | Remove-Ansi | Write-Output        # 去掉 ulog ANSI 色转义
#   Close-Serial

[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

function Open-Serial {
    param([string]$Port = "COM9", [int]$Baud = 460800)
    $script:sp = New-Object System.IO.Ports.SerialPort(
        $Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
    $sp.DtrEnable = $true
    $sp.RtsEnable = $true
    # 默认 4KB 在 460800 波特率 + 100Hz 打印链路下会溢出丢字节
    $sp.ReadBufferSize = 262144
    $sp.ReadTimeout = 100
    # 持久 UTF-8 解码器: 跨 Read 调用保持不完整多字节序列的状态
    $script:serialDecoder = [System.Text.Encoding]::UTF8.GetDecoder()
    $script:serialAcc = New-Object System.Text.StringBuilder
    $sp.Open()
    $null = $sp.ReadExisting()                  # 清开门瞬间的旧缓冲
}

# 读 Ms 毫秒窗口内到达的字节并按 UTF-8 累计解码, 返回该窗口文本
function Read-SerialWindow {
    param([int]$Ms = 1000)
    $deadline = [DateTime]::Now.AddMilliseconds($Ms)
    $win = New-Object System.Text.StringBuilder
    while ([DateTime]::Now -lt $deadline) {
        $n = $sp.BytesToRead
        if ($n -gt 0) {
            $buf = New-Object byte[] $n
            $null = $sp.BaseStream.Read($buf, 0, $n)
            $cch = $serialDecoder.GetCharCount($buf, 0, $n)
            if ($cch -gt 0) {
                $chars = New-Object char[] $cch
                $null = $serialDecoder.GetChars($buf, 0, $n, $chars, 0)
                $null = $win.Append($chars)
                $null = $serialAcc.Append($chars)
            }
        }
        else { Start-Sleep -Milliseconds 20 }
    }
    return $win.ToString()
}

# 累计缓冲 (上次 Read-SerialWindow 之后收到的全部文本), 读后清空
function Read-SerialDrain {
    $s = $serialAcc.ToString()
    $null = $serialAcc.Clear()
    return $s
}

# 发送 FinSH 命令行 (CRLF 由固件 msh 解析, CR 足够)
function Send-SerialLine {
    param([string]$Cmd)
    $sp.Write($Cmd + "`r")
}

# 去掉 ulog 的 ANSI SGR 色转义 (ESC[...m)
function Remove-Ansi {
    param([string]$Text)
    return ($Text -replace "\x1B\[[0-9;]*[A-Za-z]", "")
}

function Close-Serial {
    if ($sp -and $sp.IsOpen) { $sp.Close() }
}
