param(
    [string]$Port = 'COM9',
    [int]$Seconds = 8
)
# 串口读取走 console_lib.ps1: 字节级 UTF-8 解码 (中文日志可读),
# 输出仅剥 ANSI 色转义, 不再删除全部非 ASCII 字符
. (Join-Path $PSScriptRoot "console_lib.ps1")

try { Open-Serial -Port $Port -Baud 460800 } catch { Write-Output ('OPEN FAIL: ' + $_.Exception.Message); exit 1 }
$raw = Read-SerialWindow -Ms ($Seconds * 1000)
Close-Serial

$clean = Remove-Ansi $raw
($clean -split "`n") | Select-Object -First 34
