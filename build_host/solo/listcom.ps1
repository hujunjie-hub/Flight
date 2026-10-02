# 列出所有 COM 口的友好名称
Get-CimInstance Win32_PnPEntity | Where-Object {
    $_.Name -match '\(COM\d+\)'
} | ForEach-Object { "{0}" -f $_.Name }
