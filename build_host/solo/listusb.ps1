# 列出 USB/PnP 设备中与调试探针相关的项
Get-CimInstance Win32_PnPEntity | Where-Object {
    $_.Name -match 'DAPLink|ST-?Link|CMSIS|OpenOCD|Probe|VCP|USB.*Serial|Serial.*USB'
} | ForEach-Object { "{0}  [{1}]" -f $_.Name, $_.PNPDeviceID }
