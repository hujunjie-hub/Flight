# Click at physical screen coordinates and capture the QGroundControl window.
# The calling shell must be DPI aware or the coordinates are virtualized.
param(
    [int]$X = -1,
    [int]$Y = -1,
    [int]$Wait = 900,
    [int]$HoldMs = 60,
    [string]$Out = '',
    [string]$Title = ''
)

$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
if (-not ('UI' -as [type])) {
    Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
public class UI {
  [DllImport("user32.dll")] public static extern bool SetProcessDpiAwarenessContext(IntPtr v);
  [DllImport("user32.dll")] public static extern bool SetCursorPos(int x, int y);
  [DllImport("user32.dll")] public static extern void mouse_event(uint f, uint dx, uint dy, uint d, UIntPtr e);
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
  [DllImport("user32.dll")] public static extern bool SetWindowPos(IntPtr h, IntPtr after, int x, int y, int cx, int cy, uint flags);
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int cmd);
  [DllImport("user32.dll")] public static extern bool MoveWindow(IntPtr h, int x, int y, int w, int ht, bool repaint);
  [StructLayout(LayoutKind.Sequential)] public struct RECT { public int L, T, R, B; }
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
  [DllImport("user32.dll", CharSet=CharSet.Auto)] public static extern int GetWindowText(IntPtr h, System.Text.StringBuilder s, int c);
}
'@
}
[void][UI]::SetProcessDpiAwarenessContext([IntPtr](-4))

$qgc = Get-Process QGroundControl -ErrorAction Stop | Select-Object -First 1
$h = $qgc.MainWindowHandle
# Normalise the window geometry so callers can use a stable 1800x1050 coordinate space.
[void][UI]::ShowWindow($h, 9)
[void][UI]::MoveWindow($h, 0, 0, 1800, 1050, $true)
Start-Sleep -Milliseconds 300
# Force the window above other apps, then drop topmost again. SetForegroundWindow alone is
# frequently refused by Windows when the calling process is not already in the foreground.
[void][UI]::SetWindowPos($h, [IntPtr](-1), 0, 0, 0, 0, 0x0043)   # HWND_TOPMOST, NOMOVE|NOSIZE|SHOWWINDOW
Start-Sleep -Milliseconds 150
[void][UI]::SetWindowPos($h, [IntPtr](-2), 0, 0, 0, 0, 0x0043)   # HWND_NOTOPMOST
[void][UI]::SetForegroundWindow($h)
Start-Sleep -Milliseconds 300

if ($X -ge 0 -and $Y -ge 0) {
    [void][UI]::SetCursorPos($X, $Y)
    Start-Sleep -Milliseconds 120
    [UI]::mouse_event(0x0002, 0, 0, 0, [UIntPtr]::Zero)
    Start-Sleep -Milliseconds $HoldMs
    [UI]::mouse_event(0x0004, 0, 0, 0, [UIntPtr]::Zero)
}

Start-Sleep -Milliseconds $Wait

if ($Out) {
    $r = New-Object UI+RECT
    [void][UI]::GetWindowRect($h, [ref]$r)
    $w = $r.R - $r.L; $ht = $r.B - $r.T
    $bmp = New-Object System.Drawing.Bitmap $w, $ht
    $g = [System.Drawing.Graphics]::FromImage($bmp)
    $g.CopyFromScreen($r.L, $r.T, 0, 0, (New-Object System.Drawing.Size $w, $ht))
    $g.Dispose()
    $bmp.Save($Out, [System.Drawing.Imaging.ImageFormat]::Png)
    $bmp.Dispose()
    Write-Host "saved $Out ($w x $ht)"
}
