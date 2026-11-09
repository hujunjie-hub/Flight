"""Capture a top-level window by title (development helper).

    python tools/capture_window.py --title QGroundControl --output shot.png
"""

from __future__ import annotations

import argparse
import ctypes
import os
import time
from ctypes import wintypes

from PIL import ImageGrab


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--title", default="QGroundControl")
    parser.add_argument("--output", default="build/qgc_window.png")
    parser.add_argument("--contains", action="store_true", help="substring match")
    arguments = parser.parse_args()

    user32 = ctypes.windll.user32
    user32.SetProcessDPIAware()
    matches: list[tuple[int, str]] = []

    @ctypes.WINFUNCTYPE(ctypes.c_bool, wintypes.HWND, wintypes.LPARAM)
    def callback(hwnd, _):
        length = user32.GetWindowTextLengthW(hwnd)
        if length:
            buffer = ctypes.create_unicode_buffer(length + 1)
            user32.GetWindowTextW(hwnd, buffer, length + 1)
            title = buffer.value
            if (arguments.title in title) if arguments.contains else (title == arguments.title):
                matches.append((hwnd, title))
        return True

    user32.EnumWindows(callback, 0)
    if not matches:
        raise SystemExit(f"window not found: {arguments.title}")
    hwnd, title = matches[0]
    rect = wintypes.RECT()
    user32.GetWindowRect(hwnd, ctypes.byref(rect))
    user32.SetForegroundWindow(hwnd)
    time.sleep(0.8)
    image = ImageGrab.grab(bbox=(rect.left, rect.top, rect.right, rect.bottom),
                           all_screens=True)
    path = os.path.abspath(arguments.output)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    image.save(path)
    print(f"{title}: {image.size} -> {path}")


if __name__ == "__main__":
    main()
