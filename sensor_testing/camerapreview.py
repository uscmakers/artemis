#!/usr/bin/env python3
"""
Artemis - camera starter
Live 720p preview from the OV5647 with a crosshair overlay.

Run on the Pi (desktop session):  python3 camera_preview.py
Quit with Ctrl+C.
"""
import time
import numpy as np
from picamera2 import Picamera2, Preview

W, H = 1280, 720        # 720p is fast on the OV5647 (up to 60 fps)
FPS = 60

# Where the arrow actually lands relative to the image center (pixels).
# Tune these later when you calibrate against real shots.
AIM_OFFSET_X = 0
AIM_OFFSET_Y = 0

def make_crosshair(w, h, cx, cy, size=40, gap=8, thick=2):
    """Transparent RGBA image with a red crosshair at (cx, cy)."""
    ov = np.zeros((h, w, 4), dtype=np.uint8)
    red = (255, 0, 0, 255)
    t = thick // 2
    # horizontal arms
    ov[cy - t:cy + t + 1, cx - size:cx - gap] = red
    ov[cy - t:cy + t + 1, cx + gap:cx + size] = red
    # vertical arms
    ov[cy - size:cy - gap, cx - t:cx + t + 1] = red
    ov[cy + gap:cy + size, cx - t:cx + t + 1] = red
    # center dot
    ov[cy - t:cy + t + 1, cx - t:cx + t + 1] = red
    return ov

def main():
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (W, H)},
        controls={"FrameRate": FPS},
    )
    picam2.configure(config)

    # QTGL = window on the Pi desktop. Use Preview.DRM if running with no desktop.
    picam2.start_preview(Preview.QTGL)
    picam2.start()

    # Save one test shot so you know capture works
    time.sleep(1)
    picam2.capture_file("test.jpg")
    print("Saved test.jpg")

    cx = W // 2 + AIM_OFFSET_X
    cy = H // 2 + AIM_OFFSET_Y
    picam2.set_overlay(make_crosshair(W, H, cx, cy))

    print("Preview running - Ctrl+C to quit")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        print("Stopped")

if __name__ == "__main__":
    main()
