#!/usr/bin/env python3
"""
Artemis - stream the Pi camera to a browser on your laptop.

On the Pi:      python3 stream.py
On the laptop:  open http://<pi-ip>:8000   (find the IP with: hostname -I)
Quit with Ctrl+C.
"""
import io
import socketserver
from http import server
from threading import Condition

from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput

W, H = 1280, 720
PORT = 8000

PAGE = f"""<html><head><title>Artemis camera</title></head>
<body style="margin:0;background:#111;display:flex;justify-content:center;align-items:center;height:100vh">
<div style="position:relative">
  <img src="stream.mjpg" width="{W}" height="{H}" style="max-width:100vw;height:auto">
  <!-- crosshair -->
  <div style="position:absolute;left:50%;top:50%;width:60px;height:2px;background:red;transform:translate(-50%,-50%)"></div>
  <div style="position:absolute;left:50%;top:50%;width:2px;height:60px;background:red;transform:translate(-50%,-50%)"></div>
</div></body></html>"""


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            content = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except Exception:
                pass  # browser tab closed
        else:
            self.send_error(404)

    def log_message(self, *args):
        pass  # keep the terminal quiet


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (W, H)}))
output = StreamingOutput()
picam2.start_recording(MJPEGEncoder(), FileOutput(output))

try:
    print(f"Streaming - open http://<pi-ip>:{PORT} on your laptop (Ctrl+C to stop)")
    StreamingServer(("", PORT), StreamingHandler).serve_forever()
except KeyboardInterrupt:
    pass
finally:
    picam2.stop_recording()
