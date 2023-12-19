#!/usr/bin/python3

# Mostly copied from https://picamera.readthedocs.io/en/release-1.13/recipes2.html
# Run this script, then point a web browser at http:<this-ip-address>:8000
# Note: needs simplejpeg to be installed (pip3 install simplejpeg).

import io
import logging
import socketserver
from http import server
from threading import Condition

from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput

PAGE = """\
<html>
<head>
<title>picamera2 MJPEG streaming demo</title>
<style>
    body {
        background-color: black;
    }

    .parent {
    position: relative;
    top: 0;
    left: 0;
    }
    .front {
    position: relative;
    top: 0;
    left: 0;
    height: 780px;
    border: 1px solid #000000;
    }
    .rear {
    position: absolute;
    top: 0px;
    left: 930px;
    height: 240px;
    border: 1px solid #000000;
    }
</style>
</head>
<body>

<div class="parent">
    <img class="front" src="stream_front.mjpg"/>
    <img class="rear" src="stream_rear.mjpg"/>
</div>
</body>
</html>
"""

# <img src="stream_front.mjpg" width="48%" height="480" />
# <img src="stream_rear.mjpg" width="640" height="480" />

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
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream_front.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output_front.condition:
                        output_front.condition.wait()
                        frame_front = output_front.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame_front))
                    self.end_headers()
                    self.wfile.write(frame_front)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        elif self.path == '/stream_rear.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output_rear.condition:
                        output_rear.condition.wait()
                        frame_rear = output_rear.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame_rear))
                    self.end_headers()
                    self.wfile.write(frame_rear)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


picam_front = Picamera2(0)
picam_rear  = Picamera2(1)

# picam_front.configure(picam_front.create_video_configuration(main={"size": (640, 480)}))

picam_front.video_configuration.size = (640, 480)
# picam_front.video_configuration.controls.FrameRate = 18.0
output_front = StreamingOutput()
picam_front.start_recording(JpegEncoder(), FileOutput(output_front))

picam_rear.configure(picam_rear.create_video_configuration(main={"size": (640, 480)}))
# picam_rear.video_configuration.controls.FrameRate = 18.0
output_rear = StreamingOutput()
picam_rear.start_recording(JpegEncoder(), FileOutput(output_rear))

try:
    address = ('', 8000)
    server = StreamingServer(address, StreamingHandler)
    server.serve_forever()
finally:
    picam_front.stop_recording()
    picam_rear.stop_recording()
