#!/usr/bin/python3

# Mostly copied from https://picamera.readthedocs.io/en/release-1.13/recipes2.html
# Run this script, then point a web browser at http:<this-ip-address>:8000
# Note: needs simplejpeg to be installed (pip3 install simplejpeg).

import sys

import io
import logging
import socketserver
from http import server
from threading import Condition, Thread

from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
import libcamera 

import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

import board
import neopixel

import signal

def signal_handler(signal, frame):
    print('CTRL-C caught, exiting.')
    server.shutdown()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

pixels = neopixel.NeoPixel(board.D12, 12)

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
        self.pixels = pixels

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class StreamingHandler(server.BaseHTTPRequestHandler):    
    brightness = 120
    brightness_min = 50
    brightness_max = 175
    
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
                while not rospy.is_shutdown():
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
                while not rospy.is_shutdown():
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
        elif self.path == '/lights_on':
            pixels.fill((self.brightness, self.brightness, self.brightness))
            self.send_response(200)
        elif self.path == '/lights_off':
            pixels.fill((0, 0, 0))
            self.send_response(200)
        elif self.path == '/lights_up':
            if self.brightness <= self.brightness_max:
                self.brightness = self.brightness + 10

            pixels.fill((self.brightness, self.brightness, self.brightness))
            self.send_response(200)
        elif self.path == '/lights_down':
            if self.brightness >= self.brightness_min:
                self.brightness = self.brightness - 10
                
            print(self.brightness)
            pixels.fill((self.brightness, self.brightness, self.brightness))
            self.send_response(200)
        else:
            self.send_error(404)
            self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def PublishThreadFront():
    pub = rospy.Publisher('front_camera/image/compressed', CompressedImage, queue_size=1)
    rate = rospy.Rate(18)
    msg = CompressedImage()
    msg.format = "jpeg"
    
    print("Publishing front camera")
    while not rospy.is_shutdown():
        msg.data = np.array(output_front.frame).tobytes()
        # msg.data = output_front.frame
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()
    print("Stopped publishing front camera")

def PublishThreadRear():
    pub = rospy.Publisher('rear_camera/image/compressed', CompressedImage, queue_size=1)
    rate = rospy.Rate(18)
    msg = CompressedImage()
    msg.format = "jpeg"
    
    print("Publishing rear camera")
    while not rospy.is_shutdown():
        msg.data = np.array(output_rear.frame).tobytes()
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()
    print("Stopped publishing rear camera")

def runMjpegServer():
    print("Starting mjpeg server")
    try:
        server.serve_forever()
    finally:
        print("Stopping mjpeg server")
        picam_front.stop_recording()
        picam_rear.stop_recording()
        print("Stopped mjpeg server")

address = ('', 8000)
server = StreamingServer(address, StreamingHandler)

rospy.init_node("mjpeg_server")

picam_front = Picamera2(0)
picam_rear  = Picamera2(1)

print("Starting cameras")

front_cam_config = picam_front.create_video_configuration(
    main={
            "size": (1296, 972)
          })
picam_front.configure(front_cam_config)
picam_front.set_controls({
        "AeFlickerMode": 1, 
        "AeFlickerPeriod": 10000,
        })

picam_front.controls.AwbEnable = False
# picam_front.controls.AwbMode = 0

# picam_front.video_configuration.size = (1296, 972)
# picam_front.set_controls({'AeFlickerMode': 1, 'AeFlickerPeriod': 10000})
# picam_front.set_controls({'AeFlickerPeriod': 10000})

output_front = StreamingOutput()
picam_front.start_recording(JpegEncoder(), FileOutput(output_front))
print("Front camera started")


picam_rear.configure(picam_rear.create_video_configuration(main={"size": (640, 480)}))
picam_rear.controls.AwbEnable = False
# picam_rear.controls.AwbMode = 0
# picam_rear.video_configuration.controls.FrameRate = 18.0
output_rear = StreamingOutput()
picam_rear.start_recording(JpegEncoder(), FileOutput(output_rear))
print("Rear camera started")

front_thread = Thread(target=PublishThreadFront)
rear_thread = Thread(target=PublishThreadRear)
mjpeg_thread = Thread(target=runMjpegServer)

print("Starting threads")
print("Starting front thread")
front_thread.start()
print("Starting rear thread")
rear_thread.start()
print("Starting mjpeg thread")
mjpeg_thread.start()
print("Threads started")

front_thread.join()
rear_thread.join()
mjpeg_thread.join()

rospy.spin()