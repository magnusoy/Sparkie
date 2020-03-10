# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This module ...

__author__ = "Magnus Kvendseth Øye"
__copyright__ = "Copyright 2019, Sparkie Quadruped Robot"
__credits__ = ["Magnus Kvendseth Øye", "Petter Drønnen", "Vegard Solheim"]
__version__ = "1.0.0"
__license__ = "MIT"
__maintainer__ = "Magnus Kvendseth Øye"
__email__ = "magnus.oye@gmail.com"
__status__ = "Development"
"""


import pyrealsense2 as rs
import numpy as np
from enum import IntEnum
import base64
import cv2
import time
import zmq


IP = '*'
PORT = 5555
INTERVAL = 0.05

def convert_array_2_byte(arr):
    return cv2.imencode('.jpg', arr)

class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5


class AppState:

    def __init__(self, *args, **kwargs):
        self.paused = False
        self.decimate = 1
        self.scale = True
        self.color = True


context = zmq.Context()
footage_socket = context.socket(zmq.PUB)
footage_socket.bind('tcp://%s:%s' % (IP, PORT))

state = AppState()

ctx = rs.context()

pipelines = []

for dev in ctx.query_devices():
    pipe = rs.pipeline(ctx)
    cfg =rs.config()

    serial_number = dev.get_info(rs.camera_info.serial_number)
    cfg.enable_device(serial_number)

    if serial_number == '832112073243':
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
        cfg.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    if serial_number == '925122110637':
        cfg.enable_stream(rs.stream.pose)
        cfg.enable_stream(rs.stream.fisheye, 1)
        cfg.enable_stream(rs.stream.fisheye, 2)
    pipe.start(cfg)
    pipelines.append(pipe)

buffer = None

while True:
    try:
        tracking_frames = pipelines[1].wait_for_frames()
        depth_frames = pipelines[0].wait_for_frames()

        color_frame = depth_frames.get_color_frame()
        depth_frame = depth_frames.get_depth_frame()
        fisheye = tracking_frames.get_fisheye_frame(1)
        pose = tracking_frames.get_pose_frame()

        if pose:
            data = pose.get_pose_data()
            pos = data.translation
            position = f'{pos.x}, {pos.y}, {pos.z}'
            footage_socket.send_string('%s %s' % ('pose', position))
            
        if fisheye:
            img = np.asanyarray(fisheye.get_data())
            encoded, buffer = convert_array_2_byte(img)
            footage_socket.send_multipart([b'fisheye', base64.b64encode(buffer)])

        if depth_frame:
            img = np.asanyarray(depth_frame.get_data())
            encoded, buffer = convert_array_2_byte(img)
            footage_socket.send_multipart([b'depth', base64.b64encode(buffer)])

        if color_frame:
            img = np.asanyarray(color_frame.get_data())
            encoded, buffer = convert_array_2_byte(img)
            footage_socket.send_multipart([b'color', base64.b64encode(buffer)])
    
        #time.sleep(INTERVAL)

    except KeyboardInterrupt:
        for pipe in pipelines: pipe.stop()