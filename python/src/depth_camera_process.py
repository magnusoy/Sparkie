# -*- coding: utf-8 -*-

"""
__author__ = "Magnus Kvendseth Øye"
__copyright__ = "Copyright 2020, Sparkie Quadruped Robot"
__credits__ = ["Magnus Kvendseth Øye", "Petter Drønnen", "Vegard Solheim"]
__version__ = "1.0.0"
__license__ = "MIT"
__maintainer__ = "Magnus Kvendseth Øye"
__email__ = "magnus.oye@gmail.com"
__status__ = "Development"
"""

import cv2
import time
import numpy as np
from enum import IntEnum
import pyrealsense2 as rs

# Importing from local source
from communication.publisher import Publisher
from globals import *


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


# Publisher
pub = Publisher(ip=DEPTH_IP, port=DEPTH_PORT, topic='')   # topic is blank because of mulitple topics
pub.initialize()

state = AppState()

# Depth camera
try:
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
except Exception as e:
    print("No device connected")
    raise

profile = pipe.start(cfg)
depth_sensor = profile.get_device().first_depth_sensor()
depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)

zero_vec = (0.0, 0.0, 0.0)
depth_frame = zero_vec
color_frame  = zero_vec
depth_colormap = zero_vec

# Processing blocks
pc = rs.pointcloud()
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)
colorizer = rs.colorizer()


if __name__ == "__main__":
    
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipe.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_frame = decimate.process(depth_frame)

        # Grab new intrinsics (may be changed by decimation)
        depth_intrinsics = rs.video_stream_profile(
            depth_frame.profile).get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_colormap = np.asanyarray(
            colorizer.colorize(depth_frame).get_data())
        
        if state.color:
            mapped_frame, color_source = color_frame, color_image
        else:
            mapped_frame, color_source = depth_frame, depth_colormap

        points = pc.calculate(depth_frame)
        pc.map_to(mapped_frame)

        # Pointcloud data to arrays
        v, t = points.get_vertices(), points.get_texture_coordinates()
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
        texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

        msg = depth_image
        pub.topic = 'depth'
        pub.send(msg)

        msg = color_image
        pub.topic = 'img'
        pub.send(msg)
        print(msg)

        msg = depth_colormap
        pub.topic = 'colormap'
        pub.send(msg)

        time.sleep(DEPTH_TRANSMISSION_INTERVAL)
    
# Stop streaming
pipe.stop()
