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
pub = Publisher(ip=VISION_IP, port=VISION_PORT, topic='')   # topic is blank because of mulitple topics
pub.initialize()

# Tracking camera
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)
cfg.enable_stream(rs.stream.fisheye, 1) # Left camera
cfg.enable_stream(rs.stream.fisheye, 2) # Left camera
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)

profile = pipe.start(cfg)
depth_sensor = profile.get_device().first_depth_sensor()
depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)

state = AppState()

zero_vec = (0.0, 0.0, 0.0)
# Depth camera
depth_frame = zero_vec
color_frame  = zero_vec
depth_colormap = zero_vec

# Tracking camera
pos = zero_vec
vel = zero_vec
acc = zero_vec
img = None
data = None
position = np.zeros((3))

# Processing blocks
pc = rs.pointcloud()
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)
colorizer = rs.colorizer()

if __name__ == "__main__":
    
    while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipe.wait_for_frames()

            left = frames.get_fisheye_frame(1)
            img = np.asanyarray(left.get_data())

            pose = frames.get_pose_frame()

            if pose:
                data = pose.get_pose_data()
                pos = data.translation
                vel = data.velocity
                acc = data.acceleration
                #print('realsense pos(%f, %f, %f)' % (pos.x, pos.y, pos.z))
            
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
            
            msg = img
            pub.topic = 'img'
            pub.send(msg)

            #position = np.array([pos.x, pos.y, pos.z])
            position[0] = pos.x; position[1] = pos.y; position[2] = pos.z;
            #print(position)
            pub.topic = 'pose'
            pub.send(position)
            
            time.sleep(VISION_TRANSMISSION_INTERVAL)

        