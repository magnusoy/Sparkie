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
import pyrealsense2 as rs

# Importing from local source
from communication.publisher import Publisher
from config import *


TRACKING_IP = '*'
TRACKING_PORT = 5556
INTERVAL_TIME = 0.1

# Publisher
pub = Publisher(ip=TRACKING_IP, port=TRACKING_PORT, topic='')   # topic is blank because of mulitple topics
pub.initialize()

# Tracking camera
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)
cfg.enable_stream(rs.stream.fisheye, 1) # Left camera
cfg.enable_stream(rs.stream.fisheye, 2) # Left camera
pipe.start(cfg)


zero_vec = (0.0, 0.0, 0.0)
pos = zero_vec
vel = zero_vec
acc = zero_vec
img = None
data = None
position = np.zeros((3))


if __name__ == "__main__":
    
    while True:
        try:

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

            msg = img
            pub.topic = 'img'
            pub.send(msg)

            #position = np.array([pos.x, pos.y, pos.z])
            position[0] = pos.x; position[1] = pos.y; position[2] = pos.z;
            print(position)
            pub.topic = 'pose'
            pub.send(position)

            time.sleep(INTERVAL_TIME)
        except Exception as e:
            print(e)
            raise
    
# Stop streaming
pipe.stop()


    
