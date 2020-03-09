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


TRACKING_IP = '10.0.0.121'
TRACKING_PORT = 5556
INTERVAL_TIME = 0.1

# Publisher
pub = Publisher(ip=TRACKING_IP, port=TRACKING_PORT, topic='tracking')
pub.initialize()

# Tracking camera
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)
pipe.start(cfg)

zero_vec = (0.0, 0.0, 0.0)
pos = zero_vec
vel = zero_vec
acc = zero_vec
data = None
position = ''
velocity = ''
acceleration = ''
tracking = {'Position': None, 'Velocity': None, 'Acceleration': None}


if __name__ == "__main__":
    
    while True:
        try:
            frames = pipe.wait_for_frames()
            pose = frames.get_pose_frame()

            if pose:
                data = pose.get_pose_data()
                pos = data.translation
                vel = data.velocity
                acc = data.acceleration
                position = f'{pos.x}, {pos.y}, {pos.z}'
                velocity = f'{vel.x}, {vel.y}, {vel.z}'
                acceleration = f'{acc.x}, {acc.y}, {acc.z}'
                tracking['Position'] = position
                tracking['Velocity'] = velocity
                tracking['Acceleration'] = acceleration
                #print('realsense pos(%f, %f, %f)' % (pos.x, pos.y, pos.z))

            pub.send(tracking)

            time.sleep(INTERVAL_TIME)
        except Exception as e:
            print(e)
            raise
    
# Stop streaming
pipe.stop()


    
