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

from communication.serial_handler import SerialThread

import pyrealsense2 as rs
import numpy as np
import math as m
import json


def msg_2_json(msg):
    data = msg.decode()
    data = eval(data)
    json_msg = json.dumps(data)
    return json.loads(json_msg)


# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipe.start(cfg)

serial = SerialThread(port="COM4", baudrate=921600)
serial.connect()

if __name__ == "__main__":
    while (True):
        try:
            frames = pipe.wait_for_frames()
            pose = frames.get_pose_frame()

            if pose:
                data = pose.get_pose_data()

                w = data.rotation.w
                x = -data.rotation.z
                y = data.rotation.x
                z = -data.rotation.y

                pitch =  -m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi;
                roll  =  m.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / m.pi;
                yaw   =  m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / m.pi;
                
                data = '{"Roll" : {0:.7f}, "Pitch" : {1:.7f}, "Yaw" : {2:.7f}}'.format(roll, pitch, yaw)
                msg = msg_2_json(data)
                serial.sendOutputStream(msg)
                print('RPY [deg]: {}'.format(data))
        finally:
            pipe.stop()
                