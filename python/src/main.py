# #!/usr/bin/env python3
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


# Importing from local source
from globals import *
from robot import Sparkie
#from util.listeners import EventListener, ActionListener, WarningListener, ErrorListener
from communication.serial_handler import SerialProcess
from vision.tracking_camera import TrackingCamera
from vision.depth_camera import DepthCamera

# Importing packages
import time


sparkie = Sparkie()
serial = SerialProcess(usb_port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE, interval=100)
tracking_camera = TrackingCamera(image_output=False, ip='*', port=5556, topic='', interval=100)  # topic is blank because of mulitple topics
#depth_camera = DepthCamera(color=False, ip='*', port=5558, topic='', interval=100)  # topic is blank because of mulitple topics


# Running application
if __name__ == "__main__":
    #serial.start()
    tracking_camera.run()
    #depth_camera.start()