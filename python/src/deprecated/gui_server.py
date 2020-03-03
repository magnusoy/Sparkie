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

from time import sleep
from enum import IntEnum

# Importing from local source
from communication.subscriber import Subscriber
from communication.server import Server


   

# Subscribers
IP = 'localhost'
TRACKING_PORT = 5556
DEPTH_PORT = 5558
SERIAL_PORT = 5560

# Server
HOST = '0.0.0.0'
PORT = 8089

# Tracking camera
tracking_camera_sub = Subscriber(ip=IP, port=TRACKING_PORT, topic='pose')
tracking_camera_sub.initialize()

# Depth camera
depth_camera_sub = Subscriber(ip=IP, port=DEPTH_PORT, topic='img')
depth_camera_sub.initialize()

# Serial connection
#serial_sub = Subscriber(ip=IP, port=SERIAL_PORT, topic='serial')
#serial_sub.initialize()

# Internal server
server = Server(host=HOST, port=PORT)
server.initialize()


if __name__ == "__main__":
    while True:
        if not server.isConnected():
            server.listening()
        while server.isConnected():
            try:
                tracking_camera_sub.read()
                #server.send(tracking_camera_sub.msg)
                #serial_sub.read()
                #server.send(serial_sub.msg)
                depth_camera_sub.read()
                server.send("HELLO")
            except Exception:
                server.disconnect()
    