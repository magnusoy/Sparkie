# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
__author__ = "Magnus Kvendseth Øye"
__copyright__ = "Copyright 2019, Sparkie Quadruped Robot"
__credits__ = ["Magnus Kvendseth Øye", "Petter Drønnen", "Vegard Solheim"]
__version__ = "1.0.0"
__license__ = "MIT"
__maintainer__ = "Magnus Kvendseth Øye"
__email__ = "magnus.oye@gmail.com"
__status__ = "Development"
"""

# Importing from local source
from robot import Sparkie
from communication.serial_handler import SerialCommunication
from communication.server import Server
from util.event_handler import EventHandler
from util.action_handler import ActionHandler


sparkie = Sparkie()
server = Server()
teensy = SerialCommunication(port='COM3', baudrate=115200)
teensy.start()
server.start()

# Running application
if __name__ == "__main__":
    while not teensy.isConnected():
        
        while teensy.isConnected() and server.isConnected():
            if event_handler.incoming:
                sparkie.onEvent(event_handler.event)
            if action_handler.incoming:
                sparkie.onAction(action_handler.event)