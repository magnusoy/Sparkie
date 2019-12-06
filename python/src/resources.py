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

from communication.server import Server
from communication.serial_handler import SerialCommunication

class StorageBox(object):
    """doc"""

    def __init__(self):
        self.server = Server()
        self.teensy = SerialCommunication('COM3', 115200)
        #self.server.start()
        self.teensy.stat()
        self.content = {}
    
    def initialized(self):
        return self.teensy.isConnected() and self.server.isConnected()

    def put(self, key, content):
        """doc"""
        self.content[key] = content

    def get(self, key):
        """doc"""
        return self.content[key]


if __name__ == "__main__":
    sb = StorageBox()
    while True:
        pass
    