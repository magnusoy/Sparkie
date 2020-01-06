# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This module ...

__author__ = "Magnus Kvendseth Øye"
__copyright__ = "Copyright 2020, Sparkie Quadruped Robot"
__credits__ = ["Magnus Kvendseth Øye", "Petter Drønnen", "Vegard Solheim"]
__version__ = "1.0.0"
__license__ = "MIT"
__maintainer__ = "Magnus Kvendseth Øye"
__email__ = "magnus.oye@gmail.com"
__status__ = "Development"
"""

# Importing packages
import configparser

# Importing from local source
from communication.server import Server
from communication.serial_handler import Serial

class StorageBox(object):
    """doc"""

    def __init__(self):
        self.config = configparser.ConfigParser()
        self.updateParameters()
        #self.server = Server()
        #self.teensy = Serial(self.port, self.baudrate)
        #self.server.start()
        #self.teensy.start()
        self.content = {}
    
    def updateParameters(self):
        """doc"""
        self.config.read('static/Config.ini')
        self.port = self.config['Serial']['port']
        self.baudrate = self.config['Serial']['baudrate']


    def isRunning(self):
        """doc"""
        #return self.teensy.isConnected() and self.server.isConnected()
        return True

    def put(self, key, content):
        """doc"""
        self.content[key] = content

    def get(self, key):
        """doc"""
        content = None
        try:
            content = self.content[key]
        except KeyError:
            print(f'Key: {key}, is not present')
        return content


# Example of usage
if __name__ == "__main__":
    sb = StorageBox()
    while sb.isRunning():
        sb.put('Event', 0)
        print(sb.get('Event'))
    