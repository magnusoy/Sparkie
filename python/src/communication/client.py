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

# Importing package
import socket
from threading import Thread
import time


class Client(Thread):
    """doc"""
    def __init__(self, host='localhost', port=8089, rate=0.2):
        Thread.__init__(self)
        self.address = (host, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.isConnected = False
        self.rate = rate
    
    def run(self):
        """doc"""
        self.connect()
        while self.isConnected:
            # Do something
            pass
    
    def connect(self):
        """doc"""
        try:
            self.socket.connect(self.address)
        except OSError:
            print('Unable to connect')
        finally:
            self.isConnected = True
    
    def disconnect(self):
        """doc"""
        self.socket.close()
        self.isConnected = False
    
    def read(self):
        """doc"""
        payload = self.socket.recv(4096)
        return payload.decode('latin-1')
    
    def write(self, payload):
        """doc"""
        payload = payload + '\n'
        self.socket.sendall(payload.encode())


# Example of usage
if __name__ == "__main__":
    c1 = Client()
    c1.connect()
    while c1.isConnected:
        c1.write('Hello')
    c1.disconnect()