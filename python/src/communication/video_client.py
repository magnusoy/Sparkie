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

# Importing from local source
from subscriber import Subscriber


class VideoClient(Subscriber):
    def __init__(self, sub_ip, sub_port, sub_topic, host, port):
        Subscriber.__init__(self, sub_port, sub_topic, sub_topic)
        self.address = (host, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.isConnected = False
        self.img = None
        self.response = None
    
    def run(self):
        """docstring"""

        self.initialize()
        self.connect()
        
        while self.running and self.isConnected:
            self.read()
            self.img = self.msg
            if self.img is not None and self.response is not None:
                self.write(self.img)
                self.response = None
            self.response = self.get_response()

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
    
    def get_response(self):
        """doc"""
        payload = self.socket.recv(4096)
        return payload.decode('latin-1')
    
    def write(self, payload):
        """doc"""
        payload = payload + '\n'
        self.socket.sendall(payload.encode())


if __name__ == "__main__":
    vc = VideoClient()