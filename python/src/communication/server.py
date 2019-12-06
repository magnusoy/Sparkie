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

# Import packages
import pickle
import socket
import struct
from threading import Thread


class Server(Thread):
    """doc"""
    def __init__(self, host="0.0.0.0", port=8089):
        Thread.__init__(self)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection = None
        self.addr = None
        self.initialize(host, port)
    
    def run(self):
        """doc"""
        pass
    
    def initialize(self, host="0.0.0.0", port=8089):
        """doc"""
        addr = (host, port)
        self.socket.bind(addr)
        self.socket.listen(1)
        self.connection, self.addr = self.socket.accept()
    
    def isConnected(self):
        """doc"""
        return self.connection is not None
    

# Example of usage
if __name__ == "__main__":
    server = Server()