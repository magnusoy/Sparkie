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
import zmq
from multiprocessing import Process
import time


class ServerProcess(Process):
    """docstring"""

    def __init__(self, ip, port):
        Process.__init__(self)
        self.ip = ip
        self.port = port
        self.running = True
        self.msg = ''

    def run(self):
        """docstring"""

        self.initialize()

        while self.running:
            self.read()
            print('server: ' + self.msg)
            self.send(self.msg) 
            if self.msg == 'q':
                self.stop()
    
    def initialize(self):
        """docstring"""
        self.address = 'tcp://%s:%s' % (self.ip, self.port)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(self.address)

    def send(self, msg):
        """docstring"""

        self.socket.send_string(msg)

    def read(self):
        """docstring"""

        self.msg = self.socket.recv().decode()

    def stop(self):
        """docstring"""

        self.running = False

    
class ClientProcess(Process):
    """docstring"""

    def __init__(self, ip, port):
        Process.__init__(self)
        self.ip = ip
        self.port = port
        self.reply = ''

    def run(self):
        """docstring"""

        self.initialize()
        
        for i in range(5):
            msg = 'msg %s' % i
            print('client: ' + msg)
            self.send(msg)
            self.read()

        self.send('q')
        self.read()
    
    def initialize(self):
        """docstring"""

        self.address = 'tcp://%s:%s' % (self.ip, self.port)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(self.address)

    def send(self, msg):
        """docstring"""

        self.socket.send_string(msg)
    
    def read(self):
        """docstring"""

        self.reply = self.socket.recv().decode()


# Example of usage
if __name__ == '__main__':
    server = ServerProcess('*', 65000)
    client = ClientProcess('localhost', 65000)
    server.start()
    time.sleep(3)
    client.start()