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


class Subscriber(Process):
    """docstring"""

    __slots__ = ['ip', 'port', 'topic']

    def __init__(self, ip, port, topic):
        Process.__init__(self)
        self.ip = ip
        self.port = port
        self.topic = topic
        self.running = True
        self.msg = ''
    
    def run(self):
        """docstring"""

        self.initialize()

        while self.running:
            self.read()
    
    def initialize(self):
        """docstring"""

        self.context = zmq.Context()
        self.address = 'tcp://%s:%s' % (self.ip, self.port)
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(self.address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, self.topic)
    
    def read(self):
        """docstring"""

        self.msg = self.socket.recv_string()
        #print(self.msg)
    
    def stop(self):
        """docstring"""

        self.running = False


class Worker(Subscriber):
    """docstring"""

    def __init__(self, ip, port, topic):
        Subscriber.__init__(self, ip, port, topic)
    
    def run(self):
        """docstring"""

        self.initialize()
        
        while self.running:
            self.read()
            print(self.msg)
            
            
# Example of usage
if __name__ == "__main__":
    sub = Worker('localhost', 5556, 'pose')
    sub.initialize()
    while True:
        sub.read()
    