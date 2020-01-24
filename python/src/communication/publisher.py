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
import random


class Publisher(Process):
    """docstring"""

    __slots__ = ['ip', 'port', 'topic']

    def __init__(self, ip, port, topic):
        Process.__init__(self)
        self.ip = ip
        self.port = port
        self.topic = topic
        self.running = True
        self.msg = ''

    def initialize(self):
        """docstring"""
        
        self.context = zmq.Context()
        self.address = 'tcp://%s:%s' % (self.ip, self.port)
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(self.address)

    def run(self):
        """docstring"""
        
        self.initialize()
        
        while self.running:
            self.msg = f'{random.randint(0, 1)}'
            self.send(self.msg)
            time.sleep(0.01)

    def send(self, msg):
        """docstring"""
        
        self.socket.send_string('%s %s' % (self.topic, msg))

    def stop(self):
        """docstring"""
        
        self.running = False


class Worker(Publisher):
    """docstring"""

    def __init__(self, ip, port, topic):
        self.__init__(self, ip, port, topic)
    
    def run(self):
        """docstring"""

        self.initialize(self)
        
        while self.running:
            Publisher.send(self)


# Example of usage
if __name__ == "__main__":
    pub = Publisher('*', 5556, '0')
    pub.start()