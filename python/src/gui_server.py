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

import time

# Importing from local source
from communication.subscriber import Subscriber
from communication.server import Server
from config import *


sub = Subscriber(ip='localhost', port=5556, topic='pose')
sub.initialize()

server = Server(host='0.0.0.0', port=8089)
server.initialize()

msg = ""

if __name__ == "__main__":
    while True:
        sub.read()

        server.send(sub.msg)