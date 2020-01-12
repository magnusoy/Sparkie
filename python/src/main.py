# #!/usr/bin/env python3
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


# Importing from local source
from robot import Sparkie
from util.listeners import EventListener, ActionListener, WarningListener, ErrorListener
from util.messages import startUpMsg, waitingMsg
from experimental.pub_sub import Worker

# Importing packages
import time



sparkie = Sparkie()

worker = Worker('localhost', 5556, '*', 5556, '0')

# Running application
if __name__ == "__main__":
    startUpMsg()
