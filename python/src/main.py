# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
__author__ = "Magnus Kvendseth Øye"
__copyright__ = "Copyright 2019, Sparkie Quadruped Robot"
__credits__ = ["Magnus Kvendseth Øye", "Petter Drønnen", "Vegard Solheim"]
__version__ = "1.0.0"
__license__ = "MIT"
__maintainer__ = "Magnus Kvendseth Øye"
__email__ = "magnus.oye@gmail.com"
__status__ = "Development"
"""

# Importing from local source
from robot import Sparkie
from resources import StorageBox
from util.event_handler import EventHandler
from util.action_handler import ActionHandler
from util.messages import startUpMsg, waitingMsg

# Importing packages
import time

sparkie = Sparkie()
sb = StorageBox()


# Running application
if __name__ == "__main__":
    startUpMsg()
    while True:
        while sb.isInitialized():
            if event_handler.incoming:
                sparkie.onEvent(event_handler.event)
            if action_handler.incoming:
                sparkie.onAction(action_handler.event)
        waitingMsg()
        time.sleep(5)
        