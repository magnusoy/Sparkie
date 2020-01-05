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
from util.listeners import EventListener, ActionListener, WarningListener, ErrorListener
from util.messages import startUpMsg, waitingMsg

# Importing packages
import time
from multiprocessing import Process


sparkie = Sparkie()
sb = StorageBox()

# Listeners
events = EventListener(sb).start()
actions = ActionListener(sb).start()
warnings = WarningListener(sb).start()
errors = ErrorListener(sb).start()


# Running application
if __name__ == "__main__":
    startUpMsg()
    while True:
        while sb.isRunning():
            sparkie.onEvent(sb.get('Event'))
            sparkie.onAction(sb.get('Action'))
            sparkie.onWarning(sb.get('Warning'))
            sparkie.onError(sb.get('Error'))

        waitingMsg()
        time.sleep(5)
        