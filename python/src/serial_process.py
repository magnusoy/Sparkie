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

from communication.serial_handler import SerialProcess


SERIAL_PORT = "COM3"
SERIAL_BAUDRATE = 115200
SERIAL_INTERVAL = 0.1

serial = SerialProcess(usb_port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE, interval=SERIAL_INTERVAL)


if __name__ == "__main__":
    serial.start()