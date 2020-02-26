# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

""" Transission interval is set in seconds"""

""" Edge Server """
API_URL = 'http://83.243.165.82:5000/'


""" Depth Camera """
DEPTH_TRANSMISSION_INTERVAL = 0.034
DEPTH_IP = '*'
DEPTH_PORT = 5558


""" Tracking Camera """
TRACKING_TRANSMISSION_INTERVAL = 0.034
TRACKING_IP = '*'
TRACKING_PORT = 5556

""" Teensy Serial """
SERIAL_PORT = 'COM3'
SERIAL_BAUDRATE = 115200