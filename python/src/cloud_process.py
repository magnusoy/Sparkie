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

from communication.video_client import VideoClient
from time import sleep

# Subscirber
SUB_IP = 'localhost'
SUB_PORT = 5558
TOPIC = 'img'

# Cload computer
HOST = '83.243.165.82'
PORT = 8089

INTERVAL = 0.1


if __name__ == "__main__":
    vc = VideoClient(sub_ip=SUB_IP, sub_port=SUB_PORT , sub_topic=TOPIC, host=HOST, port=PORT)
    vc.initialize()
    vc.connect()
    while True:
        img = vc.read()
        vc.write(img)
        sleep(INTERVAL)