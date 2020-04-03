# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This module ...

__author__ = "Magnus Kvendseth Øye"
__copyright__ = "Copyright 2019, Sparkie Quadruped Robot"
__credits__ = ["Magnus Kvendseth Øye", "Petter Drønnen", "Vegard Solheim"]
__version__ = "1.0.0"
__license__ = "MIT"
__maintainer__ = "Magnus Kvendseth Øye"
__email__ = "magnus.oye@gmail.com"
__status__ = "Development"
"""


# Importing packages
import serial
import time
from threading import Thread
from multiprocessing import Process
from .publisher import Publisher
import zmq
import base64
from .util.serializer import *


class SerialThread(Thread):
    """doc"""

    __slots__ = ['port', 'baudrate']

    def __init__(self, port, baudrate=115200):
        """
        Establishes a connection to the given port.
        @port : where your device is connected
        @baudrate : the specified connection speed
                    (9600, 19200, 28800, 57600, 115200)
        """

        Thread.__init__(self)
        self.port = port
        self.baudrate = baudrate
        self.connection = None

    def run(self):
        """doc"""
        while not self.isConnected():
            self.connect()
            time.sleep(2)

    def connect(self):
        """doc"""
        try:
            self.connection = serial.Serial(self.port, self.baudrate)
            time.sleep(2)
            print(f'Established connection to {self.port}')
        except serial.SerialException as se:
            print(f'Cant establish connection to {self.port}')

    def isConnected(self):
        """
        Checks if the connection is established.
        @return False if not connected
                else True
        """

        return self.connection is not None

    def readInputStream(self):
        """
        Read data sent trough Serial.
        @return decoded message
        """

        raw = self.connection.readline()
        content = raw.decode('latin-1')
        return content.rstrip('\n')

    def sendOutputStream(self, data):
        """
        Send data trough Serial.
        """
        content = data + '\n'
        self.connection.write(content.encode())

    def disconnect(self):
        """
        Disconnect the connection
        @return True if sucsessfully disconnected
        """

        self.connection.close()
        return True


class SerialPublisher(Publisher):
    """SerialProcess is a class that extends from Publisher.
    This makes the readInputStream from serial accessible for everyone
    subscribing to the given topic."""

    __slots__ = ['port', 'baudrate']

    def __init__(self, usb_port, baudrate, ip, port, topic):
        Publisher.__init__(self, ip, port, topic)
        self.usb_port = usb_port
        self.baudrate = baudrate
        self.connection = None

    def run(self):
        """doc"""
        while not self.isConnected():
            self.connect()
            time.sleep(2)

        self.initialize()
        while self.running:
            msg = self.readInputStream()
            # print(msg)
            self.send(msg)

    def connect(self):
        """
        Establishes a connection to the given port.
        @usb_port : where your device is connected
        @baudrate : the specified connection speed
                    (9600, 19200, 28800, 57600, 115200)
        """

        try:
            self.connection = serial.Serial(self.usb_port, self.baudrate)
            time.sleep(2)
            print(f'Established connection to {self.usb_port}')
        except serial.SerialException as se:
            print(f'Cant establish connection to {self.usb_port}')

    def isConnected(self):
        """
        Checks if the connection is established.
        @return False if not connected
                else True
        """

        return self.connection is not None

    def readInputStream(self):
        """
        Read data sent trough Serial.
        @return decoded message
        """

        raw = self.connection.readline()
        content = raw.decode('latin-1')
        return content.rstrip('\n')

    def sendOutputStream(self, data):
        """
        Send data trough Serial.
        @data : msg to be sent out
        """

        content = str(data).replace("'", '"')
        self.connection.write(content.encode())

    def disconnect(self):
        """
        Disconnect the connection
        @return True if sucsessfully disconnected
        """

        self.connection.close()
        return True


class SerialProcess(Process):
    """SerialProcess is a class that extends from Process.
    This makes the readInputStream from serial accessible for everyone
    subscribing to the given topic."""

    __slots__ = ['usb_port', 'baudrate', 'interval']

    def __init__(self, usb_port, baudrate, interval):
        Process.__init__(self)
        self.interval = interval
        self.usb_port = usb_port
        self.baudrate = baudrate
        self.connection = None
        self.running = True

        # Subscriber
        self.sub_ip = 'localhost'
        self.sub_port = 5590
        self.sub_topic = 'xbox_controller'

        # Publisher
        self.pub_ip = '*'
        self.pub_port = 6010
        self.pub_topic = 'serial'

    def init_subscriber(self, ip, port, topic):
        """doc"""
        self.sub_ip = ip
        self.sub_port = port
        self.sub_topic = topic

    def init_publisher(self, ip, port, topic):
        """doc"""
        self.pub_ip = ip
        self.pub_port = port
        self.pub_topic = topic

    def run(self):
        """doc"""
        while not self.isConnected():
            self.connect()
            time.sleep(2)
        sub_thread = Thread(target=self.subscriber, args=(
            self.sub_ip, self.sub_port, self.sub_topic), daemon=True)
        pub_thread = Thread(target=self.publisher, args=(
            self.pub_ip, self.pub_port, self.pub_topic), daemon=True)
        pub_thread.start()
        sub_thread.start()
        while self.running:
            time.sleep(10)

    def subscriber(self, ip, port, topic):
        """doc"""
        contex = zmq.Context.instance()
        address = 'tcp://%s:%s' % (ip, port)
        sub = contex.socket(zmq.SUB)
        sub.setsockopt_string(zmq.SUBSCRIBE, topic)
        sub.connect(address)

        while True:
            topic, data = sub.recv_multipart()
            msg = base64.b64decode(data)
            # print(msg)
            msg = msg_2_json(msg)
            self.sendOutputStream(msg)

    def publisher(self, ip, port, topic):
        """doc"""
        contex = zmq.Context.instance()
        address = 'tcp://%s:%s' % (ip, port)
        pub = contex.socket(zmq.PUB)
        pub.bind(address)

        while True:
            msg = self.readInputStream()
            #pub.send_string(f'{topic}, {msg}')
            print(msg)
            # time.sleep(self.interval)

    def connect(self):
        """
        Establishes a connection to the given port.
        @usb_port : where your device is connected
        @baudrate : the specified connection speed
                    (9600, 19200, 28800, 57600, 115200)
        """

        try:
            self.connection = serial.Serial(self.usb_port, self.baudrate)
            time.sleep(1)
            print(f'Established connection to {self.usb_port}')
        except serial.SerialException as se:
            print(f'Cant establish connection to {self.usb_port}')

    def isConnected(self):
        """
        Checks if the connection is established.
        @return False if not connected
                else True
        """

        return self.connection is not None

    def readInputStream(self):
        """
        Read data sent trough Serial.
        @return decoded message
        """

        raw = self.connection.readline()
        content = raw.decode('latin-1')
        return content.rstrip('\n')

    def sendOutputStream(self, data):
        """
        Send data trough Serial.
        @data : msg to be sent out
        """
        # print(data)
        content = str(data).replace("'", '"')  # + '\n'
        self.connection.write(content.encode())

    def disconnect(self):
        """
        Disconnect the connection
        @return True if sucsessfully disconnected
        """

        self.connection.close()
        return True


# Example of usage
if __name__ == "__main__":
    se = SerialProcess("COM4", 921600, 0.1)
    se.start()
