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
from time import sleep
from threading import Thread


class SerialCommunication(Thread):
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
            sleep(2)
    
    def connect(self):
        """doc"""
        try:
            self.connection = serial.Serial(self.port, self.baudrate)
            sleep(2)
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


# Example of usage
if __name__ == "__main__":
    se = SerialCommunication("COM3", 115200).start()
    #se.start()
