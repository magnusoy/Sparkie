#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Importing package
import socket
from threading import Thread
import time
from .file_handler import FileHandler


class DetectionDataSender(Thread):
    """Client communicating with Server
    through socket connection."""

    def __init__(self, host="0.0.0.0", port=5056, rate=0.5):
        Thread.__init__(self)
        self.addr = (host, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.is_connected = False
        self.rate = rate
        self.terminate = False
        self.command = "POST/Detections"
        self.content = None
        self.data_reader = FileHandler(
            "C:\\Users\\Petter\\Documents\\Sparkie\\resources\\remote\\objects.json")

    def run(self):
        self.connect()
        while not self.terminate:
            data = self.data_reader.read()
            msg = self.command + data
            self.write(msg)
            time.sleep(self.rate)

    def connect(self):
        """Establish a secure connection to server."""
        try:
            self.socket.connect(self.addr)
        except OSError:
            pass
        finally:
            self.is_connected = True

    def disconnect(self):
        """Close connection."""
        self.terminate = True
        Thread.join(self)
        self.socket.close()

    def write(self, msg: str):
        """Write message to server."""
        msg = msg + "\n"
        self.socket.sendall(msg.encode())

    def read(self) -> str:
        """Read received data from server."""
        msg = self.socket.recv(4096)
        return msg.decode("latin-1")


# Example of usage
if __name__ == "__main__":

    object_client = Client("localhost", 5056)

    while True:
        msg = object_client.content
        if msg is not None:
            print(msg.split(","))
