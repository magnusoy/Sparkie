# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Import packages
import pickle
import socket
import struct
import cv2


class RemoteShapeDetectorServer:
    """This class is a TCP Server used to 
    collect incoming frames from Jetson Nano."""

    def __init__(self, host="0.0.0.0", port=8089):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection = None
        self.addr = None
        self.initialize(host, port)
        self.data = b''
        self.payload_size = struct.calcsize("L")

    def initialize(self, host="0.0.0.0", port=8089):
        """Initializes the server."""
        addr = (host, port)
        self.socket.bind(addr)
        self.socket.listen(10)
        self.connection, self.addr = self.socket.accept()

    def get_frame(self):
        """Recieves frames as incoming bytestreams.
        Returns videoframe from stream."""
        while len(self.data) < self.payload_size:
            self.data += self.connection.recv(4096)

        packed_msg_size = self.data[:self.payload_size]
        self.data = self.data[self.payload_size:]
        # Change to "L" if windows <-> Windows
        msg_size = struct.unpack("=L", packed_msg_size)[0]

        while len(self.data) < msg_size:
            self.data += self.connection.recv(4096)

        frame_data = self.data[:msg_size]
        self.data = self.data[msg_size:]
        frame = pickle.loads(frame_data, encoding='latin1')
        return frame


# Example of usage
if __name__ == "__main__":
    rsds = RemoteShapeDetectorServer(host="0.0.0.0", port=8089)

    while True:
        frame = rsds.get_frame()

        cv2.imshow('frame', frame)
        cv2.waitKey(1)
