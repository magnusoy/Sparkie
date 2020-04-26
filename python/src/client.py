# !/usr/bin/env python3
# -*- coding: utf-8 -*-

# Importing packages
import cv2
import socket
import pickle
import struct
import time
import os


class Client():
    """Client that connect and send frames to remote computer."""

    def __init__(self, host="127.0.0.1", port=8089):
        self.addr = (host, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.is_connected = False

    def send(self, img):
        """Convert and send frame as bytes."""
        frame = cv2.imread(img)
        # Create a region of interest that excludes the sorted zone
        data = pickle.dumps(frame)
        # Change to "L" if windows <-> Windows
        message_size = struct.pack("=L", len(data))
        self.write(message_size + data)
        # _, jpeg = cv2.imencode('.jpg', frame)

    def convert_to_jpeg(self, frame):
        _, jpeg = cv2.imencode('.jpg', frame)
        return jpeg.tobytes()

    def connect(self):
        """Establish a secure connection to server."""
        try:
            self.socket.connect(self.addr)
        except OSError:
            pass
        finally:
            self.is_connected = True
            time.sleep(2)

    def disconnect(self):
        """Close connection."""
        self.socket.close()
        self.is_connected = False

    def write(self, msg: str):
        """Write message to server."""
        self.socket.sendall(msg)

    def read(self) -> str:
        """Read received data from server."""
        msg = self.socket.recv(4096)
        return msg.decode("latin-1")


def get_image(directory):
    imgs = [file for file in os.listdir(directory)]
    if len(imgs) > 0:
        return imgs[-1]
    else:
        return None


def remove_image(directory):
    imgs = [file for file in os.listdir(directory)]
    if len(imgs) > 0:
        os.remove(imgs[-1])


client = Client(host="127.0.0.1", port=8089)

IMG_DIRECTORY = './img/tmp/'


if __name__ == "__main__":
    client.connect()
    if client.is_connected:
        img = get_image(IMG_DIRECTORY)
        if img is not None:
            client.send(os.path.join(IMG_DIRECTORY, img))
            # remove_image(IMG_DIRECTORY)
        client.disconnect()
