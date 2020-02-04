# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This script combines the TCP/IP Server and the CNN model.
It pares the received input from the client, and sends it
over to the CNN to do its computations before it saves the
result in an file.
"""

# Import packages
import cv2
import os

# Importing model and utility
from object_detection.model import ObjectDetector
from communication.server import RemoteShapeDetectorServer

# Change working directory to get the inference graph and labelmap
os.chdir('C:\\Users\\Petter\\Documents\\Sparkie\\resources')

# Get file paths to frozen inference graph and labelmap
CWD_PATH = os.getcwd()
PATH_TO_CKPT = os.path.join(CWD_PATH, 'model', 'frozen_inference_graph.pb')
PATH_TO_LABELS = os.path.join(CWD_PATH, 'model', 'labelmap.pbtxt')

# Change working directory back to source
os.chdir('C:\\Users\\Petter\\Documents\\Sparkie\\remote\\python\\src')

# Initialize object detector model
object_detector = ObjectDetector(PATH_TO_CKPT, PATH_TO_LABELS)
object_detector.initialize()

# Creates a TCP Server
rsds = RemoteShapeDetectorServer(host="0.0.0.0", port=8089)


# Collects frames recieved from client on server
# Computes the Object detection
# and stores them in file.
if __name__ == "__main__":
    while True:
        frame = rsds.get_frame()

        if len(frame) > 280:  # Check if resolution match
            object_detector.run(frame, debug=False)
        cv2.waitKey(1)
