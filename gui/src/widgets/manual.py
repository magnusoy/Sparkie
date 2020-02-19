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
import cv2
import sys
from PyQt5 import QtWidgets, uic, QtCore, QtGui

class ManualWindow(QtWidgets.QDialog):
    """doc"""

    def __init__(self):
        super(ManualWindow, self).__init__()
        self.ui = '../forms/manual.ui'
        uic.loadUi(self.ui, self)
        self.left = 100
        self.top = 100
        self.width = 640
        self.height = 480
        self.videoFrame = self.findChild(QtWidgets.QLabel, 'videoFrame')
        self.initUI()
    
    @QtCore.pyqtSlot(QtGui.QImage)
    def setImage(self, image):
        self.videoFrame.setPixmap(QtGui.QPixmap.fromImage(image))

    def initUI(self):
        # create a label
        th = Thread(self)
        th.changePixmap.connect(self.setImage)
        th.start()
        self.show()








class Thread(QtCore.QThread):
    changePixmap = QtCore.pyqtSignal(QtGui.QImage)

    def run(self):
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            if ret:
                # https://stackoverflow.com/a/55468544/6622587
                rgbImage = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgbImage.shape
                bytesPerLine = ch * w
                convertToQtFormat = QtGui.QImage(rgbImage.data, w, h, bytesPerLine, QtGui.QImage.Format_RGB888)
                p = convertToQtFormat.scaled(1920, 1010)
                self.changePixmap.emit(p)