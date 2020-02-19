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
import time
from PyQt5 import QtWidgets, uic, QtCore, QtGui

# Importing from local source
from globals import *

class ManualWindow(QtWidgets.QDialog):
    """doc"""
    
    activate = QtCore.pyqtSignal(bool)
    stop_video_stream = QtCore.pyqtSignal()

    def __init__(self):
        super(ManualWindow, self).__init__()
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.ui = '../forms/manual.ui'
        uic.loadUi(self.ui, self)
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        
        self.mode = JOYSTICK_ONLY_MODE
        
        self.change_mode_btn = self.findChild(QtWidgets.QPushButton, 'changeModeBtn')
        self.turn_left = self.findChild(QtWidgets.QToolButton, 'turnRobotLeft')
        self.turn_right = self.findChild(QtWidgets.QToolButton, 'turnRobotRight')
        self.video_frame = self.findChild(QtWidgets.QLabel, 'videoFrame')
        self.exitBtn = self.findChild(QtWidgets.QPushButton, 'exitBtn')
        self.powerBtn = self.findChild(QtWidgets.QPushButton, 'powerBtn')
        self.emergencyBtn = self.findChild(QtWidgets.QPushButton, 'emergencyBtn')
        self.emergencyBtn.clicked.connect(self.turn_robot_off)
        self.powerBtn.clicked.connect(self.power_on)
        self.exitBtn.setShortcut("Ctrl+Q")
        self.exitBtn.clicked.connect(self.close_window)
        
        self.change_mode_btn.clicked.connect(self.change_mode)
        
        self.powerBtn.setStyleSheet("QPushButton#powerBtn:checked {color:black; background-color: red;}")
        self.initUI()
    
    def change_mode(self):
        if self.mode < MAX_MODES:
            self.mode += 1
        else:
            self.mode = JOYSTICK_ONLY_MODE
        self.update_mode_label()
    
    def update_mode_label(self):
        if self.mode == 0:
            self.change_mode_btn.setText("JOYSTICK ONLY MODE")
        elif self.mode == 1:
            self.change_mode_btn.setText("ROBOT CAMERAS")
            
    
    @QtCore.pyqtSlot(QtGui.QImage)
    def set_image(self, image):
        self.video_frame.setPixmap(QtGui.QPixmap.fromImage(image))
    
    def power_on(self):
        active = self.powerBtn.isChecked()
        self.activate.emit(active)

    def initUI(self):
        self.video_stream = VideoThread(self)
        self.video_stream.change_pixmap.connect(self.set_image)
        self.activate.connect(self.video_stream.activate)
        self.stop_video_stream.connect(self.video_stream.stop)
        self.video_stream.start()
        self.show()
    
    def close_window(self):
        self.stop_video_stream.emit()
        self.video_stream.stop()
        self.close()
    
    def turn_robot_off(self):
        pass


class VideoThread(QtCore.QThread):
    change_pixmap = QtCore.pyqtSignal(QtGui.QImage)
    power_on = QtCore.pyqtSignal()
    
    active = False
    threadactive = True
    
    @QtCore.pyqtSlot(bool)
    def activate(self, power_on):
        self.active = power_on
    
    @QtCore.pyqtSlot()
    def stop(self):
        self.threadactive = False
        self.wait()

    def run(self):
        cap = cv2.VideoCapture(0)
        while self.threadactive:
            if self.active:
                ret, frame = cap.read()
                if ret:
                    rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    h, w, ch = rgb_image.shape
                    bytes_per_line = ch * w
                    convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
                    p = convert_to_Qt_format.scaled(1920, 1010)
                    self.change_pixmap.emit(p)
            else:
                time.sleep(0.3)
            key = cv2.waitKey(1)
        cap.release()
        cv2.destroyAllWindows()