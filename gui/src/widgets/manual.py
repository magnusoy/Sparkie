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
import numpy as np
import zmq
import base64
from PyQt5 import QtWidgets, uic, QtCore, QtGui

# Importing from local source
from config import *
from .util.subscriber import Subscriber
from .util.client import Client
from .util.xbox_controller import XboxController


class ManualWindow(QtWidgets.QDialog):
    """doc"""
    
    activate = QtCore.pyqtSignal(bool)
    stop_camera  = QtCore.pyqtSignal()
    stop_pose = QtCore.pyqtSignal()
    stop_serial = QtCore.pyqtSignal()
    stop_command = QtCore.pyqtSignal()
    stop_xbox_controller = QtCore.pyqtSignal()
    change_camera = QtCore.pyqtSignal(bool)

    def __init__(self):
        super(ManualWindow, self).__init__()
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.ui = '../forms/manual.ui'
        uic.loadUi(self.ui, self)
        #self.setWindowFlags(QtCore.Qt.FramelessWindowHint)  # Does not work good with Ubuntu
        
        self.mode = JOYSTICK_ONLY_MODE
        
        # Camera filters
        self.current_camera_topic = 0
        self.camera_topics = ('color', 'fisheye', 'depth')
        
        # Buttons
        self.change_mode_btn = self.findChild(QtWidgets.QPushButton, 'changeModeBtn')
        self.turn_left = self.findChild(QtWidgets.QToolButton, 'turnRobotLeft')
        self.turn_right = self.findChild(QtWidgets.QToolButton, 'turnRobotRight')
        self.stand_btn = self.findChild(QtWidgets.QPushButton, 'standBtn')
        self.walk_btn = self.findChild(QtWidgets.QPushButton, 'walkBtn')
        self.stairs_btn = self.findChild(QtWidgets.QPushButton, 'stairsBtn')
        self.exit_btn = self.findChild(QtWidgets.QPushButton, 'exitBtn')
        self.powerBtn = self.findChild(QtWidgets.QPushButton, 'powerBtn')
        self.emergency_btn = self.findChild(QtWidgets.QPushButton, 'emergencyBtn')
        self.slow_btn = self.findChild(QtWidgets.QPushButton, 'slowBtn')
        self.medium_btn = self.findChild(QtWidgets.QPushButton, 'mediumBtn')
        self.fast_btn = self.findChild(QtWidgets.QPushButton, 'fastBtn')
        
        # Status indicators
        self.signal_btn = self.findChild(QtWidgets.QPushButton, 'signalBtn')
        self.controller_battery_btn = self.findChild(QtWidgets.QPushButton, 'controllerBatteryBtn')
        self.battery_btn = self.findChild(QtWidgets.QPushButton, 'batteryBtn')
        self.health_btn = self.findChild(QtWidgets.QPushButton, 'healthBtn')
        
        # Button connections
        self.emergency_btn.clicked.connect(self.turn_robot_off)
        self.powerBtn.clicked.connect(self.power_on)
        self.exit_btn.clicked.connect(self.close_window)
        self.change_mode_btn.clicked.connect(self.change_mode)
        self.stand_btn.clicked.connect(self.set_stand_btn)
        self.walk_btn.clicked.connect(self.set_walk_btn)
        self.stairs_btn.clicked.connect(self.set_stairs_btn)
        self.slow_btn.clicked.connect(self.set_slow_btn)
        self.medium_btn.clicked.connect(self.set_medium_btn)
        self.fast_btn.clicked.connect(self.set_fast_btn)
        self.turn_right.clicked.connect(self.change_camera_output)
        
        #Button shortcuts
        self.exit_btn.setShortcut("Ctrl+Q")
        
        # Mode Layouts
        self.video_frame = self.findChild(QtWidgets.QLabel, 'videoFrame')
        self.xbox_controller_frame = self.findChild(QtWidgets.QLabel, 'xboxcontrollerFrame')
        self.video_frame.hide()
        
        # Stylesheets
        self.powerBtn.setStyleSheet("QPushButton#powerBtn:checked {color:black; background-color: red;}")
        self.signal_btn.setStyleSheet("QPushButton#signalBtn:checked {color:black; background-color: green;}")

        self.initUI()
    
    def change_mode(self):
        if self.mode < MAX_MODES:
            self.mode += 1
        else:
            self.mode = JOYSTICK_ONLY_MODE
        self.update_mode_label()
    
    def update_mode_label(self):
        if self.mode == 0:
            self.change_mode_btn.setText("JOYSTICK ONLY")
            self.video_frame.hide()
            self.xbox_controller_frame.show()
        elif self.mode == 1:
            self.change_mode_btn.setText("ROBOT CAMERAS")
            self.xbox_controller_frame.hide()
            self.video_frame.show()
    
    def set_walk_btn(self):
        if self.walk_btn.isChecked():
            self.stand_btn.setChecked(False)
            self.stairs_btn.setChecked(False)
    
    def set_stand_btn(self):
        if self.stand_btn.isChecked():
            self.walk_btn.setChecked(False)
            self.stairs_btn.setChecked(False)
    
    def set_stairs_btn(self):
        if self.stairs_btn.isChecked():
            self.walk_btn.setChecked(False)
            self.stand_btn.setChecked(False)
    
    def set_slow_btn(self):
        if self.slow_btn.isChecked():
            self.fast_btn.setChecked(False)
            self.medium_btn.setChecked(False)
    
    def set_medium_btn(self):
        if self.medium_btn.isChecked():
            self.slow_btn.setChecked(False)
            self.fast_btn.setChecked(False)
    
    def set_fast_btn(self):
        if self.fast_btn.isChecked():
            self.slow_btn.setChecked(False)
            self.medium_btn.setChecked(False)
    
    @QtCore.pyqtSlot(QtGui.QImage)
    def set_image(self, image):
        self.video_frame.setPixmap(QtGui.QPixmap.fromImage(image))
    
    def power_on(self):
        active = self.powerBtn.isChecked()
        if active:
            self.activate.emit(True)
        else:
            self.activate.emit(False)
    
    def blink_connection(self):
        if not self.signal_btn.isChecked():
            self.signal_btn.setChecked(True)
        else:
            self.signal_btn.setChecked(False)

    def initUI(self):
        self.activate.connect(self.active_all)
        self.init_camera()
        self.init_pose()
        self.init_serial()
        self.init_xbox_controller()
        #self.init_command()
        self.show()
    
    def init_camera(self):
        self.camera = CameraThread(self)
        self.camera.change_pixmap.connect(self.set_image)
        self.camera.blink_signal.connect(self.blink_connection)
        self.stop_camera.connect(self.camera.close_socket)
        self.change_camera.connect(self.change_camera_output)
        self.camera.start()

    def init_pose(self):
        self.pose = PoseThread(self)
        self.stop_pose.connect(self.pose.close_socket)
        self.pose.start()

    def init_serial(self):
        self.serial = SerialThread(self)
        self.stop_serial.connect(self.serial.close_socket)
        self.serial.start()
    
    def init_command(self):
        self.command = CommandThread(self)
        self.stop_command.connect(self.command.close_socket)
        self.command.start()
    
    def init_xbox_controller(self):
        self.xbox_controller = XboxControllerThread()
        self.stop_xbox_controller.connect(self.xbox_controller.close_socket)
        self.xbox_controller.start()
    
    def change_camera_output(self):
        self.current_camera_topic += 1
        self.camera.change_topic(self.camera_topics[self.current_camera_topic])
        if self.current_camera_topic == 2:
            self.current_camera_topic = -1
    
    def active_all(self):
        self.camera.activate(self.powerBtn.isChecked())
        self.pose.activate(self.powerBtn.isChecked())
        self.serial.activate(self.powerBtn.isChecked())
        self.xbox_controller.activate(self.powerBtn.isChecked())
        #self.command.activate(self.powerBtn.isChecked())
    
    def close_window(self):
        self.stop_camera.emit()
        self.stop_pose.emit()
        self.stop_serial.emit()
        self.stop_xbox_controller.emit()
        #self.stop_command.emit()
        self.close()
    
    def turn_robot_off(self):
        pass


class CameraThread(QtCore.QThread):

    blink_signal = QtCore.pyqtSignal()
    power_on = QtCore.pyqtSignal()
    change_pixmap = QtCore.pyqtSignal(QtGui.QImage)
    
    IP = '10.10.10.243'
    PORT = 5555
    FILTER = 'color'
    
    context = zmq.Context()
    footage_socket = context.socket(zmq.SUB)
    footage_socket.connect(f'tcp://{IP}:{PORT}')
    footage_socket.setsockopt_string(zmq.SUBSCRIBE, FILTER)
    footage_socket.setsockopt(zmq.CONFLATE, 1)  # last msg only.
    
    running = False

    active = False
    threadactive = True

    @QtCore.pyqtSlot()
    def close_socket(self):
        self.threadactive = False
        self.footage_socket.disconnect(f'tcp://{self.IP}:{self.PORT}')
        self.running = False
        print("Closing --> Camera")
    
    @QtCore.pyqtSlot(str)
    def change_topic(self, topic):
        self.footage_socket.setsockopt_string(zmq.UNSUBSCRIBE, self.FILTER)
        self.footage_socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        print("Changeing topic")
        self.FILTER = topic
    
    @QtCore.pyqtSlot(bool)
    def activate(self, power_on):
        self.active = power_on
        self.footage_socket.connect(f'tcp://{self.IP}:{self.PORT}')
        self.running = True
    
    def run(self):
        while self.threadactive:
            topic, frame = self.footage_socket.recv_multipart()
            img = base64.b64decode(frame)
            npimg = np.fromstring(img, dtype=np.uint8)
            source = cv2.imdecode(npimg, 1)
            if self.active:
                h, w, ch = source.shape
                bytes_per_line = ch * w
                convert_to_Qt_format = QtGui.QImage(source.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
                p = convert_to_Qt_format.scaled(1920, 1010)
                self.change_pixmap.emit(p)
                self.blink_signal.emit()

 
class PoseThread(QtCore.QThread):

    power_on = QtCore.pyqtSignal()
    
    IP = '10.10.10.243'
    PORT = 5555
    
    context = zmq.Context()
    pose_socket = context.socket(zmq.SUB)
    #pose_socket.connect('tcp://10.0.0.121:5555')
    pose_socket.setsockopt_string(zmq.SUBSCRIBE, 'pose')
    pose_socket.setsockopt(zmq.CONFLATE, 1)  # last msg only.

    active = False
    threadactive = True

    @QtCore.pyqtSlot()
    def close_socket(self):
        self.threadactive = False
        self.pose_socket.disconnect(f'tcp://{self.IP}:{self.PORT}')
        print("Closing --> Pose")
    
    @QtCore.pyqtSlot(bool)
    def activate(self, power_on):
        self.active = power_on
        self.pose_socket.connect(f'tcp://{self.IP}:{self.PORT}')
    
    def run(self):
        while self.threadactive:
            if self.active:
                pose = self.pose_socket.recv_string()
                print(pose)


class SerialThread(QtCore.QThread):

    power_on = QtCore.pyqtSignal()
    
    IP = '10.10.10.243'
    PORT = 6000
    
    context = zmq.Context()
    serial_socket = context.socket(zmq.SUB)
    #serial_socket.connect(f'tcp://{IP}:{PORT}')
    serial_socket.setsockopt_string(zmq.SUBSCRIBE, str(''))
    serial_socket.setsockopt(zmq.CONFLATE, 1)  # last msg only.

    active = False
    threadactive = True

    @QtCore.pyqtSlot()
    def close_socket(self):
        self.threadactive = False
        self.serial_socket.disconnect(f'tcp://{self.IP}:{self.PORT}')
        print("Closing --> Serial")
    
    @QtCore.pyqtSlot(bool)
    def activate(self, power_on):
        self.active = power_on
        self.serial_socket.connect(f'tcp://{self.IP}:{self.PORT}')
    
    def run(self):
        while self.threadactive:
            if self.active:
                data = self.serial_socket.recv_string()
                print(data)


class CommandThread(QtCore.QThread):

    power_on = QtCore.pyqtSignal()
    
    context = zmq.Context()
    command_socket = context.socket(zmq.PUB)
    command_socket.bind('tcp://*:5580')

    command = None
    old_command = None

    active = False
    threadactive = True

    @QtCore.pyqtSlot()
    def close_socket(self):
        self.threadactive = False
        print("Closing --> Command")
    
    @QtCore.pyqtSlot(bool)
    def activate(self, power_on):
        self.active = power_on
    
    @QtCore.pyqtSlot(str)
    def set_command(self, command):
        self.command = command
    
    def run(self):
        while self.threadactive:
            if self.active:
                if self.command is not None and self.command != self.old_command:
                    self.command_socket.send_multipart([b'command', self.command.encode()])
                    print(self.command)
                    self.old_command = self.command


class XboxControllerThread(QtCore.QThread):

    power_on = QtCore.pyqtSignal()

    PORT = 5590 
    
    context = zmq.Context()
    xbox_socket = context.socket(zmq.PUB)
    xbox_socket.bind(f'tcp://*:{PORT}')

    xbox_controller = XboxController(None, deadzone = 30, scale = 100, invertYAxis = True)

    threadactive = True
    active = False

    @QtCore.pyqtSlot()
    def close_socket(self):
        self.threadactive = False
        print("Closing --> Xbox controller")
    
    @QtCore.pyqtSlot(bool)
    def activate(self, power_on):
        self.active = power_on
        print("Active being called")
    
    def run(self):
        while self.threadactive:
            if self.active:
                if self.xbox_controller.check_events():
                    self.xbox_socket.send_multipart([b'xbox_controller', base64.b64encode(str(self.xbox_controller.controlValues).encode())])
                time.sleep(0.1)