#!/usr/bin/env python
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
from __future__ import print_function
from config import *
import rviz
from python_qt_binding import QtWidgets, QtCore, QtGui
from python_qt_binding.binding_helper import *
import cv2
from cv_bridge import CvBridge
import sys
import time
import rospy
import roslib
from sensor_msgs.msg import Image
roslib.load_manifest('rviz')


class ManualWindow(QtWidgets.QDialog):
    """doc"""

    activate = QtCore.pyqtSignal(bool)
    stop_camera = QtCore.pyqtSignal()
    stop_pose = QtCore.pyqtSignal()
    stop_serial = QtCore.pyqtSignal()
    stop_command = QtCore.pyqtSignal()
    stop_xbox_controller = QtCore.pyqtSignal()
    change_camera = QtCore.pyqtSignal(bool)

    def __init__(self):
        super(ManualWindow, self).__init__()
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.ui = '../forms/manual.ui'
        loadUi(self.ui, self)
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)

        self.layout = self.findChild(
            QtWidgets.QGridLayout, 'layout')

        self.mode = JOYSTICK_ONLY_MODE

        self.visual_frame = rviz.VisualizationFrame()
        self.visual_frame.setSplashPath("")
        self.visual_frame.initialize()
        self.add_rviz_config()

        self.visual_frame.setMenuBar(None)
        self.visual_frame.setStatusBar(None)
        self.visual_frame.setHideButtonVisibility(False)

        self.layout.addWidget(self.visual_frame)
        self.visual_frame.hide()

        self.manager = self.visual_frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt(0)

        self.video_frame = QtWidgets.QLabel()
        video_frame_font = QtGui.QFont("Verdana", 62, QtGui.QFont.Bold)
        self.video_frame.setFont(video_frame_font)
        self.video_frame.setText("No video frame")
        self.video_frame.setAlignment(QtCore.Qt.AlignCenter)
        self.video_frame.hide()
        self.layout.addWidget(self.video_frame)

        self.joystick_frame = QtWidgets.QLabel()
        pixmap = QtGui.QPixmap('../static/img/xbox_controller_grey.png')
        pixmap = pixmap.scaledToWidth(800)
        self.joystick_frame.setPixmap(pixmap)
        self.joystick_frame.setAlignment(QtCore.Qt.AlignCenter)
        self.layout.addWidget(self.joystick_frame)

        # Buttons
        self.change_mode_btn = self.findChild(
            QtWidgets.QPushButton, 'changeModeBtn')
        self.turn_left = self.findChild(QtWidgets.QToolButton, 'turnRobotLeft')
        self.turn_right = self.findChild(
            QtWidgets.QToolButton, 'turnRobotRight')
        self.stand_btn = self.findChild(QtWidgets.QPushButton, 'standBtn')
        self.walk_btn = self.findChild(QtWidgets.QPushButton, 'walkBtn')
        self.stairs_btn = self.findChild(QtWidgets.QPushButton, 'stairsBtn')
        self.exit_btn = self.findChild(QtWidgets.QPushButton, 'exitBtn')
        self.powerBtn = self.findChild(QtWidgets.QPushButton, 'powerBtn')
        self.emergency_btn = self.findChild(
            QtWidgets.QPushButton, 'emergencyBtn')
        """
        self.slow_btn = self.findChild(QtWidgets.QPushButton, 'slowBtn')
        self.medium_btn = self.findChild(QtWidgets.QPushButton, 'mediumBtn')
        self.fast_btn = self.findChild(QtWidgets.QPushButton, 'fastBtn')
        """

        # Status indicators
        self.signal_btn = self.findChild(QtWidgets.QPushButton, 'signalBtn')
        self.controller_battery_btn = self.findChild(
            QtWidgets.QPushButton, 'controllerBatteryBtn')
        self.battery_btn = self.findChild(QtWidgets.QPushButton, 'batteryBtn')
        self.health_btn = self.findChild(QtWidgets.QPushButton, 'healthBtn')

        # Button connections
        self.emergency_btn.clicked.connect(self.turn_robot_off)
        self.powerBtn.clicked.connect(self.power_on)
        self.exit_btn.clicked.connect(self.close_window)
        self.change_mode_btn.clicked.connect(self.change_mode)

        """
        self.stand_btn.clicked.connect(self.set_stand_btn)
        self.walk_btn.clicked.connect(self.set_walk_btn)
        self.stairs_btn.clicked.connect(self.set_stairs_btn)
        self.slow_btn.clicked.connect(self.set_slow_btn)
        self.medium_btn.clicked.connect(self.set_medium_btn)
        self.fast_btn.clicked.connect(self.set_fast_btn)
        # self.turn_right.clicked.connect(self.change_camera_output)
        """

        # Button shortcuts
        self.exit_btn.setShortcut("Ctrl+Q")

        # Mode Layouts
        #self.video_frame = self.findChild(QtWidgets.QLabel, 'videoFrame')
        # self.xbox_controller_frame = self.findChild(
        #    QtWidgets.QLabel, 'xboxcontrollerFrame')
        # self.video_frame.hide()

        # Stylesheets
        self.powerBtn.setStyleSheet(
            "QPushButton#powerBtn:checked {color:black; background-color: red;}")
        self.signal_btn.setStyleSheet(
            "QPushButton#signalBtn:checked {color:black; background-color: green;}")
        
        self.video_stream_subscriber = VideoStreamSubscriber('camera/image_raw', Image)

        self.show()

    def add_rviz_config(self):
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        #reader.readFile(config, "../instance/config.viz")
        reader.readFile(config, "../instance/d400_and_t265.rviz")
        self.visual_frame.load(config)

    def change_mode(self):
        if self.mode < MAX_MODES:
            self.mode += 1
        else:
            self.mode = JOYSTICK_ONLY_MODE
        self.update_mode_label()

    def update_mode_label(self):
        if self.mode == 0:
            self.change_mode_btn.setText("JOYSTICK")
            self.visual_frame.hide()
            self.joystick_frame.show()
        elif self.mode == 1:
            self.change_mode_btn.setText("CAMERAS")
            self.joystick_frame.hide()
            self.video_frame.show()
        elif self.mode == 2:
            self.change_mode_btn.setText("VISUAL")
            self.video_frame.hide()
            self.visual_frame.show()

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

    def close_window(self):
        self.close()

    def turn_robot_off(self):
        pass
        


class VideoStreamSubscriber(QtCore.QThread):

    def __init__(self, topic, msg_type):
        QtCore.QThread.__init__(self)
        self.topic = topic
        self.msg_type = msg_type

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data) 
        rgb_image = CvBridge().imgmsg_to_cv2(data, desired_encoding="rgb8")

    def run(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber(self.topic, self.msg_type, self.callback)
        rospy.spin()
