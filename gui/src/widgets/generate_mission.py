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
from python_qt_binding import QtWidgets, QtCore, QtGui
from python_qt_binding.binding_helper import *
import cv2
import numpy as np
import requests
import json
import sys
import time
import rospy
import roslib
import os
import threading
import subprocess
from nav_msgs import Odometry



class GenerateMissionWindow(QtWidgets.QDialog):
    """doc"""

    def __init__(self):
        super(GenerateMissionWindow, self).__init__()
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.ui = '../forms/generate_mission.ui'
        self.mission_dir = '../instance/missions'
        loadUi(self.ui, self)

        #Fields
        self.minAngleField = self.findChild(QtWidgets.QLineEdit, 'minAngleField')
        self.maxAngleField = self.findChild(QtWidgets.QLineEdit, 'maxAngleField')
        self.minValueField = self.findChild(QtWidgets.QLineEdit, 'minValueField')
        self.maxValueField = self.findChild(QtWidgets.QLineEdit, 'maxValueField')

        # Buttons
        self.createNewMissionBtn = self.findChild(QtWidgets.QPushButton, 'createNewMissionBtn')
        self.addWaypointBtn = self.findChild(QtWidgets.QPushButton, 'addWaypointBtn')
        self.addOrientationBtn = self.findChild(QtWidgets.QPushButton, 'addOrientationBtn')
        self.addEquipmentBtn = self.findChild(QtWidgets.QPushButton, 'addEquipmentBtn')
        self.saveMissionBtn = self.findChild(QtWidgets.QPushButton, 'saveMissionBtn')
        self.nextBtn = self.findChild(QtWidgets.QPushButton, 'nextBtn')

        # Button connections
        self.createNewMissionBtn.clicked.connect(self.create_new_mission)
        self.nextBtn.clicked.connect(self.clear_all)
        self.addWaypointBtn.clicked.connect(self.add_waypoint)
        self.addOrientationBtn.clicked.connect(self.add_orientation)
        self.addEquipmentBtn.clicked.connect(self.add_equipment)
        self.saveMissionBtn.clicked.connect(self.save_mission)

        self.equipmentComboBox = self.findChild(QtWidgets.QComboBox, 'equipmentComboBox')
        self.equipmentComboBox.addItems(["None", "gauge", "exit_sign", "fire_extinguisher","closed_valve", "open_valve"])
        self.unitComboBox = self.findChild(QtWidgets.QComboBox, 'unitComboBox')
        self.unitComboBox.addItems(["None", "Bar", "PSI"])

        self.filePath = ''

        self.x_lin = 0.0
        self.y_lin = 0.0
        self.z_lin = 0.0
        self.x_ori = 0.0
        self.y_ori = 0.0
        self.z_ori = 0.0
        self.w_ori = 0.0

        self.mission = {}
        self.missionSelection= {'x_lin': 0.00, 'y_lin': 0.00, 'z_lin': 0.00, 'x_ori': 0.00, 'y_ori': 0.00, 'z_ori': 0.00, 'w_ori': 0.00,
                           'min_angle': -1.00, 'max_angle': -1.00, 'min_value': -1.00, 'max_value': -1.00, 'equipment': -'None', 'units': 'None'}

        # ROS SUBSCRIBERS
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/t265/odom/sample', Odometry, self.pose_callback)

    
    def create_new_mission(self):
        file_path, file_extension = QtWidgets.QFileDialog.getSaveFileName(self, "Create mission", "",
                                                            "Mission files (*.txt);;All Files (*)")
        if len(file_path) > 3:
            with open(file_path, "w") as file:
                file.write('')

        self.filePath = file_path

    def clear_all(self):
        """Add new points to mission"""
        self.mission = {**self.mission, **self.missionSelection}
        self.minAngleField.setText('')
        self.maxAngleField.setText('')
        self.minValueField.setText('')
        self.maxValueField.setText('')

    def add_waypoint(self):
        pass

    def add_orientation(self):
        pass

    def add_equipment(self):
        pass

    def save_mission(self):
        """Save mission as JSON"""
        with open(self.filePath, 'a') as file:
            json.dump(self.mission, file)

    def generate_mission(self):
        pass

    def close(self):
        self.close()

    def pose_callback(self, data):
        self.x_lin = data.pose.pose.position.x
        self.y_lin = data.pose.pose.position.y
        self.z_lin = data.pose.pose.position.z

        self.x_ori = -data.pose.pose.orientation.x
        self.y_ori = data.pose.pose.orientation.y
        self.z_ori = data.pose.pose.orientation.z
        self.w_ori = data.pose.pose.orientation.w

