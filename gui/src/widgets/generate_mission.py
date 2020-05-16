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
from sensor_msgs.msg import Image
from std_msgs.msg import String, UInt8
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback, MoveBaseGoal
from actionlib_msgs.msg import GoalID, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class GenerateMissionWindow(QtWidgets.QDialog):
    """doc"""

    def __init__(self):
        super(GenerateMissionWindow, self).__init__()
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.ui = '../forms/generate_mission.ui'
        self.mission_dir = '../instance/missions'
        loadUi(self.ui, self)
    
    def create_new_mission(self):
        pass

    def add_waypoint(self):
        pass

    def add_orientation(self):
        pass

    def add_equipment(self):
        pass

    def save_mission(self):
        pass

    def generate_mission(self):
        pass

    def close(self):
        pass

    def pose_callback(self):
        pass
