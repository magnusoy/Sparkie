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
import rviz
from python_qt_binding import QtWidgets, QtCore, QtGui
from python_qt_binding.binding_helper import *
import cv2
import numpy as np
import requests
import json
from cv_bridge import CvBridge
import sys
import time
import rospy
import roslib
from sensor_msgs.msg import Image
from std_msgs.msg import String, UInt8
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback, MoveBaseGoal
from actionlib_msgs.msg import GoalID, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from threading import Thread

roslib.load_manifest('rviz')


class ManualWindow(QtWidgets.QDialog):
    """doc"""

    activate = QtCore.pyqtSignal(bool)

    def __init__(self):
        super(ManualWindow, self).__init__()
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.ui = '../forms/manual.ui'
        loadUi(self.ui, self)
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)

        self.layout = self.findChild(QtWidgets.QGridLayout, 'layout')

        self.last_goal_ID = None
        self.num_goal_reached = 0

        # Image frames
        self.videoframe = self.findChild(QtWidgets.QLabel, 'videoFrame')
        self.topImageLabel = self.findChild(QtWidgets.QLabel, 'topImageLabel')
        self.middelImageLabel = self.findChild(QtWidgets.QLabel, 'middelImageLabel')
        self.bottomImageLabel = self.findChild(QtWidgets.QLabel, 'bottomImageLabel')

        # Buttons
        self.startVideoStreamBtn = self.findChild(QtWidgets.QPushButton, 'startVideoStreamBtn')
        self.stopVideoStreamBtn = self.findChild(QtWidgets.QPushButton, 'stopVideoStreamBtn')
        self.abortMissionBtn = self.findChild(QtWidgets.QToolButton, 'abortMissionBtn')
        self.inspectStatusBtn = self.findChild(QtWidgets.QPushButton, 'inspectStatusBtn')
        self.pauseMissionBtn = self.findChild(QtWidgets.QToolButton, 'pauseMissionBtn')
        self.stopMissionBtn = self.findChild(QtWidgets.QToolButton, 'stopMissionBtn')
        self.startMissionBtn = self.findChild(QtWidgets.QToolButton, 'startMissionBtn')
        self.refreshSelectMissionAreaBtn = self.findChild(QtWidgets.QToolButton, 'refreshSelectMissionAreaBtn')
        self.refreshSelectMissionBtn = self.findChild(QtWidgets.QToolButton, 'refreshSelectMissionBtn')
        self.inspectStatusBtn.hide()

        # Button connections
        self.startMissionBtn.clicked.connect(self.start_mission)
        self.abortMissionBtn.clicked.connect(self.abort_mission)
        self.pauseMissionBtn.clicked.connect(self.pause_mission)

        # Labels
        self.runninMissionLabel = self.findChild(QtWidgets.QLabel, 'runninMissionLabel')
        self.runningTaskLabel = self.findChild(QtWidgets.QLabel, 'runningTaskLabel')
        self.currentRunningMissionLabel = self.findChild(QtWidgets.QLabel, 'currentRunningMissionLabel')

        # Progressbars
        self.runningTaskProgressBar = self.findChild(QtWidgets.QProgressBar, 'runningTaskProgressBar')

        # Comboboxes
        self.videoSourceComboBox = self.findChild(QtWidgets.QComboBox, 'videoSourceComboBox')
        self.selectMissionAreaComboBox = self.findChild(QtWidgets.QComboBox, 'selectMissionAreaComboBox')
        self.selectMissionComboBox = self.findChild(QtWidgets.QComboBox, 'selectMissionComboBox')

        self.videoSourceComboBox.addItems(["Color", "Fisheye 1", "Fisheye 2","Infared 1", "Infared 2", ])
        self.selectMissionAreaComboBox.addItems(['', "Workshop", "University", "Demo 1","Demo 2", "Demo 3"])
        self.selectMissionComboBox.addItems(['', "Apartment-Mission"])

        # RVIZ
        self.visual_frame = rviz.VisualizationFrame()
        self.visual_frame.setSplashPath("")
        self.visual_frame.initialize()
        self.add_rviz_config()

        self.visual_frame.setMenuBar(None)
        self.visual_frame.setStatusBar(None)
        self.visual_frame.setHideButtonVisibility(True)

        self.layout.addWidget(self.visual_frame)

        self.manager = self.visual_frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt(0)

        self.plan = ['fire_extinguishers','','manometers', 'valves', 'exit_signs']
        self.column_images = [self.topImageLabel, self.middelImageLabel, self.bottomImageLabel]
        self.column_image_counter = 0

        # Tables
        self.tableWidget = self.findChild(QtWidgets.QTableWidget, 'tableWidget')
        self.tableWidget.setHorizontalHeaderLabels(['Time', 'Tag', 'Operation','Status', 'Value', 'Warning', 'Error'])
        header = self.tableWidget.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(2, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(3, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(4, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(5, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(6, QtWidgets.QHeaderView.Stretch)
    
        # Top Buttons
        self.change_mode_btn = self.findChild(
            QtWidgets.QPushButton, 'changeModeBtn')
        self.exit_btn = self.findChild(QtWidgets.QPushButton, 'exitBtn')
        self.powerBtn = self.findChild(QtWidgets.QPushButton, 'powerBtn')
        self.emergency_btn = self.findChild(
            QtWidgets.QPushButton, 'emergencyBtn')

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

        # Button shortcuts
        self.exit_btn.setShortcut("Ctrl+Q")

        # Stylesheets
        self.powerBtn.setStyleSheet(
            "QPushButton#powerBtn:checked {color:black; background-color: red;}")
        self.signal_btn.setStyleSheet(
            "QPushButton#signalBtn:checked {color:black; background-color: green;}")
        
        # ROS SUBSCRIBERS
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/d435/infra1/image_rect_raw', Image, self.image_callback)
        rospy.Subscriber('goal_reached', UInt8, self.result_callback)
        rospy.Subscriber('get_img', UInt8, self.api_callback)

        # Move base ActionListener
        #tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("wait for the action server to come up")
        self.move_base.wait_for_server()

        self.show()

    def add_rviz_config(self):
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, "../instance/Sparkie.rviz")
        self.visual_frame.load(config)

    def start_mission(self):
        choice = QtWidgets.QMessageBox.question(self, 'Warning', 'Start new mission?', QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No) 
        if choice == QtWidgets.QMessageBox.Yes:
            self.startMissionBtn.setDisabled(True)
            self.refreshSelectMissionAreaBtn.setDisabled(True)
            self.refreshSelectMissionBtn.setDisabled(True)
            self.selectMissionAreaComboBox.setDisabled(True)
            self.selectMissionComboBox.setDisabled(True)
            self.runninMissionLabel.setText(self.selectMissionComboBox.currentText())
            self.currentRunningMissionLabel.setText(self.selectMissionComboBox.currentText())
            self.post_goal()
        else:
            pass
    
    def result_callback(self, data):
        if data.data:
            self.num_goal_reached += 1
            print("Goal Reached, ready for new one")

    def image_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data) 
        rgb_image = CvBridge().imgmsg_to_cv2(data, desired_encoding="rgb8")
        height, width, channel = rgb_image.shape
        bytesPerLine = 3 * width
        qImg = QtGui.QImage(rgb_image.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
        self.videoframe.setPixmap(QtGui.QPixmap(qImg))
    
    def api_callback(self, data):
        _class = self.plan[self.num_goal_reached]
        if len(_class) > 3:
            URL = 'http://localhost:5000/%s/1' % _class
            response = requests.get(URL)
            content = response.json()
            IMG = content['img'].encode('latin1')
            rgb_image = np.fromstring(IMG, dtype=np.uint8).reshape((480, 640, 3))
            height, width, channel = rgb_image.shape
            bytesPerLine = 3 * width
            qImg = QtGui.QImage(rgb_image.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
            self.column_images[self.column_image_counter].setPixmap(QtGui.QPixmap(qImg))
            self.column_image_counter += 1
            if self.column_image_counter > 2:
                self.column_image_counter = 0
        else:
            pass    
    def post_goal(self):

        if self.num_goal_reached == 0:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = 0.86
            #goal.target_pose.pose.position.y = 0.5
            goal.target_pose.pose.orientation.w = 1.0
            self.move_base.send_goal(goal)
            print("Sending 0 goal")
        elif self.num_goal_reached == 1:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = 1.31
            goal.target_pose.pose.position.y = -2.25
            goal.target_pose.pose.orientation.z = -0.364
            goal.target_pose.pose.orientation.w = 0.935
            self.move_base.send_goal(goal)
            print("Sending 1 goal")
        elif self.num_goal_reached == 2:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = 2.273
            goal.target_pose.pose.position.y = -4.722
            goal.target_pose.pose.orientation.z = 0.000
            goal.target_pose.pose.orientation.w = -0.538
            self.move_base.send_goal(goal)
            print("Sending 2 goal")
        elif self.num_goal_reached == 3:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = 3.032
            goal.target_pose.pose.position.y = -5.517
            goal.target_pose.pose.orientation.z = -0.316
            goal.target_pose.pose.orientation.w = 0.949
            self.move_base.send_goal(goal)
            print("Sending 3 goal")
        elif self.num_goal_reached == 4:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = 1.789
            goal.target_pose.pose.position.y = -6.490
            goal.target_pose.pose.orientation.z = 0.999
            goal.target_pose.pose.orientation.w = -0.037
            self.move_base.send_goal(goal)
            print("Sending 4 goal")

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

    def abort_mission(self):
        #cancel_move_pub = rospy.Publisher("/move_base/cancel", GoalID)
        x = Thread(target=self.request_server_data)
        x.start()
    
    def pause_mission(self):
        self.num_goal_reached += 1
        self.post_goal()
    
    def request_server_data(self):
        URL = 'http://localhost:5000/manometers/1'
        response = requests.get(URL)
        content = response.json()
        IMG = content['img'].encode('latin1')
        rgb_image = np.fromstring(IMG, dtype=np.uint8).reshape((640, 480, 3))
        height, width, channel = rgb_image.shape
        bytesPerLine = 3 * width
        qImg = QtGui.QImage(rgb_image.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
        self.topImageLabel.setPixmap(QtGui.QPixmap(qImg))

