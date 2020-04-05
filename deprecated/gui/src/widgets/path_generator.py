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
from PyQt5 import QtWidgets, uic, QtCore
import zmq

from .util.navigation import Waypoint, Path


class PathGeneratorWindow(QtWidgets.QDialog):
    """doc"""

    activate = QtCore.pyqtSignal(bool)
    stop_pose = QtCore.pyqtSignal()

    def __init__(self):
        super(PathGeneratorWindow, self).__init__()
        self.ui = '../forms/path_generator.ui'
        uic.loadUi(self.ui, self)

        self.path = None

        # Push buttons
        self.new_btn = self.findChild(QtWidgets.QPushButton, 'newBtn')
        self.save_btn = self.findChild(QtWidgets.QPushButton, 'saveBtn')
        self.load_btn = self.findChild(QtWidgets.QPushButton, 'loadBtn')
        self.connect_btn = self.findChild(QtWidgets.QPushButton, 'connectBtn')
        
        # Push buttons connections
        self.connect_btn.clicked.connect(self.turn_on)
        self.new_btn.clicked.connect(self.new_path)
        self.load_btn.clicked.connect(self.load_path)
        self.save_btn.clicked.connect(self.save_path)
        
        # Tool buttons
        self.remove_btn = self.findChild(QtWidgets.QToolButton, 'removeBtn')
        self.add_btn = self.findChild(QtWidgets.QToolButton, 'addBtn')

        # Tool buttons connections
        self.remove_btn.clicked.connect(self.remove)
        self.add_btn.clicked.connect(self.add)
            
        # Labels
        self.x_label = self.findChild(QtWidgets.QLabel, 'xLabel')
        self.y_label = self.findChild(QtWidgets.QLabel, 'yLabel')
        self.z_label = self.findChild(QtWidgets.QLabel, 'zLabel')

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.activate.connect(self.turn_on)

        self.pose = PoseThread(self)
        self.stop_pose.connect(self.pose.close_socket)
        self.pose.change_pose.connect(self.change_pose)
        self.pose.start()

    
    def connect(self):
        active = self.connect_btn.isChecked()
        self.activate.emit(active)
        
    def turn_on(self):
        self.pose.activate(True)
    
    def close_window(self):
        self.stop_pose.emit()
        self.close()

    def new_path(self):
        self.path = Path()
        choice = QtWidgets.QMessageBox.question(self, 'Success', 'Create your path!', QtWidgets.QMessageBox.Ok)
        if choice == QtWidgets.QMessageBox.Ok:
            pass

    def load_path(self):
        self.path = Path()
        filename, extension = QtWidgets.QFileDialog.getOpenFileName(self, "Load Path", "",
                                                            "Path files (*.txt *.csv);;All Files (*)")
        if extension == "Path files (*.txt *.csv)":
            self.path.load_path(filename)
            choice = QtWidgets.QMessageBox.question(self, 'Success', 'Path loaded!', QtWidgets.QMessageBox.Ok)
            if choice == QtWidgets.QMessageBox.Ok:
                pass
        else:
            choice = QtWidgets.QMessageBox.question(self, 'Error', 'Wrong file format!', QtWidgets.QMessageBox.Ok)
            if choice == QtWidgets.QMessageBox.Ok:
                pass
    
    def save_path(self):
        if self.path is not None:
            filename, extension = QtWidgets.QFileDialog.getSaveFileName(self, "Load Path", "",
                                                            "Path files (*.txt *.csv);;All Files (*)")
            self.path.save_path(filename)

    @QtCore.pyqtSlot(str)
    def change_pose(self, pose):
        pose = pose.split(',')
        self.x, self.y, self.z = [round(float(x), 2) for x in pose]
        self.x_label.setText(str(self.x))
        self.y_label.setText(str(self.y))
        self.z_label.setText(str(self.z))
    
    def add(self):
        if self.path is not None:
            self.path.add_waypoint(Waypoint(self.x, self.z))

    def remove(self):
        if self.path is not None:
            result = self.path.remove_waypoint(self.path.index)
            if not result:
                choice = QtWidgets.QMessageBox.question(self, 'Error', 'No more waypoints in path!', QtWidgets.QMessageBox.Ok)
                if choice == QtWidgets.QMessageBox.Ok:
                    pass


    

class PoseThread(QtCore.QThread):

    power_on = QtCore.pyqtSignal()
    change_pose = QtCore.pyqtSignal(str)

    IP = '10.10.10.243'
    PORT = 5555

    context = zmq.Context()
    pose_socket = context.socket(zmq.SUB)
    # pose_socket.connect('tcp://10.0.0.121:5555')
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
                #pose = self.pose_socket.recv_string()
                pose = "12.89239,0.232132,124.902901"
                #print(pose)
                self.change_pose.emit(pose)