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
import webbrowser


class WelcomeWindow(QtWidgets.QDialog):
    """doc"""

    switchToAutonomousWindow = QtCore.pyqtSignal()
    switchToManualWindow = QtCore.pyqtSignal()
    switchToInteractWindow = QtCore.pyqtSignal()
    switchToTestingWindow = QtCore.pyqtSignal()
    switchToConfigureWindow = QtCore.pyqtSignal()

    def __init__(self):
        super(WelcomeWindow, self).__init__()
        self.ui = '../forms/welcome.ui'
        uic.loadUi(self.ui, self)

        # Labels
        self.openAutonomousModeLabel = self.findChild(QtWidgets.QLabel, 'openAutonomousModeLabel')
        self.openManualModeLabel = self.findChild(QtWidgets.QLabel, 'openManualModeLabel')
        self.openInteractModeLabel = self.findChild(QtWidgets.QLabel, 'openInteractModeLabel')
        self.openTestingModeLabel = self.findChild(QtWidgets.QLabel, 'openTestingModeLabel')
        self.openConfigureModeLabel = self.findChild(QtWidgets.QLabel, 'openConfigureLabel')
        self.openGetHelpLabel = self.findChild(QtWidgets.QLabel, 'openGetHelpLabel')

        # Triggers
        self.openAutonomousModeLabel.mousePressEvent = self.openAutonomousMode
        self.openManualModeLabel.mousePressEvent = self.openManualMode
        self.openInteractModeLabel.mousePressEvent = self.openInteractMode
        self.openTestingModeLabel.mousePressEvent = self.openTestingMode
        self.openConfigureModeLabel.mousePressEvent = self.openConfigureMode
        self.openGetHelpLabel.mousePressEvent = self.openGetHelp

    def openAutonomousMode(self, event):
        """doc"""
        self.switchToAutonomousWindow.emit()
    
    def openManualMode(self, event):
        """doc"""
        self.switchToManualWindow.emit()

    def openInteractMode(self, event):
        """doc"""
        self.switchToInteractWindow.emit()
    
    def openTestingMode(self, event):
        """doc"""
        self.switchToTestingWindow.emit()

    def openConfigureMode(self, event):
        """doc"""
        self.switchToConfigureWindow.emit()

    def openGetHelp(self, event):
        """doc"""
        webbrowser.open('https://github.com/magnusoy/Sparkie')