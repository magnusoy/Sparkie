# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This module controls the window management in the program.
Based on triggers, the windows will be displayed or closed accordingly.

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
from PyQt5 import QtGui

# Importing local source
from widgets.login import LoginWindow
from widgets.welcome import WelcomeWindow
from widgets.configure import ConfigureWindow
from widgets.autonomous import AutonomousWindow
from widgets.manual import ManualWindow
from widgets.interact import InteractWindow
from widgets.testing import TestingWindow
from widgets.path_generator import PathGeneratorWindow


class Controller:
    """A class used to control the switching between windows
     within the program. Constrains the windowsizes and assign a Icon to
     each window."""

    def __init__(self):
        """Static icon location, and assigns the login window
        as the first window to be presented."""

        self.icon = '../static/img/favicon/favicon.png'
        self.showManualWindow()

    def showLoginWindow(self):
        """Creates a new login window, and connects an
            emitter to open the welcome window."""

        self.login = LoginWindow()
        self.login.setWindowIcon(QtGui.QIcon(self.icon))
        self.login.setFixedSize(732, 440) 
        self.login.switchToWelcomeWindow.connect(self.showWelcomeWindow)
        self.login.show()
    
    def showWelcomeWindow(self):
        """Creates a new welcome window, and connects multiple
            emitters to open different windows."""

        self.welcome = WelcomeWindow()
        self.welcome.setWindowIcon(QtGui.QIcon(self.icon))
        self.welcome.setFixedSize(623, 411)
        self.welcome.switchToAutonomousWindow.connect(self.showAutonomousWindow)
        self.welcome.switchToManualWindow.connect(self.showManualWindow)
        self.welcome.switchToInteractWindow.connect(self.showInteractWindow)
        self.welcome.switchToTestingWindow.connect(self.showTestingWindow)
        self.welcome.switchToConfigureWindow.connect(self.showConfigureWindow)
        self.login.close()
        self.welcome.show()
    
    def showAutonomousWindow(self):
        """Creates a new autonomous window in a 
            maximized window."""

        self.autonomous = AutonomousWindow()
        self.autonomous.setWindowIcon(QtGui.QIcon(self.icon))
        self.autonomous.setFixedSize(1920, 1080)
        self.autonomous.showMaximized()
    
    def showManualWindow(self):
        """Creates a new manual window in a 
            maximized window."""

        self.manual = ManualWindow()
        self.manual.setWindowIcon(QtGui.QIcon(self.icon))
        self.manual.setFixedSize(1920, 1080)
        self.manual.showMaximized()

    def showInteractWindow(self):
        """Creates a new interact window in a 
            maximized window."""

        self.interact = InteractWindow()
        self.interact.setWindowIcon(QtGui.QIcon(self.icon))
        self.interact.setFixedSize(1920, 1080)
        self.interact.showMaximized()

    def showTestingWindow(self):
        """Creates a new autonomous window."""

        self.testing = TestingWindow()
        self.testing.setWindowIcon(QtGui.QIcon(self.icon))
        self.testing.setFixedSize(623, 411)
        self.testing.show()
    
    def showConfigureWindow(self):
        """Creates a new autonomous window."""

        self.configure = ConfigureWindow()
        self.configure.setWindowIcon(QtGui.QIcon(self.icon))
        self.configure.setFixedSize(720, 400)
        self.configure.show()
    
    def showPathGenerator(self):
        """Creates a new path generator window."""

        self.path = PathGeneratorWindow()
        self.path.setWindowIcon(QtGui.QIcon(self.icon))
        self.path.setFixedSize(440, 362)
        self.path.show()
    