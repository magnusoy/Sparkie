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


# Importing PyQT
from PyQt5 import QtGui

# Importing local source
from widgets.login import LoginWindow
from widgets.configure import ConfigureWindow
from widgets.main import MainWindow

class Controller:
    """A class used to control the switching between windows
     within the program. Constrains the windowsizes and assign a Icon to
     each window."""

    def __init__(self):
        """Static icon location, and assigns the login window
        as the first window to be presented."""
        self.icon = '../static/img/favicon/favicon.png'
        self.showLoginWindow()

    def showLoginWindow(self):
        """Displayes the login window, leads to the
        welcome window if event it triggered."""
        self.login = LoginWindow()
        self.login.setWindowIcon(QtGui.QIcon(self.icon))
        self.login.setFixedSize(623, 411)
        self.login.switchToMainWindow.connect(self.showMainWindow)
        self.login.show()
    
    def showConfigureWindow(self):
        """Displays the welcome window, leads to multiple windows,
        depending on the triggers."""
        self.configure = ConfigureWindow()
        self.configure.setWindowIcon(QtGui.QIcon(self.icon))
        self.configure.setFixedSize(720, 400)
        self.welcome.show()
    
    def showMainWindow(self):
        """Displays the main window."""
        self.main = MainWindow()
        self.main.setWindowIcon(QtGui.QIcon(self.icon))
        self.main.setFixedSize(1920, 1080)
        self.login.close()
        self.main.showMaximized()
    