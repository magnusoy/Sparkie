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
from widgets.manual import ManualWindow
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
    
    def showManualWindow(self):
        """Creates a new manual window in a 
            maximized window."""

        self.manual = ManualWindow()
        self.manual.setWindowIcon(QtGui.QIcon(self.icon))
        self.manual.setFixedSize(1920, 1080)
        self.manual.showMaximized()

    def showPathGenerator(self):
        """Creates a new path generator window."""

        self.path = PathGeneratorWindow()
        self.path.setWindowIcon(QtGui.QIcon(self.icon))
        self.path.setFixedSize(440, 362)
        self.path.show()
    