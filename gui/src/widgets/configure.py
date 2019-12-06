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

from PyQt5 import QtWidgets, uic, QtCore

class ConfigureWindow(QtWidgets.QDialog):
    """doc"""

    def __init__(self):
        super(ConfigureWindow, self).__init__()
        self.ui = '../forms/configure.ui'
        uic.loadUi(self.ui, self)
