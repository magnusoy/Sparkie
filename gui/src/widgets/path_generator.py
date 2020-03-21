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


class PathGeneratorWindow(QtWidgets.QDialog):
    """doc"""

    def __init__(self):
        super(PathGeneratorWindow, self).__init__()
        self.ui = '../forms/path_generator.ui'
        uic.loadUi(self.ui, self)
    
    def connect(self):
        pass

    def new_path(self):
        pass

    def load_path(self):
        pass
    