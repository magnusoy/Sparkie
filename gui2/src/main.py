# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This file runs the desktop program.

Example:
    - python main.py
    Default username: admin
    Default password: sparkie

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
import sys
import qdarkstyle
from PyQt5 import QtWidgets 

# Importing local source
from controller import Controller


# Runs the program
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    controller = Controller()
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    app.exec_()
