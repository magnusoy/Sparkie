#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This file runs the desktop program.

Example:
    - python main.py

__author__ = "Magnus Kvendseth Øye"
__copyright__ = "Copyright 2020, Sparkie Quadruped Robot"
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
from python_qt_binding import QtWidgets

# Importing local source
from controller import Controller


# Runs the program
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    controller = Controller()
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    app.exec_()
