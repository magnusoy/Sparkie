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

class LoginWindow(QtWidgets.QDialog):
    """doc"""

    switchToMainWindow = QtCore.pyqtSignal()

    def __init__(self):
        super(LoginWindow, self).__init__()
        self.ui = '../forms/login.ui'
        uic.loadUi(self.ui, self)

        #Button
        self.loginBtn = self.findChild(QtWidgets.QPushButton, 'loginBtn')

        # Checkbox
        self.rememberMeBox = self.findChild(QtWidgets.QCheckBox, 'rememberMeCheckBox')

        # Text fields
        self.usernameField = self.findChild(QtWidgets.QLineEdit, 'usernameField')
        self.passwordField = self.findChild(QtWidgets.QLineEdit, 'passwordField')
        
        # Button click functions
        self.loginBtn.clicked.connect(self.login)


    def login(self):
        """doc"""
        if self.usernameField.text() == 'admin' and self.passwordField.text() == 'sparkie':
            self.switchToMainWindow.emit()
        else:
            choice = QtWidgets.QMessageBox.question(self, 'Error', 'Wrong password, please try again.', QtWidgets.QMessageBox.Ok)
            if choice == QtWidgets.QMessageBox.Ok:
                pass