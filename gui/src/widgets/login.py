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
import configparser

class LoginWindow(QtWidgets.QDialog):
    """doc"""

    switchToWelcomeWindow = QtCore.pyqtSignal()
    
    

    def __init__(self):
        super(LoginWindow, self).__init__()
        self.ui = '../forms/login.ui'
        uic.loadUi(self.ui, self)

        #Read config file
        self.config = configparser.ConfigParser()
        self.config.read('../instance/config.ini')

        #Button
        self.loginBtn = self.findChild(QtWidgets.QPushButton, 'loginBtn')

        # Checkbox
        self.rememberMeBox = self.findChild(QtWidgets.QCheckBox, 'rememberMeCheckBox')

        # Text fields
        self.usernameField = self.findChild(QtWidgets.QLineEdit, 'usernameField')
        self.passwordField = self.findChild(QtWidgets.QLineEdit, 'passwordField')
        
        # Button click functions
        self.loginBtn.clicked.connect(self.login)

        # Remember function
        self.usernameField.setText(self.config.get('Login','username'))
        if self.config.getboolean('Login','rememberMeBox'):
            self.rememberMeBox.setChecked(True)


    def login(self):
        """Checks the given username and password agains the correct values.
            Will give an error notification if wrong credentials is tried,
            otherwise redirected to the welcome window."""
        
        self.rememberMe()
        if self.usernameField.text() == 'admin' and self.passwordField.text() == '':
            self.switchToWelcomeWindow.emit()

        else:
            choice = QtWidgets.QMessageBox.question(self, 'Error', 'Wrong password, please try again.', QtWidgets.QMessageBox.Ok)
            if choice == QtWidgets.QMessageBox.Ok:
                pass
            
                    
    def rememberMe(self):
        """Checks if the remember me box is checked and then write the corresponding 
        username and valvue of the rememberMeBox to the config file"""
        if self.rememberMeBox.isChecked():
            self.config.set('Login','username',self.usernameField.text())
            self.config.set('Login','rememberMeBox','true')
        else:
            self.config.set('Login','username','')
            self.config.set('Login','rememberMeBox','false')
        self.writeToConfig()
        
    def writeToConfig(self):
        with open('../instance/config.ini', "w+") as configfile:
                self.config.write(configfile)