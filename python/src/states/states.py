# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This module ...

__author__ = "Magnus Kvendseth Øye"
__copyright__ = "Copyright 2020, Sparkie Quadruped Robot"
__credits__ = ["Magnus Kvendseth Øye", "Petter Drønnen", "Vegard Solheim"]
__version__ = "1.0.0"
__license__ = "MIT"
__maintainer__ = "Magnus Kvendseth Øye"
__email__ = "magnus.oye@gmail.com"
__status__ = "Development"
"""


# Importing from local source
from states.state_interface import State


# Start of the states
class InitState(State):
    """
    The state which indicates that ...
    """

    def onEvent(self, event):
        print("InitState")
        if event:
            return StandbyState()

        return self
    
    def onAction(self, actionEvent):
        """
        docstring
        """
        
        return self
    
    def onWarning(self, warningEvent):
        """
        docstring
        """

        return self
    
    def onError(self, errorWarning):
        """
        docstring
        """

        return self



class StandbyState(State):
    """
    The state which indicates that ...
    """

    def onEvent(self, event):
        print("StandbyState")
        if event:
            return RunningState()

        return self
    
    def onAction(self, actionEvent):
        """
        docstring
        """
        

        return self

    def onWarning(self, warningEvent):
        """
        docstring
        """

        return self
    
    def onError(self, errorWarning):
        """
        docstring
        """

        return self


class RunningState(State):
    """
    The state which indicates that ...
    """

    def onEvent(self, event):
        print("RunningState")
        if event:
            return StandbyState()

        return self
    
    def onAction(self, actionEvent):
        """
        docstring
        """

        return self
    
    def onWarning(self, warningEvent):
        """
        docstring
        """

        return self
    
    def onError(self, errorWarning):
        """
        docstring
        """

        return self