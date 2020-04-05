# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This interface ...

__author__ = "Magnus Kvendseth Øye"
__copyright__ = "Copyright 2020, Sparkie Quadruped Robot"
__credits__ = ["Magnus Kvendseth Øye", "Petter Drønnen", "Vegard Solheim"]
__version__ = "1.0.0"
__license__ = "MIT"
__maintainer__ = "Magnus Kvendseth Øye"
__email__ = "magnus.oye@gmail.com"
__status__ = "Development"
"""


class State(object):
    """
    Define a state object which provides some utility functions for the
    individual states within the state machine.
    """

    def __init__(self):
        print('Processing current state:', str(self))

    def onEvent(self, event):
        """
        Handle events that are delegated to this State.
        """

        pass

    def onAction(self, actionEvent):
        """
        Handle action events that are delegated to this state.
        """

        pass

    def onWarning(self, warningEvent):
        """
        Handle warning events that are delegated to this state.
        """

        pass

    def onError(self, errorEvent):
        """
        Handle error events that are delegated to this state.
        """

        pass

    def __repr__(self):
        """
        Leverages the __str__ method to describe the State.
        """

        return self.__str__()

    def __str__(self):
        """
        Returns the name of the State.
        """

        return self.__class__.__name__