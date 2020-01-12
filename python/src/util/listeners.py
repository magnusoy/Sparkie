# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
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
from communication.subscriber import Subscriber


class Listener(Subscriber):
    """doc"""

    def __init__(self, ip, port, topic):
        Subscriber.__init__(self, ip, port, topic)

    def run(self):
        """doc"""
        yield NotImplementedError
    
    def listen(self):
        """doc"""
        yield NotImplementedError


class EventListener(Listener):
    """doc"""

    def __init__(self, resource):
        Listener.__init__(self)
        self.event = None
        self.resource = resource
    
    def run(self):
        """doc"""
        while self.resource.isInitialized():
            self.listen()
    
    def listen(self):
        """doc"""
        if 'Event' in self.resource.content.keys():
            self.event = self.resource.content['Event']
        else:
            self.event = None


class ActionListener(Listener):
    """doc"""

    def __init__(self, resource):
        Listener.__init__(self)
        self.action = None
        self.resource = resource

    def run(self):
        """doc"""
        while self.resource.isInitialized():
            self.listen()
    
    def listen(self):
        """doc"""
        if 'Action' in self.resource.content.keys():
            self.action = self.resource.content['Action']
        else:
            self.action = None


class WarningListener(Listener):
    """doc"""

    def __init__(self, resource):
        Listener.__init__(self)
        self.resource = resource
        self.warnings = None

    def run(self):
        """doc"""
        while self.resource.isInitialized():
            self.listen()
    
    def listen(self):
        """doc"""
        if 'Warning' in self.resource.content.keys():
            self.warnings = self.resource.content['Warning']
        else:
            self.warnings = None


class ErrorListener(Listener):
    """doc"""

    def __init__(self, resource):
        Listener.__init__(self)
        self.errors = None
        self.resources = resource

    def run(self):
        """doc"""
        while self.resource.isInitialized():
            self.listen()
    
    def listen(self):
        """doc"""
        if 'Error' in self.resource.content.keys():
            self.errors = self.resource.content['Error']
        else:
            self.errors = None

