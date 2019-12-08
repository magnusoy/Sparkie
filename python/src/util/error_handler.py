# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
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
from threading import Thread
import time

class ErrorHandler(Thread):

    def __init__(self, resource):
        Thread.__init__(self)
        self.errors = None
        self.resources = resource
    
    def listen(self):
        """doc"""
        if 'Error' in self.resource.content.keys():
            self.errors = self.resource.content['Error']
        else:
            self.errors = None
