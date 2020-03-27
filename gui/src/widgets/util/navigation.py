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

# Importing package
from os import path

class Waypoint(object):

    def __init__(self, x=0, z=0, action=0):
        if type(x) is str:
            self.x = float(x)
            self.z = float(z)
        else:        
            self.x = x
            self.z = z
        self.action = action
    
    def __repr__(self):
        return f'{self.x},{self.z},{self.action}'
    

class Path(object):
    
    def __init__(self):
        self.waypoints = []
        self.index = 0
    
    def __repr__(self):
        return f'Size: {len(self.waypoints)} | Waypoints: {self.waypoints}'
    
    def increment(self, n=1):
        self.index += n
    
    def decrement(self, n=1):
        self.index -= n
    
    def add_waypoint(self, waypoint):
        self.waypoints.append(waypoint)
        self.increment()
    
    def remove_waypoint(self, index):
        try:
            self.waypoints.pop(index)
            self.decrement()
            return True;
        except IndexError:
            return False;
    
    def clear_waypoints(self):
        self.waypoints.clear()
    
    def copy_waypoints(self):
        return self.waypoints.copy()
    
    def reverse_waypoints(self):
        self.waypoints.reverse()
    
    def get_current_waypoint(self):
        return self.waypoints[self.index]
    
    def get_previous_waypoint(self):
        if self.index > 0:
            return self.waypoints[self.index-1]
        else:
            return None
    
    def get_next_waypoint(self):
        if self.index < len(self.waypoints):
            return self.waypoints[self.index+1]
        else:
            return None
    
    def save_path(self, filename, overwrite=False):
        with open(filename, "w") as f:
            for waypoint in self.waypoints:
                f.write(f'{waypoint}\n')
            
    def load_path(self, filename):
        with open(filename, 'r') as f:
            lines = f.readlines()
            for line in lines:
                x, z = line.split(',')
                self.waypoints.append(Waypoint(x, z))



if __name__ == "__main__":
    path = Path()
    path.add_waypoint(Waypoint(0, 0))
    path.add_waypoint(Waypoint(10, 10))
    path.add_waypoint(Waypoint(30, 12))
    path.save_path("path.txt")

    new_path = Path()
    new_path.load_path("path.txt")
    new_path.add_waypoint(Waypoint(20, 10))
    #print(new_path)
    print(new_path.get_current_waypoint())
