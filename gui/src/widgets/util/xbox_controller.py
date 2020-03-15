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


import os, sys
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame
from pygame.locals import *
import threading
import time

"""
NOTES - pygame events and values
JOYAXISMOTION
event.axis              event.value
0 - x axis left thumb   (+1 is right, -1 is left)
1 - y axis left thumb   (+1 is down, -1 is up)
2 - x axis right thumb  (+1 is right, -1 is left)
3 - y axis right thumb  (+1 is down, -1 is up)
4 - right trigger
5 - left trigger
JOYBUTTONDOWN | JOYBUTTONUP
event.button
A = 0
B = 1
X = 2
Y = 3
LB = 4
RB = 5
BACK = 6
START = 7
XBOX = 8
LEFTTHUMB = 9
RIGHTTHUMB = 10
JOYHATMOTION
event.value
[0] - horizontal
[1] - vertival
[0].0 - middle
[0].-1 - left
[0].+1 - right
[1].0 - middle
[1].-1 - bottom
[1].+1 - top
"""


class XboxController(threading.Thread):

    # Internal ids for the xbox controls
    class XboxControls():
        LTHUMBX = 0
        LTHUMBY = 1
        RTHUMBX = 2
        RTHUMBY = 3
        RTRIGGER = 4
        LTRIGGER = 5
        A = 6
        B = 7
        X = 8
        Y = 9
        LB = 10
        RB = 11
        BACK = 12
        START = 13
        XBOX = 14
        LEFTTHUMB = 15
        RIGHTTHUMB = 16
        DPAD = 17

    # Pygame axis constants for the analogue controls of the xbox controller
    class PyGameAxis():
        LTHUMBX = 0
        LTHUMBY = 1
        RTHUMBX = 2
        RTHUMBY = 3
        RTRIGGER = 4
        LTRIGGER = 5

    # Pygame constants for the buttons of the xbox controller
    class PyGameButtons():
        A = 0
        B = 1
        X = 2
        Y = 3
        LB = 4
        RB = 5
        BACK = 6
        START = 7
        XBOX = 8
        LEFTTHUMB = 9
        RIGHTTHUMB = 10

    # Map between pygame axis (analogue stick) ids and xbox control ids
    AXISCONTROLMAP = {PyGameAxis.LTHUMBX: XboxControls.LTHUMBX,
                      PyGameAxis.LTHUMBY: XboxControls.LTHUMBY,
                      PyGameAxis.RTHUMBX: XboxControls.RTHUMBX,
                      PyGameAxis.RTHUMBY: XboxControls.RTHUMBY}
    
    # Map between pygame axis (trigger) ids and xbox control ids
    TRIGGERCONTROLMAP = {PyGameAxis.RTRIGGER: XboxControls.RTRIGGER,
                         PyGameAxis.LTRIGGER: XboxControls.LTRIGGER}

    # Map between pygame buttons ids and xbox contorl ids
    BUTTONCONTROLMAP = {PyGameButtons.A: XboxControls.A,
                        PyGameButtons.B: XboxControls.B,
                        PyGameButtons.X: XboxControls.X,
                        PyGameButtons.Y: XboxControls.Y,
                        PyGameButtons.LB: XboxControls.LB,
                        PyGameButtons.RB: XboxControls.RB,
                        PyGameButtons.BACK: XboxControls.BACK,
                        PyGameButtons.START: XboxControls.START,
                        PyGameButtons.XBOX: XboxControls.XBOX,
                        PyGameButtons.LEFTTHUMB: XboxControls.LEFTTHUMB,
                        PyGameButtons.RIGHTTHUMB: XboxControls.RIGHTTHUMB}
                        
    def __init__(self,
                 controllerCallBack = None,
                 joystickNo = 0,
                 deadzone = 0.1,
                 scale = 1,
                 invertYAxis = False):

        #setup threading
        threading.Thread.__init__(self)
        
        #persist values
        self.running = False
        self.connected = False
        self.controllerCallBack = controllerCallBack
        self.joystickNo = joystickNo
        self.lowerDeadzone = deadzone * -1
        self.upperDeadzone = deadzone
        self.scale = scale
        self.invertYAxis = invertYAxis
        self.controlCallbacks = {}

        #setup controller properties
        self.controlValues = {self.XboxControls.LTHUMBX:0,
                              self.XboxControls.LTHUMBY:0,
                              self.XboxControls.RTHUMBX:0,
                              self.XboxControls.RTHUMBY:0,
                              self.XboxControls.RTRIGGER:0,
                              self.XboxControls.LTRIGGER:0,
                              self.XboxControls.A:0,
                              self.XboxControls.B:0,
                              self.XboxControls.X:0,
                              self.XboxControls.Y:0,
                              self.XboxControls.LB:0,
                              self.XboxControls.RB:0,
                              self.XboxControls.BACK:0,
                              self.XboxControls.START:0,
                              self.XboxControls.XBOX:0,
                              self.XboxControls.LEFTTHUMB:0,
                              self.XboxControls.RIGHTTHUMB:0,
                              self.XboxControls.DPAD:(0,0)}

        self.setupPygame(joystickNo)

    # Create controller properties
    @property
    def LTHUMBX(self):
        return self.controlValues[self.XboxControls.LTHUMBX]

    @property
    def LTHUMBY(self):
        return self.controlValues[self.XboxControls.LTHUMBY]

    @property
    def RTHUMBX(self):
        return self.controlValues[self.XboxControls.RTHUMBX]

    @property
    def RTHUMBY(self):
        return self.controlValues[self.XboxControls.RTHUMBY]

    @property
    def RTRIGGER(self):
        return self.controlValues[self.XboxControls.RTRIGGER]

    @property
    def LTRIGGER(self):
        return self.controlValues[self.XboxControls.LTRIGGER]

    @property
    def A(self):
        return self.controlValues[self.XboxControls.A]

    @property
    def B(self):
        return self.controlValues[self.XboxControls.B]

    @property
    def X(self):
        return self.controlValues[self.XboxControls.X]

    @property
    def Y(self):
        return self.controlValues[self.XboxControls.Y]

    @property
    def LB(self):
        return self.controlValues[self.XboxControls.LB]

    @property
    def RB(self):
        return self.controlValues[self.XboxControls.RB]

    @property
    def BACK(self):
        return self.controlValues[self.XboxControls.BACK]

    @property
    def START(self):
        return self.controlValues[self.XboxControls.START]

    @property
    def XBOX(self):
        return self.controlValues[self.XboxControls.XBOX]

    @property
    def LEFTTHUMB(self):
        return self.controlValues[self.XboxControls.LEFTTHUMB]

    @property
    def RIGHTTHUMB(self):
        return self.controlValues[self.XboxControls.RIGHTTHUMB]

    @property
    def DPAD(self):
        return self.controlValues[self.XboxControls.DPAD]


    def setupPygame(self, joystickNo):
        pygame.init()
        screen = pygame.display.set_mode((1, 1))
        self.clock = pygame.time.Clock()
            
        while not self.connected:
            pygame.joystick.init()
            if pygame.joystick.get_count() != 0:
                joy = pygame.joystick.Joystick(joystickNo)   
                print("Found controller")
                self.connected = True    
            else:
                print("Could not detect controller.\nCheck connection\n")
                pygame.joystick.quit()
            time.sleep(2)
        
        joy.init()

    def run(self):
        self.running = True
        while self.running:
            while(self.connected):
                self.check_events()
                self.clock.tick(10)
            
            self.setupPygame(0)
    
    def check_events(self):
        got_event = False
        for event in pygame.event.get():
            if event.type == JOYAXISMOTION:
                if event.axis in self.AXISCONTROLMAP:
                    yAxis = True if (event.axis == self.PyGameAxis.LTHUMBY or event.axis == self.PyGameAxis.RTHUMBY) else False
                    self.updateControlValue(self.AXISCONTROLMAP[event.axis],
                                            self.sortOutAxisValue(event.value, yAxis))
                    got_event = True
                    
                if event.axis in self.TRIGGERCONTROLMAP:
                    self.updateControlValue(self.TRIGGERCONTROLMAP[event.axis],
                                            self.sortOutTriggerValue(event.value))
                    got_event = True

            elif event.type == JOYHATMOTION:
                self.updateControlValue(self.XboxControls.DPAD, event.value)
                got_event = True

            elif event.type == JOYBUTTONUP or event.type == JOYBUTTONDOWN:
                if event.button in self.BUTTONCONTROLMAP:
                    self.updateControlValue(self.BUTTONCONTROLMAP[event.button],
                                            self.sortOutButtonValue(event.type))
                    got_event = True
        return got_event

    def stop(self):
        self.running = False

    def updateControlValue(self, control, value):
        if self.controlValues[control] != value:
            self.controlValues[control] = value
            print(self.controlValues)
            #self.doCallBacks(control, value)
    
    def doCallBacks(self, control, value):
        if self.controllerCallBack != None: self.controllerCallBack(control, value)

        if control in self.controlCallbacks:
            self.controlCallbacks[control](value)
            
    def setupControlCallback(self, control, callbackFunction):
        self.controlCallbacks[control] = callbackFunction
                
    def sortOutAxisValue(self, value, yAxis = False):
        if yAxis and self.invertYAxis: value = value * -1
        value = value * self.scale
        if value < self.upperDeadzone and value > self.lowerDeadzone: value = 0
        return value

    def sortOutTriggerValue(self, value):
        value = max(0,(value + 1) / 2)
        value = value * self.scale
        return value

    def sortOutButtonValue(self, eventType):
        value = 1 if eventType == JOYBUTTONDOWN else 0
        return value
    

# Example of usage
if __name__ == '__main__':

    def controlCallBack(xboxControlId, value):
        print("Control Id = {}, Value = {}".format(xboxControlId, value))

    def leftThumbX(xValue):
        print("LX {}".format(xValue))
        
    def leftThumbY(yValue):
        print("LY {}".format(yValue))

    xboxCont = XboxController(None, deadzone = 30, scale = 100, invertYAxis = True)

    xboxCont.setupControlCallback(xboxCont.XboxControls.LTHUMBX, None)
    xboxCont.setupControlCallback(xboxCont.XboxControls.LTHUMBY, None)

    try:
        xboxCont.start()
        print("xbox controller running")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("User cancelled")
     
    except:
        print("Unexpected error:", sys.exc_info()[0])
        raise
        
    finally:
        xboxCont.stop()
