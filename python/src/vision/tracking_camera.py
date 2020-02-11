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


import time
import numpy as np
import pyrealsense2 as rs

# Importing from local source
from .communication.publisher import Publisher


class TrackingCamera(Publisher):

    def __init__(self, image_output, ip, port, topic, interval):
        self.__init__(self, ip, port, topic)
        self.image_output = False
        self.interval = interval
        self.lastUpdate = self.millis(self)

        # Declare RealSense pipeline, encapsulating the actual device and sensors
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)

        self.enable_image_output(image_output)

        # Start streaming with requested config
        self.pipe.start(cfg)
        self.running = True
        
        zero_vec = (0.0, 0.0, 0.0)
        self.pos = zero_vec
        self.vel = zero_vec
        self.acc = zero_vec
        self.img = None
    
    @staticmethod
    def millis(self):
        """Returns the current time in milliseconds
        Returns
        -------
        current time in milliseconds
        """

        return int(round(time.time() * 1000))
    
    def enable_image_output(self, enable):
        if enable:
            cfg.enable_stream(rs.stream.fisheye, 1) # Left camera
            cfg.enable_stream(rs.stream.fisheye, 2) # Right camera
            self.image_output = True
            self.pipe.start(cfg)

    def poll(self):
        try:
            frames = self.pipe.wait_for_frames()
        except Exception as e:
            print(e)
            return

        if self.image_output:
            left = frames.get_fisheye_frame(1)
            self.img = np.asanyarray(left.get_data())

        pose = frames.get_pose_frame()

        if pose:
            self.data = pose.get_pose_data()
            self.pos = self.data.translation
            self.vel = self.data.velocity
            self.acc = self.data.acceleration
            #print('realsense pos(%f, %f, %f)' % (self.pos.x, self.pos.y, self.pos.z))

    def update(self):
        while self.running:
            self.poll()

    def run_threaded(self):
        return self.pos, self.vel, self.acc, self.img

    def run(self):

        self.initialize(self)

        while self.running:
            now = self.millis(self)
            timeDifference = now - self.lastUpdate
            if timeDifference >= self.interval:
                self.poll()
                self.publish_img()
                self.publish_data()
                self.lastUpdate = now
        
    def publish_img(self):
        Publisher.topic = 'img'
        Publisher.send(self.img)
    
    def publish_data(self):
        Publisher.topic = 'pose'
        Publisher.send(self.data)

    def shutdown(self):
        self.running = False
        time.sleep(0.1)
        self.pipe.stop()


# Example of usage
if __name__ == "__main__":
    camera = TrackingCamera()
    
    while True:
        pos, vel, acc = camera.run()
        print(pos)
        time.sleep(0.1)