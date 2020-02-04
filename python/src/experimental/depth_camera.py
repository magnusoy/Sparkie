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

import cv2
import time
import numpy as np
import pyrealsense2 as rs

# Importing from local source
from ..communication.publisher import Publisher


class DepthCamera(Publisher):

    def __init__(self, color=False, ip, port, topic):
        self.__init__(self, ip, port, topic)
         self.pipe = rs.pipeline()
         cfg = rs.config()
         cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
         cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

         self.pipe.start(cfg)

         self.running = True
         self.color = color

         # Processing blocks
         self.pc = rs.pointcloud()
         self.decimate = rs.decimation_filter()
         #self.decimate.set_option(rs.option.filter_magnitude, 2 ** self.decimate)
         self.colorizer = rs.colorizer()
         
        
    def poll(self):
        try:
            frames = self.pipe.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            color_img = np.asanyarray(color_frame.get_data())
            depth_colormap = np.asanyarray(self.colorizer.colorize(depth_frame).get_data())
            
            if self.color:
                mapped_frame, color_source = color_frame, color_image
            else:
                mapped_frame, color_source = depth_frame, depth_colormap
            
            points = self.pc.calculate(depth_frame)
            self.pc.map_to(mapped_frame)
            
            v, t = points.get_vertices(), points.get_texture_coordinates()
            verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
            texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv
            print(verts)

            cv2.imshow('img',texcoords)
            cv2.waitKey(1)
            
            #depth_frame = self.decimate.process(depth_frame)
        except Exception as e:
            print(e)
            return
    
    def run(self):

        self.initialize(self)

        while self.running:
            pass
    

if __name__ == "__main__":
    dc = DepthCamera()
    while True:
        dc.poll()
