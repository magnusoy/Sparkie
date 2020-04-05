import time

import numpy as np
import pyrealsense2 as rs

class TrackingCamera:

    def __init__(self, image_output=False):
        self.image_output = image_output

        # Declare RealSense pipeline, encapsulating the actual device and sensors
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)

        if self.image_output:
            cfg.enable_stream(rs.stream.fisheye, 1) # Left camera
            cfg.enable_stream(rs.stream.fisheye, 2) # Right camera

        # Start streaming with requested config
        self.pipe.start(cfg)
        self.running = True
        
        zero_vec = (0.0, 0.0, 0.0)
        self.pos = zero_vec
        self.vel = zero_vec
        self.acc = zero_vec
        self.img = None

    def poll(self):
        try:
            frames = self.pipe.wait_for_frames()
        except Exception as e:
            #print(e)
            return

        if self.image_output:
            left = frames.get_fisheye_frame(1)
            self.img = np.asanyarray(left.get_data())

        pose = frames.get_pose_frame()

        if pose:
            data = pose.get_pose_data()
            self.pos = data.translation
            self.vel = data.velocity
            self.acc = data.acceleration
            #print('realsense pos(%f, %f, %f)' % (self.pos.x, self.pos.y, self.pos.z))

    def update(self):
        while self.running:
            self.poll()

    def run_threaded(self):
        return self.pos, self.vel, self.acc, self.img

    def run(self):
        self.poll()
        return self.run_threaded()

    def shutdown(self):
        self.running = False
        time.sleep(0.1)
        self.pipe.stop()


# Example of usage
if __name__ == "__main__":
    camera = TrackingCamera(True)
    
    while True:
        pos, vel, acc, img = camera.run()
        print(img)
        time.sleep(0.1)
    c.shutdown()