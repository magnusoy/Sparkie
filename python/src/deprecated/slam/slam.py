from equations import *
import numpy as np
import pyrealsense2 as rs

# Tracking camera
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)
pipe.start(cfg)

zero_vec = (0.0, 0.0, 0.0)
pos = zero_vec
vel = zero_vec
acc = zero_vec

x = []
y = []
z = []

if __name__ == "__main__":
    while True:
        try:
            frames = pipe.wait_for_frames()
            pose = frames.get_pose_frame()
            if pose:
                data = pose.get_pose_data()
                pos = data.translation
                #vel = data.velocity
                #acc = data.acceleration
            print(f'Position X: {round(pos.x, 3)}, Position Y: {round(pos.y, 3)}, Position Z: {round(pos.z, 3)}')
            x.append(pos.x)
            y.append(pos.y)
            z.append(pos.z)
        except KeyboardInterrupt:
            # Stop streaming
            pipe.stop()
            save_to_csv(x, y, z)
            
