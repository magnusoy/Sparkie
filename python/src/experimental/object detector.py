import pyrealsense2 as rs
import numpy as np
import cv2
import os


def nothing(args):
    pass

def thresholdDepth(depth):
    depth[depth==0] = 255 #set all invalid depth pixels to 255
    threshold_value = cv2.getTrackbarPos('Threshold','Truncated Depth')
    ret,truncated_depth=cv2.threshold(depth_color_image,threshold_value,255,cv2.THRESH_BINARY_INV) # Zero if dist>TH
    return truncated_depth

def spatial_filtering(depth_frame, magnitude=2, alpha=0.5, delta=20, holes_fill=0):
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude, magnitude)
    spatial.set_option(rs.option.filter_smooth_alpha, alpha)
    spatial.set_option(rs.option.filter_smooth_delta, delta)
    spatial.set_option(rs.option.holes_fill, holes_fill)
    depth_frame = spatial.process(depth_frame)
    return depth_frame

def hole_filling(depth_frame):
    hole_filling = rs.hole_filling_filter()
    depth_frame = hole_filling.process(depth_frame)
    return depth_frame


cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
cv2.namedWindow('Truncated Depth', cv2.WINDOW_AUTOSIZE)
cv2.createTrackbar('Threshold','Truncated Depth',30,255,nothing)

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)


intrinsics = False
rotate_camera = False

align_to = rs.stream.color
align = rs.align(align_to)

try:

    for x in range(30):
        frames = pipeline.wait_for_frames()
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()

        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        depth_frame = spatial_filtering(depth_frame, magnitude=2, alpha=0.5, delta=50, holes_fill=0)
        # Apply hole filling filter
        depth_frame = hole_filling(depth_frame)
    
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_color_frame = rs.colorizer().colorize(depth_frame)

        depth_image = np.array(depth_frame.get_data())
        depth_color_image = np.asanyarray(depth_color_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

         # Stack rgb and depth map images horizontally for visualisation only
        images = np.hstack((color_image, depth_color_image))

        # Show images
        cv2.imshow('RealSense', images)
        truncated_depth=thresholdDepth(depth_color_image)
        cv2.imshow('Truncated Depth', truncated_depth)

        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break


finally:

    # Stop streaming
    pipeline.stop()