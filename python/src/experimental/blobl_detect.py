## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2

def nothing(args):
    pass

def detectBlobs(mask):
        # Set up the SimpleBlobdetector with default parameters.
    params = cv2.SimpleBlobDetector_Params()
         
    # Change thresholds
    params.minThreshold = 1;
    params.maxThreshold = 255;
    
    # Filter by Area.
    params.filterByArea = True
    params.maxArea = 4000
    params.minArea = 300
    
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
    
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.5
    
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.1

         
    detector = cv2.SimpleBlobDetector_create(params)
 
    # Detect blobs.
    reversemask= mask
    keypoints = detector.detect(reversemask)
    im_with_keypoints = cv2.drawKeypoints(mask, keypoints, np.array([]),
            (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    return im_with_keypoints

def thresholdDepth(depth):
    depth[depth==0] = 255 #set all invalid depth pixels to 255
    threshold_value = cv2.getTrackbarPos('Threshold','Truncated Depth')
    ret,truncated_depth=cv2.threshold(scaled_depth,threshold_value,255,cv2.THRESH_BINARY_INV) # Zero if dist>TH
    return truncated_depth

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

try:
    while True:

        
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        scaled_depth=cv2.convertScaleAbs(depth_image, alpha=0.08)
        depth_colormap = cv2.applyColorMap(scaled_depth, cv2.COLORMAP_JET)

        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.imshow('RealSense', images)
        truncated_depth=thresholdDepth(scaled_depth)
        truncated_depth=detectBlobs(truncated_depth)
        cv2.imshow('Truncated Depth', truncated_depth)

        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

finally:

    # Stop streaming
    pipeline.stop()