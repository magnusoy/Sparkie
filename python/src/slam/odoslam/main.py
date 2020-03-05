#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")


#################################################################
#################################################################
##
##  This software is Copyright 2018, Mark Fassler
##  This software is licensed to you under the GPL version 3
##
#################################################################
#################################################################


import time
import collections
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
import pyrealsense2 as rs
import open3d

from rigid_transform import rigid_transform_3D

#np.seterr(all='raise')
np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)


colormap =  np.int32(plt.cm.jet(np.linspace(0,1,256)) * 255)
def depth_to_color(d):
    dmin = 1.0  # if d is here, then ii should be 255.0
    dmax = 9.0  # if d is here, then ii should be 0.0

    m = -255.0 / (dmax - dmin);
    b = 255 - (m * dmin);

    ii = m*d + b;

    i = int(round(ii))
    if i < 0:
        i = 0
    elif i > 255:
        i = 255;

    # OpenCV is in BGR order
    return int(colormap[i][2]), int(colormap[i][1]), int(colormap[i][0])



lk_params = {
    'winSize': (15, 15),
    'maxLevel': 4,
    'criteria': (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03)
}

feature_params = {
    'maxCorners': 500,
    'qualityLevel': 0.04,   # originally 0.3
    'minDistance': 7,
    'blockSize': 7
}


#track_len = 30
track_len = 130
detect_interval = 3
frame_idx = 0;
tracks = []

permanent_cloud_points = []  # np.zeros((1,3))
haveInitialWorldMap = False

notAddedYet = True
prev_gray = None


pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#config.enable_device_from_file(sys.argv[1], repeat_playback=False)

profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

rgb_stream = profile.get_stream(rs.stream.color)
rgb_stream_profile = rs.video_stream_profile(rgb_stream)
rgb_intrinsics = rgb_stream_profile.get_intrinsics()

w_minus_1 = rgb_intrinsics.width - 1
h_minus_1 = rgb_intrinsics.height - 1

align = rs.align(rs.stream.color)


class Track(collections.deque):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.realWorldPointIdx = None  # index of real-world 3-D point
        self.point_3d = None  # current 3-D point in camera coordinates


vis = open3d.visualization.Visualizer()
vis.create_window(width=800, height=600, left=1100, top=50)
vis2 = open3d.visualization.Visualizer()
vis2.create_window(width=800, height=600, left=1100, top=150)

perm_pcd = open3d.geometry.PointCloud()


pcd = open3d.geometry.PointCloud()
prev_pcd = open3d.geometry.PointCloud()

cur_points = np.zeros((1,3))

def update_point_cloud():
    global cur_points

    prev_points = cur_points
    prev_colors = np.tile( [0.5, 0, 0], (len(prev_points), 1))

    numPts = len(tracks)
    cur_points = np.empty((numPts, 3))
    cur_colors = np.tile( [0, 0.5, 0], (numPts, 1))

    for i, t in enumerate(tracks):
        cur_points[i] = t.point_3d

    prev_pcd.points = open3d.utility.Vector3dVector(prev_points)
    prev_pcd.colors = open3d.utility.Vector3dVector(prev_colors)
    pcd.points = open3d.utility.Vector3dVector(cur_points)
    pcd.colors = open3d.utility.Vector3dVector(cur_colors)



#position = np.array((0,0,0,1))
position = np.array((0,0,0), np.float64)
direction = np.array((0,0,1), np.float64)

while True:

    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    if not depth_frame or not color_frame:
        print("missing frame(s)")
        continue

    imRGB = np.asanyarray(color_frame.get_data())
    imD = np.asanyarray(depth_frame.get_data())


    # We use grayscale for calculations:
    frame_gray = cv.cvtColor(imRGB, cv.COLOR_BGR2GRAY)

    if prev_gray is None:
        prev_gray = frame_gray.copy()

    if len(tracks):
        # The p0 vector is just the last point in each of the tracks items
        p0 = np.empty((len(tracks), 1, 2), np.float32)
        for i, t in enumerate(tracks):
            p0[i] = [t[-1]]

        # Forward tracking
        p1, _st, _err = cv.calcOpticalFlowPyrLK(prev_gray, frame_gray, p0, None, **lk_params)

        # Reverse tracking
        p0r, _st, _err = cv.calcOpticalFlowPyrLK(frame_gray, prev_gray, p1, None, **lk_params)

        new_tracks = []

        for i, p in enumerate(p0):
            x = p1[i][0][0]
            y = p1[i][0][1]
            xx = max(0, min(int(round(x)), w_minus_1))
            yy = max(0, min(int(round(y)), h_minus_1))
            d = cv.norm(p - p0r[i])  # TODO: perhaps this could be a single op in numpy?...
            z_depth = depth_scale * imD[yy, xx]
            if (d < 1.5) and (z_depth < 8.0) and (z_depth > 0.1):
                pt3d = rs.rs2_deproject_pixel_to_point(rgb_intrinsics, [x,y], z_depth)
                tracks[i].append( (x, y) )
                tracks[i].point_3d = np.array([pt3d[0], -pt3d[1], -pt3d[2]])
                #color = imRGB[yy, xx] / 255
                #tracks[-1].color = color
                new_tracks.append(tracks[i])

                z_color = depth_to_color(z_depth);
                cv.circle(imRGB, (x, y), 3, z_color, -1)

        tracks = new_tracks

        # Draw the green lines showing the tracks:
        cv.polylines(imRGB, [np.int32(tr) for tr in tracks], False, (0, 255, 0))


        if haveInitialWorldMap:
            # Find the tracks that are connected to permanent (real-world) 3D points
            activePermPoints = 0
            for t in tracks:
                if t.realWorldPointIdx is not None:
                    activePermPoints += 1


            ###########
            # BEGIN:  find the transform from world coordinates to current camera coordinates
            ###########
            if activePermPoints > 5:
                world_points = np.empty((activePermPoints, 3))
                current_points = np.empty((activePermPoints, 3))

                i = 0
                for t in tracks:
                    if t.realWorldPointIdx is not None:
                        world_points[i] = permanent_cloud_points[t.realWorldPointIdx]
                        current_points[i] = t.point_3d
                        i += 1

                R, tt, inv_R = rigid_transform_3D(world_points, current_points)
                print(tt)
            else:
                print(' * Lost tracking.')
            ###########
            # END:  find the transform from world coordinates to current camera coordinates
            ###########


            ########################################
            # All tracks that are connected to permanent (real-world) 3D points
            #  will be transformed back into real-world coordinates.  New points will
            #  be added to the world as-is, existing points will be averaged in 
            #  (running average)
            
            for i, t in enumerate(tracks):
                if t.realWorldPointIdx is not None:
                    # Convert t.point_3d to world coordinates:
                    w3d = np.dot(inv_R, t.point_3d - tt)
                    # Running avg:
                    permanent_cloud_points[t.realWorldPointIdx] *= 0.99
                    permanent_cloud_points[t.realWorldPointIdx] += 0.01 * w3d
                # Find any new, stable tracks:
                if t.realWorldPointIdx is None and len(t) > 20:  # seems stable, so add it to the permanent point cloud
                    # Convert t.point_3d to world coordinates:
                    w3d = np.dot(inv_R, t.point_3d - tt)
                    # Attach to permanent_cloud_points:
                    t.realWorldPointIdx = len(permanent_cloud_points)
                    permanent_cloud_points.append(w3d)
            

    if not haveInitialWorldMap:
        if len(tracks) > 50:
            doIt = True
            for t in tracks[:35]:
                if len(t) < 30:
                    doIt = False
                    break
            if doIt:
                print("ready to make initial map")
                for t in tracks:
                    if len(t) > 20:  # seems to be stable, so add it to the world map
                        t.realWorldPointIdx = len(permanent_cloud_points)
                        permanent_cloud_points.append(t.point_3d)

                perm_pcd.points = open3d.utility.Vector3dVector(permanent_cloud_points)
                pcp_colors = np.tile([0, 0.5, 0], (len(permanent_cloud_points), 1))
                perm_pcd.colors = open3d.utility.Vector3dVector(pcp_colors)
                vis2.add_geometry(perm_pcd)

                haveInitialWorldMap = True


    # Every once-in-while, we'll try to add new points to the list of
    # points that we're tracking:
    if frame_idx % detect_interval == 0:

        # we won't bother detecting near points that we're already tracking:
        mask = np.zeros_like(frame_gray)
        mask[:] = 255
        for track in tracks:
            xy = track[-1]
            cv.circle(mask, xy, 5, 0, -1)

        pNew = cv.goodFeaturesToTrack(frame_gray, mask=mask, **feature_params)
        if pNew is not None:
            for x, y in np.float32(pNew).reshape(-1, 2):
                xx = max(0, min(int(round(x)), w_minus_1))
                yy = max(0, min(int(round(y)), h_minus_1))
                z_depth = depth_scale * imD[yy, xx]
                if z_depth < 8.0 and z_depth > 0.1:
                    tracks.append(Track(maxlen=track_len))
                    tracks[-1].append( (x, y) )
                    pt3d = rs.rs2_deproject_pixel_to_point(rgb_intrinsics, [x,y], z_depth)
                    tracks[-1].point_3d = np.array([pt3d[0], -pt3d[1], -pt3d[2]])
                    #color = imRGB[yy, xx]
                    #tracks[-1].color = color

    frame_idx += 1
    prev_gray = frame_gray

    update_point_cloud()

    if notAddedYet and len(pcd.points) > 50 and len(prev_pcd.points) > 50:
        vis.add_geometry(pcd)
        vis.add_geometry(prev_pcd)
        notAddedYet = False

    vis.update_geometry()
    vis.poll_events()
    vis.update_renderer()

    perm_pcd.points = open3d.utility.Vector3dVector(permanent_cloud_points)
    pcp_colors = np.tile([0, 0.5, 0], (len(permanent_cloud_points), 1))
    perm_pcd.colors = open3d.utility.Vector3dVector(pcp_colors)
    vis2.update_geometry()
    vis2.poll_events()
    vis2.update_renderer()


    '''
    if len(prev_points) > 10:
        R, tt = rigid_transform_3D(prev_points, cur_points)
        position = np.dot(R, position) + tt
        distance = cv.norm(position[:3])
        direction = np.dot(R, direction)
        #print(direction, position, "%.02f" % (distance))
        #print(R, tt)
        print(position[:3], direction, "%.02f" % (distance), len(permanent_cloud_points))
    '''

    cv.imshow('lk_track', imRGB)
    cv.moveWindow('lk_track', 20, 20)
    cv.waitKey(1)



