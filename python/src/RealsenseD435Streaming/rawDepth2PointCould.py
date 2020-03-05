import pyrealsense2 as rs
import cv2
import numpy as np

ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''


def write_ply(fn, verts, colors):
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])
    with open(fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')


def depth2D_to_point3D(depth_intrinsic, raw_depth, rgb_image, depth_scale):
    rows, cols = np.array(raw_depth).shape
    points3D = []
    rgb_values = []
    for r in range(rows):
        for c in range(cols):
            depth_pixel = [r, c]
            rgb_value = rgb_image[r, c] # use this for the ply file
            raw_depth_value = raw_depth[r, c]
            #print(raw_depth_value)
            if raw_depth_value > 0:

                print("Hello")
                pt3D = rs.rs2_deproject_pixel_to_point(depth_intrinsic, depth_pixel, raw_depth_value * depth_scale)
                points3D.append(pt3D)
                rgb_values.append(rgb_value)
    return np.array(points3D), np.array(rgb_values)


def read_images(rgb="captured_images/rgb_image/1.png", depth="captured_images/depth_image/1.raw"):
    # Read the raw depth and colour images
    rgb_image = cv2.imread(rgb)
    rows, cols, channel = rgb_image.shape
    f = open(depth, mode='rb')
    raw_depth = np.fromfile(f, dtype=np.uint16)
    raw_depth = raw_depth.reshape(rows, cols)
    return rgb_image, raw_depth


def segment_raw_depth_image(segmented_rgb, raw_depth):

    segmented_gray = cv2.cvtColor(segmented_rgb, cv2.COLOR_BGR2GRAY)
    rows, cols = np.array(segmented_gray).shape

    for r in range(rows):
        for c in range(cols):
            segmented_gray_value = segmented_gray[r, c]  # use this for the ply file
            # print(raw_depth_value)
            if segmented_gray_value == 0:
                raw_depth[r, c] = 0

    return raw_depth


if __name__ == "__main__":

    rgb_image, raw_depth = read_images(rgb="captured_images/rgb_image/1.png", depth="captured_images/depth_image/1.raw")

    # Configure depth and color streams. Please ensure the resolutions are the same as used in recording the images
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)

    # Start streaming
    profile = pipeline.start(config)

    # Get the depth sensor's depth scale. This will be used to multiple the depth values to get point could in meters
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: ", depth_scale)
    print("")

    # Create an align object, which will align the depth image to the rgb image
    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        # Skip 30 first frames to give the Auto-Exposure time to adjust
        for x in range(30):
            frames = pipeline.wait_for_frames()
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

        # Now auto-exposure is ready, so get the frame
        frames = pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # get the depth and colour frames
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # get the depth and colour instrinsics
        print("Intel Realsense D435 Camera Intrinsics: ")
        print("========================================")
        depth_intrinsic = depth_frame.profile.as_video_stream_profile().intrinsics
        print(depth_intrinsic)
        color_intrinsic = color_frame.profile.as_video_stream_profile().intrinsics
        print(color_intrinsic)
        print("")

        #depth_intrinsic = {'width': 640, 'height': 480, 'ppx': 320.156, 'ppy': 238.092, 'fx': 385.367, 'fy': 385.367, 'model': 'Brown Conrady', 'coeffs': [0, 0, 0, 0, 0]}
        #color_intrinsic = {'width': 640, 'height': 480, 'ppx': 320.156, 'ppy': 238.092, 'fx': 385.367, 'fy': 385.367, 'model': 'Brown Conrady', 'coeffs': [0, 0, 0, 0, 0]}

        # segment the rgb image using deep learning prediction and mask the segmented image from the depth image
        segmented_rgb = cv2.imread("captured_images/rgb_image/1.png")
        raw_depth_segmented = segment_raw_depth_image(segmented_rgb, raw_depth)

        pts3D, rgb_values = depth2D_to_point3D(depth_intrinsic, raw_depth_segmented, rgb_image, depth_scale)  # depth and rgb should have the same dimensions
        #print(np.array(pts3D).shape)
        fn = "captured_images/1.ply"
        write_ply(fn, pts3D, rgb_values)

    finally:
        # Stop streaming
        pipeline.stop()