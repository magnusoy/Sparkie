import pyrealsense2 as rs
import numpy as np
import cv2
import os
import json
import time

# Control parameters
# =======================
json_file = "MidResHighDensityPreset.json" # MidResHighDensityPreset.json / custom / MidResHighAccuracyPreset
clipping_distance_in_meters = 1.5  # 1.5 meters
# ======================


def image_file_counter(path):
    files = 0
    for _, _, filenames in os.walk(path):
        files += len(filenames)
    return files + 1


def loadConfiguration(profile, json_file):
    dev = profile.get_device()
    advnc_mode = rs.rs400_advanced_mode(dev)
    print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")
    json_obj = json.load(open(json_file))
    json_string = str(json_obj).replace("'", '\"')
    advnc_mode.load_json(json_string)

    while not advnc_mode.is_enabled():
        print("Trying to enable advanced mode...")
        advnc_mode.toggle_advanced_mode(True)

        # At this point the device will disconnect and re-connect.
        print("Sleeping for 5 seconds...")
        time.sleep(5)

        # The 'dev' object will become invalid and we need to initialize it again
        dev = profile.get_device()
        advnc_mode = rs.rs400_advanced_mode(dev)
        print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")
        advnc_mode.load_json(json_string)


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

# define global variables
# ========================
# file names and paths
rgb_img_path = 'captured_images/rgb_image/'
depth_img_path = 'captured_images/depth_image/'
colored_depth_img_path = 'captured_images/coloured_depth_image/'
intrinsics = True
rotate_camera = True


if __name__ == "__main__":
        # ========================
    # 1. Configure all streams
    # ========================
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # ======================
    # 2. Start the streaming
    # ======================
    print("Starting up the Intel Realsense D435...")
    print("")
    profile = pipeline.start(config)

    # Load the configuration here
    loadConfiguration(profile, json_file)

    # =================================
    # 3. The depth sensor's depth scale
    # =================================
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: ", depth_scale)
    print("")

    clipping_distance = clipping_distance_in_meters / depth_scale

    # ==========================================
    # 4. Create an align object.
    #    Align the depth image to the rgb image.
    # ==========================================
    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        # ===========================================
        # 5. Skip the first 30 frames.
        # This gives the Auto-Exposure time to adjust
        # ===========================================
        for x in range(50):
            frames = pipeline.wait_for_frames()
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

        print("Intel Realsense D435 started successfully.")
        print("")

        while True:
            # ======================================
            # 6. Wait for a coherent pair of frames:
            # ======================================
            frames = pipeline.wait_for_frames()

            # =======================================
            # 7. Align the depth frame to color frame
            # =======================================
            aligned_frames = align.process(frames)

            # ================================================
            # 8. Fetch the depth and colour frames from stream
            # ================================================
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # print the camera intrinsics just once. it is always the same
            if intrinsics:
                print("Intel Realsense D435 Camera Intrinsics: ")
                print("========================================")
                print(depth_frame.profile.as_video_stream_profile().intrinsics)
                print(color_frame.profile.as_video_stream_profile().intrinsics)
                print("")
                intrinsics = False

            # =====================================
            # 9. Apply filtering to the depth image
            # =====================================
            # Apply a spatial filter without hole_filling (i.e. holes_fill=0)
            depth_frame = spatial_filtering(depth_frame, magnitude=2, alpha=0.5, delta=10, holes_fill=0)
            # Apply hole filling filter
            depth_frame = hole_filling(depth_frame)

            # ===========================
            # 10. colourise the depth map
            # ===========================
            depth_color_frame = rs.colorizer().colorize(depth_frame)

            # ==================================
            # 11. Convert images to numpy arrays
            # ==================================
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_color_image = np.asanyarray(depth_color_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # ======================================================================
            # 12. Only rotate the images if the realsense camera is placed vertical.
            # Otherwise set the variable "rotate_camera = False"
            # ======================================================================
            if rotate_camera:
                depth_image = np.rot90(depth_image, 3)
                depth_color_image = np.rot90(depth_color_image, 3)
                color_image = np.rot90(color_image, 3)

            grey_color = 0
            depth_image_3d = np.dstack((depth_image, depth_image, depth_image))

            bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

            # Stack rgb and depth map images horizontally for visualisation only
            images = np.hstack((color_image, depth_color_image))

            # Show horizontally stacked rgb and depth map images
            cv2.namedWindow('RGB and Depth Map Images')
            cv2.imshow('RGB and Depth Map Images', images)
            c = cv2.waitKey(1)

            # =============================================
            # If the 's' key is pressed, we save the images
            # =============================================
            if c == ord('s'):
                img_counter = image_file_counter(rgb_img_path)

                '''create a stream folders'''
                if not os.path.exists(rgb_img_path):
                    os.makedirs(rgb_img_path)
                if not os.path.exists(depth_img_path):
                    os.makedirs(depth_img_path)
                if not os.path.exists(colored_depth_img_path):
                    os.makedirs(colored_depth_img_path)

                filename = str(img_counter) + '.png'
                filename_csv = str(img_counter) + '.csv'

                np.savetxt(os.path.join(depth_img_path, filename_csv), np.array(depth_image), delimiter=",")

                filename_raw = str(img_counter) + '.raw'
                # save the rgb colour image
                cv2.imwrite(os.path.join(rgb_img_path, filename), color_image)
                # Save the depth image in raw binary format uint16.
                f = open(os.path.join(depth_img_path, filename_raw), mode='wb')
                depth_image.tofile(f)
                cv2.imwrite(os.path.join(colored_depth_img_path, filename), depth_color_image)

                print('images have been successfully saved')

            elif c == 27:  # esc to exit
                break

    finally:
        # Stop streaming
        pipeline.stop()