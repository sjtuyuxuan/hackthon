import pyrealsense2 as rs
import math
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
from interbotix_xs_modules.arm import InterbotixManipulatorXS

bot = InterbotixManipulatorXS("px100", "arm", "gripper")
bot.arm.go_to_home_pose()
bot.arm.set_ee_cartesian_trajectory(x = -0.15)
bot.gripper.set_pressure(1.0)
bot.gripper.open()

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# get intr
p = profile.get_stream(rs.stream.color)
intr = p.as_video_stream_profile().get_intrinsics()
# Streaming loop
Grep_flag = False
try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        # define range of blue color in HSV
        lower_purple = np.array([115,110,50])
        upper_purple = np.array([140,255,255])
        # Threshold the HSV image to get only purple colors
        mask = cv2.inRange(hsv, lower_purple, upper_purple)
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(color_image, color_image, mask= mask)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(color_image, contours, -1, (0,0,255), 1)
        center_size = 0
        center = np.array([0, 0])
        point3d = [0, 0, 0]
        for contour in contours:
            if len(contour) < 30:
                continue
            for point in contour:
                center += point[0]
            center_size += len(contour)
        if center_size > 0:
            center = center / center_size
            cv2.circle(color_image, (int(center[0]), int(center[1])), 10, (255,255,255), -1)
            depth = depth_image[int(center[1])][int(center[0])] / 1000
            if 1 > depth > 0.01:
                point3d = rs.rs2_deproject_pixel_to_point(intr, [center[0], center[1]], depth)
        

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        # Render images:
        #   depth align to color on left
        #   depth on right
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
	# images = np.hstack((bg_removed, depth_colormap))
        images = np.hstack((color_image, depth_colormap))

        cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        cv2.imshow('Align Example', images)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
        elif key & 0xFF == ord('g') or Grep_flag:
            print(point3d)
            if 0.2 < point3d[2] < 1.0:
                theta = math.atan2(0.405 - point3d[2], 0.15 - point3d[0])
                bot.arm.set_single_joint_position("waist", theta)
                x_x = math.sqrt((0.15 - point3d[0]) * (0.15 - point3d[0]) + (0.405 - point3d[2]) * (0.405 - point3d[2]))
                bot.arm.set_ee_cartesian_trajectory(x = x_x - 0.1, z = -0.04 - point3d[1])
                bot.gripper.close(3)
                bot.arm.go_to_home_pose()
                bot.arm.set_single_joint_position("waist", -np.pi/2.0)
                bot.gripper.open()
                bot.arm.go_to_home_pose()
                bot.arm.set_ee_cartesian_trajectory(x = -0.15)
                Grep_flag = False
            else:
                Grep_flag = True
        elif key & 0xFF == ord('h'):
            bot.arm.go_to_home_pose()
            bot.arm.set_ee_cartesian_trajectory(x = -0.15)
            
finally:
    pipeline.stop()

