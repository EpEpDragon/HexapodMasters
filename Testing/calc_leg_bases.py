# import pyrealsense2 as rs
import numpy as np
import math
# import cv2

# pipe = rs.pipeline()
# cfg  = rs.config()

# # cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30)
# cfg.enable_stream(rs.stream.depth, 640,480, rs.format.z16, 60)

# pipe.start(cfg)

# while True:
#     frame = pipe.wait_for_frames()
#     depth_frame = frame.get_depth_frame()
#     # color_frame = frame.get_color_frame()

#     depth_image = np.asanyarray(depth_frame.get_data())
#     # color_image = np.asanyarray(color_frame.get_data())
#     depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,
#                                      alpha = 0.5), cv2.COLORMAP_JET)

#     # gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

#     # cv2.imshow('rgb', color_image)
#     cv2.imshow('depth', depth_cm)

#     if cv2.waitKey(1) == ord('q'):
#         break

# pipe.stop()

angle = math.radians(30)
for i in range(6):
    print("{%.3f, %.3f}," % (125.54*math.cos(angle), 125.54*math.sin(angle)))
    angle += 2*math.pi/6

angle = math.radians(30)
for i in range(6):
    print("{%.3f, %.3f, %.3f, %.3f}," % (math.cos(-angle/2), 0, 0, -math.sin(-angle/2)))
    angle += 2*math.pi/6

