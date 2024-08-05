# import pyrealsense2 as rs
import numpy as np
import math
from sympy import *
from IPython.display import display, Math

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


print("----origins----")
print("{")
angle = math.radians(-30)
for i in range(6):
    print("{%.3f, %.3f, 0}," % (125.54*math.cos(angle), 125.54*math.sin(angle)))
    angle -= 2*math.pi/6
print("}")

print("----quats----")
print("{")
angle = math.radians(-30)
for i in range(6):
    print("{%.3f, %.3f, %.3f, %.3f}," % (math.cos(-angle/2), 0, 0, math.sin(-angle/2)))
    angle -= 2*math.pi/6
print("}")

print("----base pos----")
angle = math.radians(-30)
print("{")
for i in range(6):
    print("{%.3f, %.3f, -140}," % (240*math.cos(angle), 240*math.sin(angle)))
    angle -= 2*math.pi/6
print("}")

# Servo Rates
init_printing()
t = symbols('t')
# theta1 = Function('theta1')
# theta2 = Function('theta2')
# theta3 = Function('theta3')
Px = Function('Px')
Py = Function('Py')
Pz = Function('Pz')

theta1 = atan(Px(t)/Py(t))
dt_theta1 = diff(theta1, t)


print(dt_theta1)