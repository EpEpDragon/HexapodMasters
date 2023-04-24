import os
import time
import threading

import mujoco
from mujoco import viewer
from math import sin, cos, tan, pi

import numpy as np
from numpy import deg2rad, rad2deg
from numpy import array as a

import cv2
import pyray as pr

# import keyboard
import windowFuncs

from walkStateMachine import WalkCycleMachine
from controlInterface import ControInterface
import motion
from roboMath import rotate_vec

# Camera
RES_X = 1280
RES_Y = 720

is_sim_running = True

def input(event):
    # print(get_active_window_title())
    # window = windowFuncs.get_active_window_title()
    # if window == "MuJoCo : MuJoCo Model" or  window == "Control Interface":
    if event.scan_code == 1:
        os._exit(os.EX_OK)

def start_interface(walk_machine):
    control_interface = ControInterface(walk_machine)
    control_interface.update()

if __name__ == '__main__':
    # Setup model
    model = mujoco.MjModel.from_xml_path("hexapod.xml")
    data = mujoco.MjData(model)
    timestep = model.opt.timestep

    # Start walkin state machine
    walk_machine = WalkCycleMachine()
    walk_machine.speed = 0.5

    # Start contorl interface
    # control_interface = ControInterface(walk_machine)
    control_interface_thread = threading.Thread(target=start_interface, args=(walk_machine,))
    control_interface_thread.start()
    # keyboard.on_press(input)

    # Start movement handler
    movement_handler = motion.MovementHandler(data.ctrl, data.qpos)

    # Camera Stuff
    # -------------------------------------------------------------------------------------------------------
    gl_ctx = mujoco.GLContext(RES_X, RES_Y)
    gl_ctx.make_current()

    scn = mujoco.MjvScene(model, maxgeom=100)

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
    cam.fixedcamid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'cam')

    vopt = mujoco.MjvOption()
    pert = mujoco.MjvPerturb()

    ctx = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
    mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, ctx)

    viewport = mujoco.MjrRect(0, 0, RES_X, RES_Y)

    yfov = model.cam_fovy[cam.fixedcamid]
    fy = (RES_Y/2) / np.tan(yfov * np.pi / 180 / 2)
    fx = fy
    cx = (RES_X-1) / 2.0
    cy = (RES_Y-1) / 2.0

    cam_K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

    # Get the 3D direction vector for each pixel in the simulated sensor
    # in the format (x, y, 1)
    cam_x_over_z, cam_y_over_z = cv2.initInverseRectificationMap(
            cam_K, # Intrinsics
            None, # Distortion (0 for GPU rendered images)
            np.eye(3), # Rectification
            np.eye(3), # Unity rectification intrinsics (we want direction vector)
            (RES_X, RES_Y), # Test all pixels in physical sensor
            m1type=cv2.CV_32FC1)

    sample_list = []
    # -------------------------------------------------------------------------------------------------------

    # Start simulation
    viewer.launch_passive(model, data)
    time.sleep(1)
    windowFuncs.move_size_window("MuJoCo : MuJoCo Model",0,0,0,0.8,0.9)
    windowFuncs.move_size_window("Control Interface",0,0.8,0,0.2,0.9)

    # Used for real time sim
    error = 0.0 # Timestep error integrator
    start_time = time.perf_counter()
    dt = 0.0

    k = 0
    while is_sim_running:
        # control_interface.update_input()
        
        # Run every X timesteps(k)
        if k % 20 == 0:
            # control_interface.update()
            # Camera Stuff
            # -----------------------------------------------------------------------------------------------
            mujoco.mjv_updateScene(model, data, vopt, pert, cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
            mujoco.mjr_render(viewport, scn, ctx)
            image = np.empty((RES_Y, RES_X, 3), dtype=np.uint8)
            depth = np.empty((RES_Y, RES_X, 1),    dtype=np.float32)
            mujoco.mjr_readPixels(image, depth, viewport, ctx)
            image = cv2.flip(image, 0) # OpenGL renders with inverted y axis
            depth = cv2.flip(depth, 0) # OpenGL renders with inverted y axis

            # Show the simulated camera image
            cv2.imshow('image', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)

            # -----------------------------------------------------------------------------------------------
            
            k = 0
        k += 1

        step_start = time.perf_counter()
        walk_machine.update(timestep)
        # Move actuators
        movement_handler.set_targets(walk_machine.foot_pos)
        movement_handler.update_moves(timestep)

        # Step by integrating timestep error to simulation in (approximatley) real time
        mujoco.mj_step(model, data)
        step_elapse = time.perf_counter() - step_start
        time.sleep(max(min(timestep - step_elapse - error, 1), 0)) # Delay remaining timestep - error
        dt = time.perf_counter() - step_start
        error +=  dt - timestep # Integrate error