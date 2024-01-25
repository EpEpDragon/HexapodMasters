import time
import threading
# from multiprocessing import shared_memory, Process, Lock
import mujoco
import mujoco.viewer

import numpy as np
from numpy import deg2rad, rad2deg

import cv2
import glfw
import viz_cloud

from walkStateMachine import WalkCycleMachine
from perception import Perception
import controlInterface
import motion

READ_CAMERA = True

# Camera
RES_X = int(160)
RES_Y = int(90)
POINT_CLOUD_DIVISOR = 10
# Changed from control interface thread, thus list for mutable
view = [0]
snapshot = [False]

is_sim_running = True

# Linearize Depth from an OpenGL buffer
def linearize_depth(depth, znear, zfar):
    zlinear = (znear * zfar) / (zfar + depth * (znear - zfar))
    return zlinear

def init_sim_camera():
    scn = mujoco.MjvScene(model, maxgeom=100)

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
    cam.fixedcamid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'cam')

    vopt = mujoco.MjvOption()
    pert = mujoco.MjvPerturb()

    ctx = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
    mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, ctx)

    viewport = mujoco.MjrRect(0, 0, RES_X, RES_Y)
    cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
    # cv2.resizeWindow("Camera", 960, 540)

    return scn, vopt, pert, viewport, cam, ctx

def read_camera():
    mujoco.mjv_updateScene(model, data, vopt, pert, cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
    mujoco.mjr_render(viewport, scn, ctx)
    image = np.empty((RES_Y, RES_X, 3), dtype=np.uint8)
    depth = np.empty((RES_Y, RES_X, 1), dtype=np.float32)
    mujoco.mjr_readPixels(image, depth, viewport, ctx)
    image = cv2.flip(image, 0) # OpenGL renders with inverted y axis
    depth = cv2.flip(depth, 0) # OpenGL renders with inverted y axis

    # Check XML reference, choice of zfar and znear can have big effect on accuracy
    zfar  = model.vis.map.zfar * model.stat.extent
    znear = model.vis.map.znear * model.stat.extent
    depth_linear = linearize_depth(depth, znear=znear, zfar=zfar)

    # TODO Implement better filteringh method? Maybe not for performance.
    depth_linear[depth_linear < 1.9] = 0 # Zero out depth that would fall on robot
    
    # Show the simulated camera image
    if view[0] == 0:
        cv2.imshow('Camera', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    elif view[0] == 1:
        cv2.imshow('Camera', depth_linear / np.max(depth_linear))
    cv2.waitKey(1)

    return depth_linear

if __name__ == '__main__':
    # Setup model
    model = mujoco.MjModel.from_xml_path("hexapod.xml")
    data = mujoco.MjData(model)
    timestep = model.opt.timestep



    # Visual sensors0
    # -------------------------------------------------------------------------------------------------------
    if READ_CAMERA:
        gl_ctx = mujoco.GLContext(RES_X, RES_Y)
        gl_ctx.make_current()
        scn, vopt, pert, viewport, cam, ctx = init_sim_camera()

    # -------------------------------------------------------------------------------------------------------
    
    # Start movement handler
    movement_handler = motion.MovementHandler(data.ctrl, data.qpos)

    # Start sim viewer
    viewer_handle = mujoco.viewer.launch_passive(model, data)
    
    # Start perception
    perception = Perception(int(RES_Y*RES_X))
    
    # Start walkin state machine
    walk_machine = WalkCycleMachine(perception)
    walk_machine.speed = 0.5
    
    # Start contorl interface
    # controlInterface.start_interface(walk_machine,perception,view)
    control_interface_thread = threading.Thread(target=controlInterface.start_interface, args=(walk_machine,view))
    control_interface_thread.start()
    
 

    # Used for real time sim
    error = 0.0 # Timestep error integrator
    start_time = time.perf_counter()
    dt = 0.0

    k = 0
    while is_sim_running:
        # control_interface.update_input()
        
        step_start = time.perf_counter()
        walk_machine.update(timestep)
        # Move actuators-
        movement_handler.set_targets(walk_machine.foot_pos_post_yaw)
        movement_handler.update_moves()

        # Step by integrating timestep error to simulation in (approximatley) real time
        mujoco.mj_step(model, data)
        viewer_handle.sync()

        # Render camera every X timesteps(k)
        # ----------------------------------------------------------------------------------------------------
        if READ_CAMERA and k % 20 == 0:
<<<<<<< HEAD
            mujoco.mjv_updateScene(model, data, vopt, pert, cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
            mujoco.mjr_render(viewport, scn, ctx)
            image = np.empty((RES_Y, RES_X, 3), dtype=np.uint8)
            depth = np.empty((RES_Y, RES_X, 1), dtype=np.float32)
            mujoco.mjr_readPixels(image, depth, viewport, ctx)
            image = cv2.flip(image, 0) # OpenGL renders with inverted y axis
            depth = cv2.flip(depth, 0) # OpenGL renders with inverted y axis

            # Check XML reference, choice of zfar and znear can have big effect on accuracy
            zfar  = model.vis.map.zfar * model.stat.extent
            znear = model.vis.map.znear * model.stat.extent
            depth_linear = linearize_depth(depth, znear=znear, zfar=zfar)
            
            
            # For visualization
            # depth_linear[depth_linear > model.vis.map.zfar - 0.0005] = 0#model.vis.map.zfar - 0.0005 # Set depths farther than the z buffer to max z buffer

            # TODO Implement better filteringh method
            depth_linear[depth_linear < 2.5] = 0 # Zero out depth that would fall on robot
            
            # Show the simulated camera image
            if view[0] == 0:
                cv2.imshow('Camera', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            elif view[0] == 1:
                cv2.imshow('Camera', depth_linear / np.max(depth_linear))
            cv2.waitKey(1)

            # p_X = cam_x_over_z * depth_linear
            # p_Y = cam_y_over_z * depth_linear
            # p_Z = depth_linear
            # p_F = np.logical_and(np.logical_and(p_X[:][:] != 0, p_Y[:][:] != 0), p_Z[:][:] != 0)
            # points = np.dstack((-p_X, p_Y, p_Z))[p_F]

            # Update perception module
            perception.update_new(data.sensordata[0:3], data.sensordata[[4,5,6,3]], depth_linear.reshape(RES_X*RES_Y))
=======
            depth_linear = read_camera()
            perception.update(data.sensordata[0:3], data.sensordata[[4,5,6,3]], depth_linear.reshape(RES_X*RES_Y))
>>>>>>> 82761bea7d66da5f543ff4cc52ceb95ac7a0a2f2
            k = 0
        k += 1
        # ----------------------------------------------------------------------------------------------------

        step_elapse = time.perf_counter() - step_start
        time.sleep(max(min(timestep - step_elapse - error, 1), 0)) # Delay remaining timestep - error
        dt = time.perf_counter() - step_start
        error +=  (dt - timestep)*3 # Integrate error
        error = min(max(error,0), 0.002)