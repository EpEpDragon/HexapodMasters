import time
import threading
# from multiprocessing import shared_memory, Process, Lock
import mujoco
import mujoco.viewer

import numpy as np
from numpy import deg2rad, rad2deg

import cv2
import os

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

servo_torque = []

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
    depth_linear[depth_linear < 3] = 0 # Zero out depth that would fall on robot
    
    # Show the simulated camera image ~ 5ms should change
    if view[0] == 0:
        cv2.imshow('Camera', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    elif view[0] == 1:
        cv2.imshow('Camera', depth_linear / np.max(depth_linear))
    cv2.waitKey(1)

    return depth_linear

def read_sensors():
    return data.sensordata[0:3], data.sensordata[3:6], data.sensordata[[7,8,9,6]], data.sensordata[[11,12,13,10]]

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
    control_interface_thread = threading.Thread(target=controlInterface.start_interface, args=(walk_machine, view, perception.map_view))
    control_interface_thread.start()
    perception.add_refs(walk_machine)
    
 
    # Used for real time sim
    error = 0.0 # Timestep error integrator
    start_time = time.perf_counter()
    dt = 0.0
    k = 0

    while is_sim_running:
        # control_interface.update_input()
        step_start = time.perf_counter()
        cam_pos, body_pos, cam_quat, body_quat = read_sensors()
        # print(data.sensordata[14:32])
        # servo_torque.append([data.sensordata[i] for i in [15,18,21,24,27,30]])
        # servo_torque.append(data.sensordata[15])

        walk_machine.update(timestep, body_quat)
        # Move actuators-
        if walk_machine.is_move_valid:
            movement_handler.set_targets(walk_machine.foot_pos_post_yaw)
            movement_handler.update_moves()

        # Step by integrating timestep error to simulation in (approximatley) real time
        mujoco.mj_step(model, data)
        viewer_handle.sync()
        # os.system('clear') 
        # print(f"True body height: {data.sensordata[5]}")

        # Render camera every X timesteps(k)
        # ----------------------------------------------------------------------------------------------------
        if READ_CAMERA and k %30 == 0:
            # torque_file = open("torque.csv","a")
            # torque_file.write(str(data.sensordata[15])+",")
            # torque_file.close

            depth_linear = read_camera()
            perception.update(cam_pos, body_pos, cam_quat, body_quat, depth_linear.reshape(RES_X*RES_Y))
            k = 0
        k += 1
        # ----------------------------------------------------------------------------------------------------

        step_elapse = time.perf_counter() - step_start
        time.sleep(max(min(timestep - step_elapse - error, 1), 0)) # Delay remaining timestep - error
        dt = time.perf_counter() - step_start
        error +=  (dt - timestep)*3 # Integrate error
        error = min(max(error,0), 0.02)
        # print(dt)
    print("end")