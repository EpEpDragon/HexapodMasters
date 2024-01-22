import os
import time
import threading
from multiprocessing import shared_memory, Process, Lock
import mujoco
from mujoco import viewer
from math import sin, cos, tan, pi

import numpy as np
from numpy import deg2rad, rad2deg
from numpy import array as a

import cv2
import viz_cloud
import open3d as o3d

import depth_image_test as dit
# import keyboard

from walkStateMachine import WalkCycleMachine
from perception import Perception, SDF_EXTENTS
import controlInterface
import motion
from roboMath import rotate_vec

READ_CAMERA = True

# Camera
RES_X = 160
RES_Y = 90
POINT_CLOUD_DIVISOR = 10
# Changed from control interface thread, thus list for mutable
view = [0]
snapshot = [False]

is_sim_running = True


# def input(event):
#     # print(get_active_window_title())
#     # window = windowFuncs.get_active_window_title()
#     # if window == "MuJoCo : MuJoCo Model" or  window == "Control Interface":
#     if event.scan_code == 1:
#         os._exit(os.EX_OK)

# Linearize Depth from an OpenGL buffer
def linearize_depth(depth, znear, zfar):
    zlinear = (znear * zfar) / (zfar + depth * (znear - zfar))
    return zlinear

if __name__ == '__main__':
    # Setup model
    model = mujoco.MjModel.from_xml_path("hexapod.xml")
    data = mujoco.MjData(model)
    timestep = model.opt.timestep

    # Start walkin state machine
    walk_machine = WalkCycleMachine()
    walk_machine.speed = 0.5

    # Visual sensors0
    # -------------------------------------------------------------------------------------------------------
    if READ_CAMERA:
        # Perception memory
        perception = Perception()

        # a = np.random.rand(SDF_EXTENTS,SDF_EXTENTS,SDF_EXTENTS,3)
        # a = np.zeros((SDF_EXTENTS*SDF_EXTENTS*SDF_EXTENTS,3))
        # lock = Lock()
        # sdf_shm = shared_memory.SharedMemory(create=True,size=perception.sdf_buffer.nbytes)
        # sdf_buffer = np.ndarray(perception.sdf_buffer.shape, dtype=np.float64, buffer=sdf_shm.buf)
        # sdf_buffer[:] = a[:]  # Copy the original data into shared memory
        
        # a = np.zeros(3)
        # sdf_index_shm = shared_memory.SharedMemory(create=True,size=a.nbytes)
        # sdf_index = np.ndarray(a.shape, dtype=np.float32, buffer=sdf_index_shm.buf)
        # sdf_index[:] = a[:]
        
        # Visualisation process
        # p1 = Process(target=viz_cloud.start, args=(perception.sdf_shm.name, perception.sdf_index_shm.name,), daemon=True)
        # p1.start()
    
        # Camera Setup
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
        cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Camera", 960, 540)
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

    # Start contorl interface
    # control_interface = ControInterface(walk_machine)
    control_interface_thread = threading.Thread(target=controlInterface.start_interface, args=(walk_machine,perception,view,snapshot))
    control_interface_thread.start()
    # walk_machine.set_walk_direction(np.array([0,-1,0]))
    
    # Start movement handler
    movement_handler = motion.MovementHandler(data.ctrl, data.qpos)
    

    # Start simulation
    viewer.launch_passive(model, data)
    time.sleep(1)
    perception.init_shader(int(RES_Y*RES_X))

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

        # Render camera every X timesteps(k)
        # ----------------------------------------------------------------------------------------------------
        if READ_CAMERA and k % 20 == 0:
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
            
            # Update perception module
            perception.update_new(data.sensordata[0:3], data.sensordata[[4,5,6,3]], depth_linear.reshape(RES_X*RES_Y))
            k = 0
        k += 1
        # ----------------------------------------------------------------------------------------------------
        step_elapse = time.perf_counter() - step_start
        time.sleep(max(min(timestep - step_elapse - error, 1), 0)) # Delay remaining timestep - error
        dt = time.perf_counter() - step_start
        error +=  (dt - timestep)*3 # Integrate error
        error = min(max(error,0), 0.002)