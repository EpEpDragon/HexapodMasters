#!/usr/bin/env python

import numpy as np
from roboMath import rotate
# from simLaunch import RES_X, RES_Y
from OpenGL.GL import *
from OpenGL.GL.shaders import compileProgram,compileShader

import cv2
import glfw
from math import copysign
# from scipy.spatial import Octree

import rospy

EXTENTS = 16                        # Extents of SDF block, in distance units
DIVISIOINS = 16                      # Cells per distance unit
HMAP_EXTENTS = EXTENTS*DIVISIOINS    # Extents of SDF block, in number of cells

VOXEL_TRACE_INVOCAIONS = 32         # NB This must match the x and y invocations specified in voxel_trace.glsl
CELL_DISTANCE_INVOCAIONS = 32       # NB This must match the x and y invocations specified in set_cell_distance.glsl

# COMPUTE_FILE = '../../compute'

def global_to_hmap(global_pos):
    """Convert global position to position in SDF grid, which has its corner at 0,0,0"""
    return (((global_pos)%(EXTENTS))*DIVISIOINS).astype(np.int32)

class Perception():
    def __init__(self, n_points):
        # Shared Memory buffers for communication with 3D visualisation process
        #---------------------------------------------------------------------------------
         # SDF grind, cell origin at lower corner
        hmap_buffer = np.zeros((HMAP_EXTENTS, HMAP_EXTENTS), dtype=np.float32)
        # self.sdf_shm = shared_memory.SharedMemory(create=True,size=hmap_buffer.nbytes)
        self.hmap_buffer = np.ndarray(hmap_buffer.shape, dtype=np.float32)
        self.hmap_buffer[:] = hmap_buffer[:]
        # Index of current cell
        hmap_index = np.zeros(3, dtype=np.int32)
        # self.hmap_index_shm = shared_memory.SharedMemory(create=True,size=hmap_index.nbytes)
        self.hmap_index = np.ndarray(hmap_index.shape, dtype=np.int32)
        self.hmap_index[:] = hmap_index[:]
        self.position = np.zeros(3)
        self.body_quat = np.zeros(4)
        self.temporal_i = 0
        #---------------------------------------------------------------------------------
        
        self.cell_offset = np.zeros(3) # Position offset from cell origin
        self.heightmap_program = glCreateProgram()
        self.clean_heightmap_program = glCreateProgram()

        # Make GL context
        if not glfw.init():
            return
        # Set window hint NOT visible
        glfw.window_hint(glfw.VISIBLE, False)
        # Create a windowed mode window and its OpenGL context
        window = glfw.create_window(10, 10, "hidden window", None, None)
        if not window:
            glfw.terminate()
            return
        # Make the window's context current
        glfw.make_context_current(window)

        self._init_shader(n_points)


    # initialise compute shader
    def _init_shader(self, n_points):
        # Compile cell trace program
        heightmap_src = open(rospy.get_param('pkg_root') + '/compute/heightmap.glsl','r').readlines()
        self.heightmap_program = compileProgram(compileShader(heightmap_src, GL_COMPUTE_SHADER))

        # Compile map cleaning program
        clean_heightmap_src = open(rospy.get_param('pkg_root') + '/compute/clean_heightmap.glsl','r').readlines()
        self.clean_heightmap_program = compileProgram(compileShader(clean_heightmap_src, GL_COMPUTE_SHADER))
        
        self.glbuffers  = glGenBuffers(4)

        # Bind image buffer
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, self.glbuffers[0])
        glBufferData(GL_SHADER_STORAGE_BUFFER, n_points * 4, None, GL_DYNAMIC_DRAW)
        
        # Bind SDF buffer
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, self.glbuffers[1])
        glBufferData(GL_SHADER_STORAGE_BUFFER, self.hmap_buffer.nbytes, self.hmap_buffer, GL_DYNAMIC_READ)
        

        # Bind temporal buffer
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, self.glbuffers[2])
        glBufferData(GL_SHADER_STORAGE_BUFFER, self.hmap_buffer.nbytes * 4, None, GL_DYNAMIC_READ)

        # Bind Score buffer
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, self.glbuffers[3])
        glBufferData(GL_SHADER_STORAGE_BUFFER, self.hmap_buffer.nbytes * 4, None, GL_DYNAMIC_READ)


        # Bind points buffer
        # glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, self.glbuffers[2])
        # glBufferData(GL_SHADER_STORAGE_BUFFER, n_points * 4 * 4, None, GL_DYNAMIC_COPY)
 
 
    def _generate_heightmap(self, depth, camera_quat, cam_hmap_i):
        # Set depth image data
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.glbuffers[0])
        try:        
            glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, depth.nbytes, depth)
        except:
            rospy.logerr("Data Error")


        # ################# HMap Tasks ##################
        glUseProgram(self.clean_heightmap_program)
        
        # Set Uniforms
        glUniform3i(0, self.hmap_index[0], self.hmap_index[1], self.hmap_index[2])         # Robot position
        
        glDispatchCompute(int(HMAP_EXTENTS/CELL_DISTANCE_INVOCAIONS), int(HMAP_EXTENTS/CELL_DISTANCE_INVOCAIONS), 1)


        ################### Project Depth to HMap ##################
        glUseProgram(self.heightmap_program)
        
        # Set Uniforms
        glUniform3i(0, cam_hmap_i[0], cam_hmap_i[1], cam_hmap_i[2])                     # Camera position
        glUniform4f(1, camera_quat[0], camera_quat[1], camera_quat[2], camera_quat[3])  # Camera rotation
        glUniform1i(2, self.temporal_i)
        
        glDispatchCompute(int((424)/VOXEL_TRACE_INVOCAIONS), int((240)/VOXEL_TRACE_INVOCAIONS), 1)
        
        # Sync
        # glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT)



        # Sync, read SDF
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.glbuffers[1])
        self.hmap_buffer[:] = np.frombuffer(glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, self.hmap_buffer.nbytes), dtype=self.hmap_buffer.dtype).reshape(HMAP_EXTENTS,HMAP_EXTENTS)[:]


    # HACK for visualisation of feet
    def add_walkmachine(self, walkmachine):
        self.walmachine = walkmachine
    

    def _local_to_hmap(self, local_pos):
        # Transfer point to hmap space
        return (np.round(global_to_hmap(rotate(self.body_quat, -local_pos))) - self.hmap_index + int(HMAP_EXTENTS/2)) % HMAP_EXTENTS


    def _display_heightmap(self):
        # img = np.ones((self.hmap_buffer.shape[0],self.hmap_buffer.shape[1],3), dtype=np.float32)
        img = self.hmap_buffer#[..., np.newaxis]
        # img = np.concatenate((img,img,img), axis=2)
        low = self.hmap_buffer.min()
        diff = self.hmap_buffer.max() - low
        # print("low")
        # print(low)
        # print("diff")
        # print(diff)
        img = (img - low+0.1)/(diff+0.1)

        # TODO ~70ms very slow, make better
        # for x in range(HMAP_EXTENTS):
            # for y in range(HMAP_EXTENTS):
            #     img[x,y] *= (self.hmap_buffer[(x-self.hmap_index[0])%HMAP_EXTENTS, (y-self.hmap_index[1])%HMAP_EXTENTS] + 2) / 4
                # img[x,y] = self.hmap_buffer[(x-self.hmap_index[0])%HMAP_EXTENTS, (y-self.hmap_index[1])%HMAP_EXTENTS]

        # Draw foot targets
        # for i in range(6):
        #     hmap_i = self._local_to_hmap(self.walmachine.foot_pos_post_yaw[i])
        #     # img_i = (hmap_i-self.hmap_index)%HMAP_EXTENTS   # Hold in center of centered image
        #     if (self.walmachine.is_swinging[i]):
        #         img[int(hmap_i[0]), int(hmap_i[1])] = np.array([0,1,0])
        #     else:
        #         img[int(hmap_i[0]), int(hmap_i[1])] = np.array([0,0,1])
        
        # ~8ms
        # img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)


        cv2.imshow('HMap', (img * 255).astype(np.uint8))
        cv2.waitKey(1)


    def update(self, camera_pos, body_pos, camera_quat, body_quat, depth):
        self.body_quat = body_quat
        self.hmap_index = global_to_hmap(body_pos)
        self.position = body_pos%EXTENTS
        self._generate_heightmap(depth, camera_quat, global_to_hmap(camera_pos))
        # self._display_heightmap()
        self.temporal_i = int((self.temporal_i + 1)%4)

    def get_height_at_point(self, point):
        """Returns the height at a point relative to the robot center, the height is in world space"""
        hmap_i= self._local_to_hmap(point)
        h = self.hmap_buffer[hmap_i[0], hmap_i[1]]
        return h
    
    def get_hmap(self):
        return self.hmap_buffer
