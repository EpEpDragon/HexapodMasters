import numpy as np
from multiprocessing import shared_memory
from roboMath import rotate_vec, rotate_vec_quat
# from simLaunch import RES_X, RES_Y
import time
from OpenGL.GL import *
from OpenGL.GL.shaders import compileProgram,compileShader
import numpy as np
import glfw
import cv2
from math import copysign
# from scipy.spatial import Octree

EXTENTS = 16                        # Extents of SDF block, in distance units
DIVISIOINS = 8                      # Cells per distance unit
SDF_EXTENTS = EXTENTS*DIVISIOINS    # Extents of SDF block, in number of cells

VOXEL_TRACE_INVOCAIONS = 32         # NB This must match the x and y invocations specified in voxel_trace.glsl
CELL_DISTANCE_INVOCAIONS = 32       # NB This must match the x and y invocations specified in set_cell_distance.glsl

def to_sdf_index(global_pos):
    """Convert global position to position in SDF grid, which has its corner at 0,0,0"""
    return (((global_pos)%(EXTENTS))*DIVISIOINS).astype(np.int32)


class Perception():
    def __init__(self, n_points) -> None:
        # Shared Memory buffers for communication with 3D visualisation process
        #---------------------------------------------------------------------------------
         # SDF grind, cell origin at lower corner
        sdf_buffer = np.ones((SDF_EXTENTS, SDF_EXTENTS), dtype=np.float32) * SDF_EXTENTS/2.0
        self.sdf_shm = shared_memory.SharedMemory(create=True,size=sdf_buffer.nbytes)
        self.sdf_buffer = np.ndarray(sdf_buffer.shape, dtype=np.float32, buffer=self.sdf_shm.buf)
        self.sdf_buffer[:] = sdf_buffer[:]
        # Index of current cell
        sdf_index = np.zeros(3, dtype=np.int32)
        self.sdf_index_shm = shared_memory.SharedMemory(create=True,size=sdf_index.nbytes)
        self.sdf_index = np.ndarray(sdf_index.shape, dtype=np.int32, buffer=self.sdf_index_shm.buf)
        self.sdf_index[:] = sdf_index[:]
        #---------------------------------------------------------------------------------
        
        self.cell_offset = np.zeros(3) # Position offset from cell origin
        self.heightmap_program = glCreateProgram()
        self.clean_heightmap_program = glCreateProgram()

        # visualize window
        cv2.namedWindow('SDF Slice', cv2.WINDOW_NORMAL)
        self.init_shader(n_points)


    # initialise compute shader
    def init_shader(self, n_points):
        # Compile cell trace program
        heightmap_src = open('compute/heightmap.glsl','r').readlines()
        self.heightmap_program = compileProgram(compileShader(heightmap_src, GL_COMPUTE_SHADER))

        # Compile map cleaning program
        clean_heightmap_src = open('compute/clean_heightmap.glsl','r').readlines()
        self.clean_heightmap_program = compileProgram(compileShader(clean_heightmap_src, GL_COMPUTE_SHADER))
        
        self.glbuffers  = glGenBuffers(2)

        # Bind image buffer
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, self.glbuffers[0])
        glBufferData(GL_SHADER_STORAGE_BUFFER, n_points * 4, None, GL_DYNAMIC_DRAW)
        
        # Bind SDF buffer
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, self.glbuffers[1])
        glBufferData(GL_SHADER_STORAGE_BUFFER, self.sdf_buffer.nbytes, self.sdf_buffer, GL_DYNAMIC_READ)
        
        # Bind points buffer
        # glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, self.glbuffers[2])
        # glBufferData(GL_SHADER_STORAGE_BUFFER, n_points * 4 * 4, None, GL_DYNAMIC_COPY)
 
 
    def generate_heightmap(self, depth, camera_quat):
        # Set depth image data
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.glbuffers[0])        
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, depth.nbytes, depth)

        ################### Trace stage ##################
        glUseProgram(self.heightmap_program)
        
        # Set Uniforms
        glUniform3i(0, self.sdf_index[0], self.sdf_index[1], self.sdf_index[2])         # Robot position
        glUniform4f(1, camera_quat[0], camera_quat[1], camera_quat[2], camera_quat[3])  # Robot rotation
        
        glDispatchCompute(int(160/VOXEL_TRACE_INVOCAIONS), int(90/VOXEL_TRACE_INVOCAIONS), 1)
        
        # Sync
        # glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT)

        # ################# Distance stage ##################
        glUseProgram(self.clean_heightmap_program)
        
        # # # Set Uniforms
        glUniform3i(0, self.sdf_index[0], self.sdf_index[1], self.sdf_index[2])         # Robot position
        
        glDispatchCompute(int(SDF_EXTENTS/CELL_DISTANCE_INVOCAIONS), int(SDF_EXTENTS/CELL_DISTANCE_INVOCAIONS), 1)

        # Sync, read SDF
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.glbuffers[1])
        self.sdf_buffer[:] = np.frombuffer(glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, self.sdf_buffer.nbytes), dtype=self.sdf_buffer.dtype).reshape(SDF_EXTENTS,SDF_EXTENTS)[:]
        # glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.glbuffers[2])
        # points = np.frombuffer(glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, 160*90*4*4), dtype=np.float32).reshape((160*90,4))
        # points_tree = KDTree(points)
        # test = np.indices(self.sdf_buffer.shape)
        # points_tree.query()
    
 
    def update(self, global_pos, body_quaternion, points):
        # f = lambda x: rotate_vec(x, np.array([1,0,0]), -0.54)
        f2 = lambda x: rotate_vec_quat(x, body_quaternion)
        self.update_sdf_index(global_pos)
        # points = ((f2(points) + EXTENTS/2.0)*DIVISIOINS).astype(np.float32)
        points = ((f2(points) + EXTENTS/2)*DIVISIOINS).astype(int)
        # self.generate_heightmap(points)
        # self.sdf_buffer[:] = 100
        self.sdf_buffer[(points[:,0] - self.sdf_index[0])%SDF_EXTENTS, (points[:,1] - self.sdf_index[1])%SDF_EXTENTS, (points[:,2] - self.sdf_index[2])%SDF_EXTENTS] = 0.0
    
    def display_heightmap(self):

        img = np.zeros(self.sdf_buffer.shape)
        low = self.sdf_buffer.max()
        diff = self.sdf_buffer.min() - low
        for x in range(SDF_EXTENTS):
            for y in range(SDF_EXTENTS):
                img[x,y] = (self.sdf_buffer[(x-self.sdf_index[0])%SDF_EXTENTS, (y-self.sdf_index[1])%SDF_EXTENTS] - low) / diff
                # img[x,y] = self.sdf_buffer[(x-self.sdf_index[0])%SDF_EXTENTS, (y-self.sdf_index[1])%SDF_EXTENTS]
        img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        cv2.imshow('SDF Slice', img)
        cv2.waitKey(1)

    def update_new(self, global_pos, camera_quat, depth):
        self.sdf_index = to_sdf_index(global_pos)
        self.generate_heightmap(depth, camera_quat)
        self.display_heightmap()

def swap(a,b):
    t = a
    a = b
    b = t
    return a, b