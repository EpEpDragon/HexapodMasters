import numpy as np
from multiprocessing import shared_memory
from roboMath import rotate_vec, rotate_vec_quat
# from simLaunch import RES_X, RES_Y, POINT_CLOUD_DIVISOR
import time
from OpenGL.GL import *
from OpenGL.GL.shaders import compileProgram,compileShader
import numpy as np
import glfw
import cv2

EXTENTS = 30                        # Extents of SDF block, in distance units
DIVISIOINS = 4                      # Cells per distance unit
SDF_EXTENTS = EXTENTS*DIVISIOINS    # Extents of SDF block, in number of cells

def to_sdf_index(global_pos):
    """Convert global position to position in SDF grid, which has its corner at 0,0,0"""
    return ((global_pos)%(EXTENTS))*DIVISIOINS


class Perception():
    def __init__(self) -> None:
        # Shared Memory buffers for communication with 3D visualisation process
        #---------------------------------------------------------------------------------
         # SDF grind, cell origin at lower corner
        sdf_buffer = np.ones((SDF_EXTENTS, SDF_EXTENTS, SDF_EXTENTS), dtype=np.float32)
        self.sdf_shm = shared_memory.SharedMemory(create=True,size=sdf_buffer.nbytes)
        self.sdf_buffer = np.ndarray(sdf_buffer.shape, dtype=np.float32, buffer=self.sdf_shm.buf)
        self.sdf_buffer[:] = sdf_buffer[:]
        # Index of current cell
        sdf_index = np.zeros(3, dtype=np.int8)
        self.sdf_index_shm = shared_memory.SharedMemory(create=True,size=sdf_index.nbytes)
        self.sdf_index = np.ndarray(sdf_index.shape, dtype=np.int8, buffer=self.sdf_index_shm.buf)
        self.sdf_index[:] = sdf_index[:]
        #---------------------------------------------------------------------------------
        
        self.cell_offset = np.zeros(3) # Position offset from cell origin


    # initialise compute shader
    def init_shader(self, n_points):
        compute_src = open('voxel_trace.glsl','r').readlines()
        shader = compileProgram(compileShader(compute_src, GL_COMPUTE_SHADER))
        glUseProgram(shader)
        self.glbuffers  = glGenBuffers(2)
        # Points buffer
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, self.glbuffers[0])
        glBufferData(GL_SHADER_STORAGE_BUFFER, n_points * 4, None, GL_DYNAMIC_READ)
        # SDF GPU buffer
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, self.glbuffers[1])
        glBufferData(GL_SHADER_STORAGE_BUFFER, self.sdf_buffer.nbytes, self.sdf_buffer, GL_DYNAMIC_READ)
 
 
    def trace_voxels(self, depth):
        # Set current sdf index
        glUniform3i(0, self.sdf_index[0], self.sdf_index[1], self.sdf_index[2])
        
        # Set point cloud buffer
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.glbuffers[0])
        # glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, points.size*4*4, points)
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, depth.nbytes, depth)

        # glDispatchCompute(SDF_EXTENTS,SDF_EXTENTS,SDF_EXTENTS)
        glDispatchCompute(1280,720,1)
        # glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT)

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.glbuffers[1])
        self.sdf_buffer = np.frombuffer(glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, self.sdf_buffer.nbytes), dtype=self.sdf_buffer.dtype).reshape(SDF_EXTENTS,SDF_EXTENTS,SDF_EXTENTS)
    
 
    def update(self, global_pos, body_quaternion, points):
        # f = lambda x: rotate_vec(x, np.array([1,0,0]), -0.54)
        f2 = lambda x: rotate_vec_quat(x, body_quaternion)
        self.update_sdf_index(global_pos)
        # points = ((f2(points) + EXTENTS/2.0)*DIVISIOINS).astype(np.float32)
        points = ((f2(points) + EXTENTS/2)*DIVISIOINS).astype(int)
        # self.trace_voxels(points)
        # self.sdf_buffer[:] = 100
        self.sdf_buffer[(points[:,0] - self.sdf_index[0])%SDF_EXTENTS, (points[:,1] - self.sdf_index[1])%SDF_EXTENTS, (points[:,2] - self.sdf_index[2])%SDF_EXTENTS] = 0.0
    
    def update_new(self, global_pos, body_quaternion, depth):
        self.trace_voxels(depth)
    

    def update_sdf_index(self, global_pos):
        """Update local pos to match the global pos and clear old data"""
        local_new = to_sdf_index(global_pos)
        self.cell_offset = local_new % 1
        local_new = local_new.astype(int)
        diff = local_new - self.sdf_index
        self.sdf_index += diff
        # Clear required cells
        #-------------------------------------------------------------
        if diff[0] != 0:
            x1 = -self.sdf_index[0]
            x2 = x1 + diff[0]
            if x1 > x2:
                x1, x2 = swap(x1,x2)
            self.sdf_buffer[x1:x2,:,:] = 100
            print(f"{x1} -- {x2}")
        if diff[1] != 0:
            y1 = -self.sdf_index[1]
            y2 = y1 + diff[1]
            if y1 > y2:
                y1, y2 = swap(y1,y2)
            self.sdf_buffer[:,y1:y2,:] = 100
            print(f"{y1} -- {y2}")
        if diff[2] != 0:
            z1 = -self.sdf_index[2]
            z2 = z1 + diff[2]
            if z1 > z2:
                z1, z2 = swap(z1,z2)
            self.sdf_buffer[:,:,z1:z2] = 100
            print(f"{z1} -- {z2}")
        
        #-------------------------------------------------------------


def swap(a,b):
    t = a
    a = b
    b = t
    return a, b