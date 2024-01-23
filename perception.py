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
# from scipy.spatial import Octree

EXTENTS = 16                        # Extents of SDF block, in distance units
DIVISIOINS = 10                      # Cells per distance unit
SDF_EXTENTS = EXTENTS*DIVISIOINS    # Extents of SDF block, in number of cells

VOXEL_TRACE_INVOCAIONS = 32         # NB This must match the x and y invocations specified in voxel_trace.glsl
CELL_DISTANCE_INVOCAIONS = 10       # NB This must match the x and y invocations specified in set_cell_distance.glsl

def to_sdf_index(global_pos):
    """Convert global position to position in SDF grid, which has its corner at 0,0,0"""
    return (((global_pos)%(EXTENTS))*DIVISIOINS).astype(np.int32)


class Perception():
    def __init__(self) -> None:
        # Shared Memory buffers for communication with 3D visualisation process
        #---------------------------------------------------------------------------------
         # SDF grind, cell origin at lower corner
        sdf_buffer = np.ones((SDF_EXTENTS, SDF_EXTENTS), dtype=np.float32) * SDF_EXTENTS/2
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
        self.distance_program = glCreateProgram()

        # visualize REMOVE THIS IN PRODUCTION
        self.vslice = 50

    # initialise compute shader
    def init_shader(self, n_points):
        # Compile cell trace program
        heightmap_src = open('compute/heightmap.glsl','r').readlines()
        self.heightmap_program = compileProgram(compileShader(heightmap_src, GL_COMPUTE_SHADER))

        # Compile cell distance program
        # distance_src = open('compute/set_cell_distance.glsl','r').readlines()
        # self.distance_program = compileProgram(compileShader(distance_src, GL_COMPUTE_SHADER))
        
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
 
 
    def generate_heightmap(self, depth, camera_quat, erase_range):
        # Set depth image data
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.glbuffers[0])        
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, depth.nbytes, depth)

        ################### Trace stage ##################
        glUseProgram(self.heightmap_program)
        
        # Set Uniforms
        glUniform3i(0, self.sdf_index[0], self.sdf_index[1], self.sdf_index[2])         # Robot position
        # print(self.sdf_index)
        glUniform4f(1, camera_quat[0], camera_quat[1], camera_quat[2], camera_quat[3])  # Robot rotation
        
        # Erase range
        glUniform1i(2,erase_range[0])
        glUniform1i(3,erase_range[1])
        glUniform1i(4,erase_range[2])
        glUniform1i(5,erase_range[3])
        
        glDispatchCompute(int(160/VOXEL_TRACE_INVOCAIONS), int(90/VOXEL_TRACE_INVOCAIONS), 1)
        
        # Sync
        # glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT)

        # ################# Distance stage ##################
        # glUseProgram(self.distance_program)
        
        # # # Set Uniforms
        # glUniform1i(0,erase_range[0])
        # glUniform1i(1,erase_range[1])
        # glUniform1i(2,erase_range[2])
        # glUniform1i(3,erase_range[3])
        # glUniform1i(4,erase_range[4])
        # glUniform1i(5,erase_range[5])
        
        # glDispatchCompute(int(SDF_EXTENTS/CELL_DISTANCE_INVOCAIONS), int(SDF_EXTENTS/CELL_DISTANCE_INVOCAIONS))
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
        low = self.sdf_buffer.min()
        diff = self.sdf_buffer.max() - low
        for x in range(SDF_EXTENTS):
            for y in range(SDF_EXTENTS):
                img[x,y] = (self.sdf_buffer[(x-self.sdf_index[0])%SDF_EXTENTS, (y-self.sdf_index[1])%SDF_EXTENTS] - low) / diff
                # img[x,y] = self.sdf_buffer[(x-self.sdf_index[0])%SDF_EXTENTS, (y-self.sdf_index[1])%SDF_EXTENTS]
        img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        cv2.namedWindow('SDF Slice', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('SDF Slice', 512,512)
        cv2.imshow('SDF Slice', img)
        cv2.waitKey(1)

    def update_new(self, global_pos, camera_quat, depth):
        self.generate_heightmap(depth, camera_quat, self.update_sdf_index(global_pos))
        self.display_heightmap()
        
    
    def update_sdf_index(self, global_pos):
        """Update local pos to match the global pos and clear old data"""
        local_new = to_sdf_index(global_pos)
        self.cell_offset = local_new % 1
        local_new = local_new.astype(int)
        diff = local_new - self.sdf_index
        self.sdf_index += diff
        # Find clear range
        #-------------------------------------------------------------
        x1=x2=y1=y2=0
        if diff[0] != 0:
            x1 = -self.sdf_index[0]
            x2 = x1 + diff[0]
            if x1 > x2:
                x1, x2 = swap(x1,x2)
            # self.sdf_buffer[x1:x2,:] = SDF_EXTENTS/2
            print(f"{x1} -- {x2}")
        if diff[1] != 0:
            y1 = -self.sdf_index[1]
            y2 = y1 + diff[1]
            if y1 > y2:
                y1, y2 = swap(y1,y2)
            # self.sdf_buffer[:,y1:y2] = SDF_EXTENTS/2
            print(f"{y1} -- {y2}")
        # if diff[2] != 0:
        #     z1 = -self.sdf_index[2]
        #     z2 = z1 + diff[2]
        #     if z1 > z2:
        #         z1, z2 = swap(z1,z2)
            # self.sdf_buffer[:,:,z1:z2] = 100
            # print(f"{z1} -- {z2}")
        #-------------------------------------------------------------
        # print(np.array([x1,x2,y1,y2,z1,z2], dtype=int)+120)
        
        return np.array([x1,x2,y1,y2], dtype=int) + SDF_EXTENTS



def swap(a,b):
    t = a
    a = b
    b = t
    return a, b