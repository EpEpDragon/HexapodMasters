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
HMAP_EXTENTS = EXTENTS*DIVISIOINS    # Extents of SDF block, in number of cells

VOXEL_TRACE_INVOCAIONS = 32         # NB This must match the x and y invocations specified in voxel_trace.glsl
CELL_DISTANCE_INVOCAIONS = 32       # NB This must match the x and y invocations specified in set_cell_distance.glsl

def to_hmap_index(global_pos):
    """Convert global position to position in SDF grid, which has its corner at 0,0,0"""
    return (((global_pos)%(EXTENTS))*DIVISIOINS).astype(np.int32)


class Perception():
    def __init__(self, n_points) -> None:
        # Shared Memory buffers for communication with 3D visualisation process
        #---------------------------------------------------------------------------------
         # SDF grind, cell origin at lower corner
        hmap_buffer = np.zeros((HMAP_EXTENTS, HMAP_EXTENTS), dtype=np.float32)
        self.sdf_shm = shared_memory.SharedMemory(create=True,size=hmap_buffer.nbytes)
        self.hmap_buffer = np.ndarray(hmap_buffer.shape, dtype=np.float32, buffer=self.sdf_shm.buf)
        self.hmap_buffer[:] = hmap_buffer[:]
        # Index of current cell
        hmap_index = np.zeros(3, dtype=np.int32)
        self.hmap_index_shm = shared_memory.SharedMemory(create=True,size=hmap_index.nbytes)
        self.hmap_index = np.ndarray(hmap_index.shape, dtype=np.int32, buffer=self.hmap_index_shm.buf)
        self.hmap_index[:] = hmap_index[:]
        self.position = np.zeros(3)
        #---------------------------------------------------------------------------------
        
        self.cell_offset = np.zeros(3) # Position offset from cell origin
        self.heightmap_program = glCreateProgram()
        self.clean_heightmap_program = glCreateProgram()

        # visualize window
        cv2.namedWindow('SDF Slice', cv2.WINDOW_NORMAL)
        self._init_shader(n_points)


    # initialise compute shader
    def _init_shader(self, n_points):
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
        glBufferData(GL_SHADER_STORAGE_BUFFER, self.hmap_buffer.nbytes, self.hmap_buffer, GL_DYNAMIC_READ)
        
        # Bind points buffer
        # glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, self.glbuffers[2])
        # glBufferData(GL_SHADER_STORAGE_BUFFER, n_points * 4 * 4, None, GL_DYNAMIC_COPY)
 
 
    def _generate_heightmap(self, depth, camera_quat):
        # Set depth image data
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.glbuffers[0])        
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, depth.nbytes, depth)

        ################### Trace stage ##################
        glUseProgram(self.heightmap_program)
        
        # Set Uniforms
        glUniform3i(0, self.hmap_index[0], self.hmap_index[1], self.hmap_index[2])         # Robot position
        glUniform4f(1, camera_quat[0], camera_quat[1], camera_quat[2], camera_quat[3])  # Robot rotation
        
        glDispatchCompute(int((160)/VOXEL_TRACE_INVOCAIONS), int((90)/VOXEL_TRACE_INVOCAIONS), 1)
        
        # Sync
        # glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT)

        # ################# Distance stage ##################
        glUseProgram(self.clean_heightmap_program)
        
        # # # Set Uniforms
        glUniform3i(0, self.hmap_index[0], self.hmap_index[1], self.hmap_index[2])         # Robot position
        
        glDispatchCompute(int(HMAP_EXTENTS/CELL_DISTANCE_INVOCAIONS), int(HMAP_EXTENTS/CELL_DISTANCE_INVOCAIONS), 1)

        # Sync, read SDF
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.glbuffers[1])
        self.hmap_buffer[:] = np.frombuffer(glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, self.hmap_buffer.nbytes), dtype=self.hmap_buffer.dtype).reshape(HMAP_EXTENTS,HMAP_EXTENTS)[:]
        # glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.glbuffers[2])
        # points = np.frombuffer(glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, 160*90*4*4), dtype=np.float32).reshape((160*90,4))
        # points_tree = KDTree(points)
        # test = np.indices(self.hmap_buffer.shape)
        # points_tree.query()
    
    def _display_heightmap(self):
        img = np.zeros(self.hmap_buffer.shape)
        low = self.hmap_buffer.max()
        diff = self.hmap_buffer.min() - low
        for x in range(HMAP_EXTENTS):
            for y in range(HMAP_EXTENTS):
                img[x,y] = (self.hmap_buffer[(x-self.hmap_index[0])%HMAP_EXTENTS, (y-self.hmap_index[1])%HMAP_EXTENTS] - low) / diff
                # img[x,y] = self.hmap_buffer[(x-self.hmap_index[0])%HMAP_EXTENTS, (y-self.hmap_index[1])%HMAP_EXTENTS]
        img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        cv2.imshow('SDF Slice', img)
        cv2.waitKey(1)

    def update(self, global_pos, camera_quat, depth):
        self.hmap_index = to_hmap_index(global_pos)
        self.position = global_pos%EXTENTS
        self._generate_heightmap(depth, camera_quat)
        self._display_heightmap()


    def get_height_at_point(self, point):
        """Returns the height at a point relative to the robot center"""
        hmap_i = (to_hmap_index(point) + self.hmap_index) % HMAP_EXTENTS    # Transfer point to hmap space
        h = self.hmap_buffer[hmap_i[0], hmap_i[1]]/DIVISIOINS               # Addition due to mismatched axis systems 
        return h
