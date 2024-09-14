import numpy as np
from multiprocessing import shared_memory
from roboMath import rotate
# from simLaunch import RES_X, RES_Y
from OpenGL.GL import *
from OpenGL.GL.shaders import compileProgram,compileShader

import time
import numpy as np
import cv2
from math import copysign
# from scipy.spatial import Octree

from walkStateMachine import ANCHOR_CORRECTION_RADIUS, ANCHOR_CORRECTION_THRESHOLD
EXTENTS = 16                        # Extents of SDF block, in distance units
DIVISIOINS = 16                      # Cells per distance unit
INV_DIVISIOINS = 1/DIVISIOINS                # For faster computation
HMAP_EXTENTS = EXTENTS*DIVISIOINS    # Extents of SDF block, in number of cells

VOXEL_TRACE_INVOCAIONS = 32         # NB This must match the x and y invocations specified in voxel_trace.glsl
CELL_DISTANCE_INVOCAIONS = 32       # NB This must match the x and y invocations specified in set_cell_distance.glsl

def global_to_hmap(global_pos):
    """Convert global position to position in hmap grid, which has its corner at 0,0,0"""
    return (((global_pos)%(EXTENTS))*DIVISIOINS).astype(np.int32)

class Perception():
    def __init__(self, n_points) -> None:
        # Shared Memory buffers for communication with 3D visualisation process
        #---------------------------------------------------------------------------------
         # SDF grind, cell origin at lower corner
        hmap_buffer = np.zeros((HMAP_EXTENTS, HMAP_EXTENTS), dtype=np.float32)
        # hmap_buffer[:,:] = 0.2
        self.sdf_shm = shared_memory.SharedMemory(create=True,size=hmap_buffer.nbytes)
        self.hmap_buffer = np.ndarray(hmap_buffer.shape, dtype=np.float32, buffer=self.sdf_shm.buf)
        self.hmap_buffer[:] = hmap_buffer[:]
        self.score_buffer = np.zeros_like(self.hmap_buffer, dtype=float)
        # Index of current cell
        hmap_index = np.zeros(3, dtype=np.int32)
        self.hmap_index_shm = shared_memory.SharedMemory(create=True,size=hmap_index.nbytes)
        self.hmap_index = np.ndarray(hmap_index.shape, dtype=np.int32, buffer=self.hmap_index_shm.buf)
        self.hmap_index[:] = hmap_index[:]
        self.position = np.zeros(3)
        self.position_prev = np.zeros(3)
        self.body_quat = np.zeros(4)
        self.temporal_i = 0
        #---------------------------------------------------------------------------------
        
        self.cell_offset = np.zeros(3) # Position offset from cell origin
        self.heightmap_program = glCreateProgram()
        self.clean_heightmap_program = glCreateProgram()
        self.map_view = [0]

        # visualize window
        self.anchor_correction_windows = np.array((6,2))
        self.mouseX = 0
        self.mouseY = 0
        cv2.namedWindow('Heightmap', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('Heightmap', self._get_hmap_mouse_pos)
        self._init_shader(n_points)


    # initialise compute shader
    def _init_shader(self, n_points):
        # Compile cell trace program
        heightmap_src = open('compute/heightmap.glsl','r').readlines()
        self.heightmap_program = compileProgram(compileShader(heightmap_src, GL_COMPUTE_SHADER))

        # Compile map cleaning program
        clean_heightmap_src = open('compute/clean_heightmap.glsl','r').readlines()
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


        # Bind scores buffer
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, self.glbuffers[3])
        glBufferData(GL_SHADER_STORAGE_BUFFER, self.hmap_buffer.nbytes * 4, None, GL_DYNAMIC_READ)


        # Bind points buffer
        # glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, self.glbuffers[2])
        # glBufferData(GL_SHADER_STORAGE_BUFFER, n_points * 4 * 4, None, GL_DYNAMIC_COPY)
 
 
    def _generate_heightmap(self, depth, camera_quat, cam_hmap_i, cam_height):
        # Set depth image data
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.glbuffers[0])        
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, depth.nbytes, depth)

        ################### Trace stage ##################
        glUseProgram(self.heightmap_program)
        
        # Set Uniforms
        glUniform3i(0, cam_hmap_i[0], cam_hmap_i[1], cam_hmap_i[2])                     # Camera position
        glUniform4f(1, camera_quat[0], camera_quat[1], camera_quat[2], camera_quat[3])  # Camera rotation
        glUniform1i(2, self.temporal_i)
        glUniform1f(3, cam_height)
        
        glDispatchCompute(int((160)/VOXEL_TRACE_INVOCAIONS), int((90)/VOXEL_TRACE_INVOCAIONS), 1)
        
        # Sync
        # glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT)

        # ################# Distance stage ##################
        glUseProgram(self.clean_heightmap_program)
        
        # Set Uniforms
        glUniform3i(0, self.hmap_index[0], self.hmap_index[1], self.hmap_index[2])         # Robot position
        
        glDispatchCompute(int(HMAP_EXTENTS/CELL_DISTANCE_INVOCAIONS), int(HMAP_EXTENTS/CELL_DISTANCE_INVOCAIONS), 1)

        # Sync, read SDF
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.glbuffers[1])
        self.hmap_buffer[:] = np.frombuffer(glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, self.hmap_buffer.nbytes), dtype=self.hmap_buffer.dtype).reshape(HMAP_EXTENTS,HMAP_EXTENTS)[:]
        
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.glbuffers[3])
        self.score_buffer[:] = np.frombuffer(glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, self.hmap_buffer.nbytes), dtype=self.hmap_buffer.dtype).reshape(HMAP_EXTENTS,HMAP_EXTENTS)[:]

    # HACK for visualisation of feet
    def add_refs(self, walkmachine):
        self.walmachine = walkmachine
    
    def _local_to_hmap(self, local_pos):
        temp = (np.round(global_to_hmap(rotate(self.body_quat, -local_pos))) - self.hmap_index + int(HMAP_EXTENTS*0.5))[:2]
        return  temp % HMAP_EXTENTS, temp
    
    def _hmap_to_local(self, hmap_pos_raw):
        diff = hmap_pos_raw + self.hmap_index[0:2] - int(HMAP_EXTENTS*0.5)
        
        # x_new = 0
        # y_new = 0
        
        # if abs(diff[0]) > HMAP_EXTENTS*0.5: 
        #     x_new = -INV_DIVISIOINS*( diff[0] % -HMAP_EXTENTS )
        # else: 
        #     x_new = -INV_DIVISIOINS*( diff[0] % HMAP_EXTENTS )

        # if abs(diff[1]) > HMAP_EXTENTS*0.5: 
        #     y_new = -INV_DIVISIOINS*( diff[1] % -HMAP_EXTENTS )
        # else: 
        #     y_new = -INV_DIVISIOINS*( diff[1] % HMAP_EXTENTS )
        # anchor_new = np.array([x_new, y_new, self.hmap_buffer[hmap_pos_raw[0]%HMAP_EXTENTS, hmap_pos_raw[1]%HMAP_EXTENTS]])
        
        anchor_new = np.append(-INV_DIVISIOINS*diff, 0.0)
        if abs(anchor_new[0]) > EXTENTS*0.5:
            anchor_new[0] -= np.sign(anchor_new[0])*EXTENTS
        if abs(anchor_new[1]) > EXTENTS*0.5:
            anchor_new[1] -= np.sign(anchor_new[1])*EXTENTS
        
        
        
        return rotate(self.body_quat*[-1,-1,-1,1], anchor_new)
    
    def _get_hmap_mouse_pos(self, event,x,y,flags,param):
        self.mouseX, self.mouseY = x,y

    def _display_heightmap(self):
        # img = np.ones((self.hmap_buffer.shape[0],self.hmap_buffer.shape[1],3), dtype=np.float32)
        buffer = None
        img_max = 1.5
        if self.map_view[0] == 0:
            buffer = self.hmap_buffer
        else:
            buffer = self.score_buffer
            img_max = 5.0

        img = -buffer[..., np.newaxis]
        img = np.concatenate((img,img,img), axis=2)
        img = np.clip((img - 0)/(img - img_max), 0.0, img_max)

        # TODO ~70ms very slow, make better
        # for x in range(HMAP_EXTENTS):
        #     for y in range(HMAP_EXTENTS):
        #         img[x,y] *= (self.hmap_buffer[(x-self.hmap_index[0])%HMAP_EXTENTS, (y-self.hmap_index[1])%HMAP_EXTENTS] + 2) / 4
        #         img[x,y] = self.hmap_buffer[(x-self.hmap_index[0])%HMAP_EXTENTS, (y-self.hmap_index[1])%HMAP_EXTENTS]

        # # Draw foot targets
        for i in range(6):
            # hmap_i = self._local_to_hmap(self.walmachine.foot_pos_post_yaw[i])
            hmapt_init = self.walmachine.targets_init_map[i]
            hmap_i, _ = self._local_to_hmap(self.walmachine.foot_pos_post_yaw[i])
            
            # img_i = (hmap_i-self.hmap_index)%HMAP_EXTENTS   # Hold in center of centered image
            img[int(hmap_i[0]), int(hmap_i[1])] = np.array([1,0,0])
            
            if (self.walmachine.is_swinging[i]):
                img[int(hmapt_init[0]), int(hmapt_init[1])] = np.array([0,1,0])
                box_color = np.array([0,1,0])
                if self.walmachine.is_move_valid:
                    hmapt, _ = self._local_to_hmap(self.walmachine.targets[i])
                    img[int(hmapt[0]), int(hmapt[1])] = np.array([1,1,0])
                else:
                    box_color = np.array([0,0,1])
                # Draw search block
                for r in range(-ANCHOR_CORRECTION_RADIUS-1, ANCHOR_CORRECTION_RADIUS+2):
                    img[int(hmapt_init[0]+r) % HMAP_EXTENTS, int(hmapt_init[1]+ANCHOR_CORRECTION_RADIUS+1) % HMAP_EXTENTS] = box_color
                    img[int(hmapt_init[0]+r) % HMAP_EXTENTS, int(hmapt_init[1]-ANCHOR_CORRECTION_RADIUS-1) % HMAP_EXTENTS] = box_color
                    img[int(hmapt_init[0]+ANCHOR_CORRECTION_RADIUS+1) % HMAP_EXTENTS, int(hmapt_init[1]+r) % HMAP_EXTENTS] = box_color
                    img[int(hmapt_init[0]-ANCHOR_CORRECTION_RADIUS-1) % HMAP_EXTENTS, int(hmapt_init[1]+r) % HMAP_EXTENTS] = box_color
            else:
                img[int(hmapt_init[0]), int(hmapt_init[1])] = np.array([0,0,1])
        
        # ~8ms
        # img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        # x = (self.mouseX + int(HMAP_EXTENTS*0.5) - hmap_i[0])%HMAP_EXTENTS
        # y = (self.mouseY + int(HMAP_EXTENTS*0.5) - hmap_i[1])%HMAP_EXTENTS
        # img
        # print('max', buffer.max())
        # print(self.mouseY, self.mouseX ,buffer[self.mouseY, self.mouseX])
        # print(self.mouseX, self.mouseY)
        
        # img[self.mouseY, self.mouseX] = [255,0,0]
        # new_anchor = (self.find_anchor(np.array([self.mouseY, self.mouseX,0]), ANCHOR_CORRECTION_RADIUS, ANCHOR_CORRECTION_THRESHOLD))%HMAP_EXTENTS
        # img[new_anchor[0], new_anchor[1]] = [0,1,1]
        cv2.imshow('Heightmap', (img * 255).astype(np.uint8))
        cv2.waitKey(1)

    def update(self, camera_pos, body_pos, camera_quat, body_quat, depth):
        self.body_quat = body_quat
        self.hmap_index = global_to_hmap(body_pos)
        self.position = body_pos
        self._generate_heightmap(depth, camera_quat, global_to_hmap(camera_pos), camera_pos[2])
        self._display_heightmap()
        self.temporal_i = int((self.temporal_i + 1)%4)
        # print(self.hmap_index)

    def get_height_at_point(self, point):
        """Returns the height at a point relative to the robot center, the height is in world space"""
        hmap_i, _ = self._local_to_hmap(point)
        h = self.hmap_buffer[hmap_i[0], hmap_i[1]]
        return h
    
    def find_anchor(self, anchor, search_rad, threshold):
        """Find first value under threshold within search radius, expanding square"""
        # Early exit if initial point is valid
        anchor_map, anchor_map_raw = self._local_to_hmap(anchor)[0:2]
        anchor_map_x = anchor_map[0]
        anchor_map_y = anchor_map[1]
        score_max = score_max = self._find_block_max(anchor_map_x, anchor_map_y)
        if score_max < threshold:
            # print(score_max, "Valid")
            return self._local_to_hmap(anchor)[0]

        minval = 1000 # Some sufficiently large number
        for r in range(search_rad):
            for i in range(r+1):
                # Left side
                should_return, minval = self._compare((anchor_map_y+i)%HMAP_EXTENTS, (anchor_map_x-r)%HMAP_EXTENTS, minval, threshold)
                if should_return: return np.array([anchor_map_raw[0]-r, anchor_map_raw[1]+i])
                should_return, minval = self._compare((anchor_map_y-i)%HMAP_EXTENTS, (anchor_map_x-r)%HMAP_EXTENTS, minval, threshold)
                if should_return: return np.array([anchor_map_raw[0]-r, anchor_map_raw[1]-i])

                # Right side
                should_return, minval = self._compare((anchor_map_y+i)%HMAP_EXTENTS, (anchor_map_x+r)%HMAP_EXTENTS, minval, threshold)
                if should_return: return np.array([anchor_map_raw[0]+r, anchor_map_raw[1]+i])
                should_return, minval = self._compare((anchor_map_y-i)%HMAP_EXTENTS, (anchor_map_x+r)%HMAP_EXTENTS, minval, threshold)
                if should_return: return np.array([anchor_map_raw[0]+r, anchor_map_raw[1]-i])
                
                # Top side
                should_return, minval = self._compare((anchor_map_y+r)%HMAP_EXTENTS, (anchor_map_x+i)%HMAP_EXTENTS, minval, threshold)
                if should_return: return np.array([anchor_map_raw[0]+i, anchor_map_raw[1]+r])
                should_return, minval = self._compare((anchor_map_y+r)%HMAP_EXTENTS, (anchor_map_x-i)%HMAP_EXTENTS, minval, threshold)
                if should_return: return np.array([anchor_map_raw[0]-i, anchor_map_raw[1]+r])

                # Bottom side
                should_return, minval = self._compare((anchor_map_y-r)%HMAP_EXTENTS, (anchor_map_x+i)%HMAP_EXTENTS, minval, threshold)
                if should_return: return np.array([anchor_map_raw[0]+i, anchor_map_raw[1]-r])
                should_return, minval = self._compare((anchor_map_y-r)%HMAP_EXTENTS, (anchor_map_x-i)%HMAP_EXTENTS, minval, threshold)
                if should_return: return np.array([anchor_map_raw[0]-i, anchor_map_raw[1]-r])

        return (np.array([-1, -1]))
    
    def _find_block_max(self, x,y):
        score_block = np.full(9,1000.0)
        score_block[0] = self.score_buffer[(x-1)%HMAP_EXTENTS, (y+1)%HMAP_EXTENTS]
        score_block[1] = self.score_buffer[x, (y+1)%HMAP_EXTENTS]
        score_block[2] = self.score_buffer[(x+1)%HMAP_EXTENTS, (y+1)%HMAP_EXTENTS]
        score_block[3] = self.score_buffer[(x-1)%HMAP_EXTENTS, y]
        score_block[4] = self.score_buffer[x, y]
        score_block[5] = self.score_buffer[(x+1)%HMAP_EXTENTS, y]
        score_block[6] = self.score_buffer[(x-1)%HMAP_EXTENTS, (y-1)%HMAP_EXTENTS]
        score_block[7] = self.score_buffer[x, (y-1)%HMAP_EXTENTS]
        score_block[8] = self.score_buffer[(x+1)%HMAP_EXTENTS, (y-1)%HMAP_EXTENTS]
        return score_block.max()

    def _compare(self, y, x, minval, threshold):
        """Helper function for _find_min()"""

        # Find max score in block
        score_max = self._find_block_max(x,y)

        if score_max < minval:
            minval = score_max
            if minval < threshold:
                # print(score_max, "Alter")
                return True, minval
        return False, minval