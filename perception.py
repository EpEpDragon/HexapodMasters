import numpy as np
from multiprocessing import shared_memory
from roboMath import rotate_vec, rotate_vec_quat
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

    
    def update(self, global_pos, body_quaternion, points):
        # f = lambda x: rotate_vec(x, np.array([1,0,0]), -0.54)
        f2 = lambda x: rotate_vec_quat(x, body_quaternion)
        self.update_sdf_index(global_pos)
        indices = ((f2(points) + EXTENTS/2)*DIVISIOINS).astype(int)
        # self.sdf_buffer[:] = 100
        self.sdf_buffer[(indices[:,0] - self.sdf_index[0])%SDF_EXTENTS, (indices[:,1] - self.sdf_index[1])%SDF_EXTENTS, (indices[:,2] - self.sdf_index[2])%SDF_EXTENTS] = 0.0
        

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

        
        # print(f"Global: {np.array2string(global_pos,precision=2, floatmode='fixed')}        Index: {self.sdf_index}         Offset: {np.array2string(self.cell_offset,precision=2,floatmode='fixed')}")
        # print(diff)

def swap(a,b):
    t = a
    a = b
    b = t
    return a, b