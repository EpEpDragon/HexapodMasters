import open3d as o3d
import numpy as np
import threading
import cv2

# Input
class KeyboardThread(threading.Thread):
    def __init__(self, input_cbk = None, name='keyboard-input-thread'):
        self.input_cbk = input_cbk
        super(KeyboardThread, self).__init__(name=name)
        self.start()

    def run(self):
        while True:
            self.input_cbk(input()) #waits to get input + Return

MOVE_STEP = 5
def my_callback(inp):
    # Commands
    if inp == '\x1b[D':
        adjust_local_pos(local_pos, np.array([-MOVE_STEP,0,0]))
        print(f"Left: {local_pos}")
    elif inp == '\x1b[C':
        adjust_local_pos(local_pos, np.array([MOVE_STEP,0,0]))
        print(f"Left: {local_pos}")
    elif inp == ("\x1b[A"):
        adjust_local_pos(local_pos, np.array([0,MOVE_STEP,0]))
        print(f"Forwards: {local_pos}")
    elif inp == ("\x1b[B"):
        adjust_local_pos(local_pos, np.array([0,-MOVE_STEP,0]))
        print(f"Backwards: {local_pos}")
    elif inp == ("q"):
        adjust_local_pos(local_pos, np.array([0,0,MOVE_STEP]))
        print(f"Up: {local_pos}")
    elif inp == ("a"):
        adjust_local_pos(local_pos, np.array([0,0,-MOVE_STEP]))
        print(f"Down: {local_pos}")
    elif inp == 'clear':
        clear_sdf()
        update_viz()
    elif inp == 'spawn':
        add_block(np.array([0,0,0]),np.array([5,5,5]))
        update_viz()
    elif inp == 'center':
        local_pos[:] = 0
        update_viz()
    elif inp == 'red':
        pcd.paint_uniform_color([1,0,0])
        update_viz()
    elif inp == 'green':
        pcd.paint_uniform_color([0,1,0])
        update_viz()
    elif inp == 'blue':
        pcd.paint_uniform_color([0,0,1])
        update_viz()
    elif inp == 'black':
        pcd.paint_uniform_color([0,0,0])
        update_viz()
    elif inp == 'gray':
        pcd.paint_uniform_color([0.5,0.5,0.5])
        update_viz()

# Move in vector
def adjust_local_pos(local_pos, vector):    
    local_pos += vector
    # Wrap local pos around range
    local_pos[:] = (local_pos%EXTENTS)[:]
    
    # Clear required cells
    #-------------------------------------------------------------
    # Y axis
    if vector[0] > 0:
        start = (local_pos[0]+EXTENTS-MOVE_STEP)%(EXTENTS)
        end = (local_pos[0]+EXTENTS)%(EXTENTS)
        if end == 0:
            end = EXTENTS
        sdf_buffer[start:end,:,:] = 100
    elif vector[0] < 0:
        start = (local_pos[0])%(EXTENTS)
        end = (local_pos[0]+MOVE_STEP)%(EXTENTS)
        if end == 0:
            end = EXTENTS
        sdf_buffer[start:end,:,:] = 100
    
    # X axis
    if vector[1] > 0:
        start = (local_pos[1]+EXTENTS-MOVE_STEP)%(EXTENTS)
        end = (local_pos[1]+EXTENTS)%(EXTENTS)
        if end == 0:
            end = EXTENTS
        sdf_buffer[:,start:end,:] = 100
    elif vector[1] < 0:
        start = (local_pos[1])%(EXTENTS)
        end = (local_pos[1]+MOVE_STEP)%(EXTENTS)
        if end == 0:
            end = EXTENTS
        sdf_buffer[:,start:end,:] = 100

    # Z axis
    if vector[2] > 0:
        start = (local_pos[2]+EXTENTS-MOVE_STEP)%(EXTENTS)
        end = (local_pos[2]+EXTENTS)%(EXTENTS)
        if end == 0:
            end = EXTENTS
        sdf_buffer[:,:,start:end] = 100
    elif vector[2] < 0:
        start = (local_pos[2])%(EXTENTS)
        end = (local_pos[2]+MOVE_STEP)%(EXTENTS)
        if end == 0:
            end = EXTENTS
        sdf_buffer[:,:,start:end] = 100
    #-------------------------------------------------------------
    update_viz()

# Add block at current position
def add_block(center_xyz, extent_xyz):
    start_xyz = center_xyz + local_pos - (extent_xyz/2).astype(int) + (np.array(sdf_buffer.shape)/2).astype(int)
    end_xyz = start_xyz + extent_xyz
    
    for x in range(start_xyz[0], end_xyz[0]):
        for y in range(start_xyz[1], end_xyz[1]):
            for z in range(start_xyz[2], end_xyz[2]):
                sdf_buffer[x%EXTENTS,y%EXTENTS,z%EXTENTS] = 0


def clear_sdf():
    sdf_buffer[:] = 10


def update_viz():
    for i in range(sdf_buffer.shape[0]):
        for j in range(sdf_buffer.shape[1]):
            for k in range(sdf_buffer.shape[2]):
                x = (i-local_pos[0])%EXTENTS
                y = (j-local_pos[1])%EXTENTS
                z = (k-local_pos[2])%EXTENTS
                index = int(x + y*sdf_buffer.shape[0] + z*sdf_buffer.shape[0]*sdf_buffer.shape[1])
                if sdf_buffer[i,j,k] <= 0:
                    points_buffer[index] = np.array([x,y,z])
                else:
                    points_buffer[index] = np.array([100,100,100])
    pcd.points = o3d.utility.Vector3dVector(points_buffer)
    vis.update_geometry(pcd)
    vis.update_renderer()

# Setup
#-----------------------------------------------------------------------
#start the Keyboard thread
kthread = KeyboardThread(my_callback)
EXTENTS = 30
pcd = o3d.geometry.PointCloud()
# pcd = o3d.geometry.VoxelGrid()
sdf_buffer = np.zeros((EXTENTS,EXTENTS,EXTENTS))
points_buffer_size = (sdf_buffer.shape[0]*sdf_buffer.shape[0]*sdf_buffer.shape[0], 3)
points_buffer = np.empty(points_buffer_size)

# create visualizer and window.
vis = o3d.visualization.Visualizer()
vis.create_window(height=480, width=640)

for i in range(sdf_buffer.shape[0]):
    for j in range(sdf_buffer.shape[1]):
        for k in range(sdf_buffer.shape[2]):
            points_buffer[i + j*sdf_buffer.shape[0] + k*sdf_buffer.shape[0]*sdf_buffer.shape[1]] = np.array([i,j,k])
pcd.points = o3d.utility.Vector3dVector(points_buffer)
pcd.paint_uniform_color(np.array([0,0,0]))
# pcd.colors = o3d.utility.Vector3dVector(np.full(points_buffer_size,0))

# include it in the visualizer before non-blocking visualization.
vis.add_geometry(pcd)
#-----------------------------------------------------------------------
local_pos = np.zeros(3,dtype=int)

# run non-blocking visualization. 
# To exit, press 'q' or click the 'x' of the window.
keep_running = True

while keep_running:
    keep_running = vis.poll_events()
vis.destroy_window()