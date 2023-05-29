import open3d as o3d
import numpy as np
import time
from perception import SDF_EXTENTS, EXTENTS, DIVISIOINS

from multiprocessing import shared_memory
import threading

keep_running = True

def update_points_buffer(sdf_index, sdf_buffer, pcd):
    if sdf_buffer.min() <= 0:
        index = np.where(sdf_buffer <= 0)
        index[0][:] = (index[0][:] + sdf_index[0])%SDF_EXTENTS
        index[1][:] = (index[1][:] + sdf_index[1])%SDF_EXTENTS
        index[2][:] = (index[2][:] + sdf_index[2])%SDF_EXTENTS
        # index_flat = index[0][:] + index[1][:]*SDF_EXTENTS + index[2][:]*SDF_EXTENTS*SDF_EXTENTS
        points = np.transpose(index)/DIVISIOINS - EXTENTS/2
        pos3 = np.array(points)
        pos3[:,0] = -points[:,0]
        pos3[:,1] = -points[:,1]
        pos3[:,2] = -points[:,2]
        pcd.points = o3d.utility.Vector3dVector(pos3)



def start(sdf_shmn, sdf_index_shmn):
    sdf_shm = shared_memory.SharedMemory(name=sdf_shmn)
    # points_buffer = np.ndarray((int((1280*720)/4), 3,), dtype=np.float64, buffer=sdf_shm.buf)
    sdf_buffer = np.ndarray((SDF_EXTENTS,SDF_EXTENTS,SDF_EXTENTS), dtype=np.float32, buffer=sdf_shm.buf)
    
    sdf_index_shm = shared_memory.SharedMemory(name=sdf_index_shmn)
    sdf_index = np.ndarray(3, dtype=np.int8, buffer=sdf_index_shm.buf)

    # create visualizer and window.
    vis = o3d.visualization.Visualizer()
    vis.create_window(height=480, width=640)
    time.sleep(2)

    # initialize pointcloud instance.
    pcd = o3d.geometry.PointCloud()


    points_buffer = np.zeros((SDF_EXTENTS*SDF_EXTENTS*SDF_EXTENTS + 12, 3))
    pos3 = np.zeros((SDF_EXTENTS*SDF_EXTENTS*SDF_EXTENTS + 12, 3))
    points_buffer[0] = np.array([15.5, 15.5, 15.5])
    points_buffer[1] = np.array([15.5, 15.5, -15.5])
    points_buffer[2] = np.array([15.5, -15.5, -15.5])
    points_buffer[3] = np.array([-15.5, 15.5, -15.5])
    points_buffer[4] = np.array([-15.5, -15.5, 15.5])
    points_buffer[5] = np.array([-15.5, -15.5, -15.5])
    points_buffer[6] = np.array([15.5, -15.5, 15.5])
    points_buffer[7] = np.array([-15.5, 15.5, 15.5])
    
    # Swap for correct visualisation
    pos3[:,0] = -points_buffer[:,1]
    pos3[:,1] = points_buffer[:,2]
    pos3[:,2] = -points_buffer[:,0]

    # Coordinate Frame Colors
    color_buffer = np.zeros((SDF_EXTENTS*SDF_EXTENTS*SDF_EXTENTS + 8, 3))
    color_buffer[0:9] = np.array([1,0.5,0.5])

    pcd.points = o3d.utility.Vector3dVector(pos3)
    pcd.colors = o3d.utility.Vector3dVector(color_buffer)

    # include it in the visualizer before non-blocking visualization.
    vis.add_geometry(pcd)
    vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(15))

    # to add new points each dt secs.
    dt = 0.02

    # run non-blocking visualization. 
    # To exit, press 'q' or click the 'x' of the window.
    previous_t = time.time()
    global keep_running
    while keep_running:
        # if time.time() - previous_t > dt:
        # print(f"Index: {sdf_index}")
        update_points_buffer(sdf_index, sdf_buffer, pcd)
            
        vis.update_geometry(pcd)
        # previous_t = time.time()

        keep_running = vis.poll_events()
        vis.update_renderer()
    vis.destroy_window()
    sdf_shm.close()
    sdf_index_shm.close()
