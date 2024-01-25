import open3d as o3d
import numpy as np
import time
from perception import HMAP_EXTENTS, EXTENTS, DIVISIOINS
import cv2

from multiprocessing import shared_memory

keep_running = True

def update_points_buffer(sdf_index, sdf_buffer, pcd, erase_markers):
    if sdf_buffer.min() <= 0:
        index = np.where(sdf_buffer <= 0)
        index[0][:] = (index[0][:] + sdf_index[0])%HMAP_EXTENTS
        index[1][:] = (index[1][:] + sdf_index[1])%HMAP_EXTENTS
        index[2][:] = (index[2][:] + sdf_index[2])%HMAP_EXTENTS
        # index[0][:] = (index[0][:])%HMAP_EXTENTS
        # index[1][:] = (index[1][:])%HMAP_EXTENTS
        # index[2][:] = (index[2][:])%HMAP_EXTENTS
        points = np.transpose(index)/DIVISIOINS - EXTENTS/2
<<<<<<< HEAD
        x = (sdf_index[0]-SDF_EXTENTS/2)/DIVISIOINS
        y = (sdf_index[1]-SDF_EXTENTS/2)/DIVISIOINS
=======



        
        x = (sdf_index[0]-HMAP_EXTENTS/2)/DIVISIOINS
        y = (sdf_index[1]-HMAP_EXTENTS/2)/DIVISIOINS
>>>>>>> 82761bea7d66da5f543ff4cc52ceb95ac7a0a2f2
        erase_markers.points = o3d.utility.Vector3dVector(np.array([[x,50,0],[x,-50,0], [50,y,0],[-50,y,0]]))
        pcd.points = o3d.utility.Vector3dVector(-points)



def start(sdf_shmn, sdf_index_shmn):
    sdf_shm = shared_memory.SharedMemory(name=sdf_shmn)
    # points_buffer = np.ndarray((int((1280*720)/4), 3,), dtype=np.float64, buffer=sdf_shm.buf)
    sdf_buffer = np.ndarray((HMAP_EXTENTS,HMAP_EXTENTS,HMAP_EXTENTS), dtype=np.float32, buffer=sdf_shm.buf)
    
    sdf_index_shm = shared_memory.SharedMemory(name=sdf_index_shmn)
    sdf_index = np.ndarray(3, dtype=np.int32, buffer=sdf_index_shm.buf)

    # create visualizer and window.
    vis = o3d.visualization.Visualizer()
    vis.create_window(height=480, width=640)
    time.sleep(2)

    # initialize pointcloud instance.
    pcd = o3d.geometry.PointCloud()


    points_buffer = np.zeros((HMAP_EXTENTS*HMAP_EXTENTS*HMAP_EXTENTS + 12, 3))
    pos3 = np.zeros((HMAP_EXTENTS*HMAP_EXTENTS*HMAP_EXTENTS + 12, 3))
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
    # color_buffer = np.zeros((HMAP_EXTENTS*HMAP_EXTENTS*HMAP_EXTENTS + 8, 3))
    # color_buffer[0:9] = np.array([1,0.5,0.5])

    pcd.points = o3d.utility.Vector3dVector(pos3)
    # pcd.colors = o3d.utility.Vector3dVector(color_buffer)

    # include it in the visualizer before non-blocking visualization.
    vis.add_geometry(pcd)
    vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(6))
    markers = o3d.geometry.LineSet()
    markers.points = o3d.utility.Vector3dVector(np.array([[-EXTENTS/2,50,0], [-EXTENTS/2,-50,0], [EXTENTS/2,50,0], [EXTENTS/2,-50,0], [50,-EXTENTS/2,0], [-50,-EXTENTS/2,0], [-50,EXTENTS/2,0], [50,EXTENTS/2,0]]))
    markers.lines = o3d.utility.Vector2iVector(np.array([[0,1],[2,3],[4,5],[6,7]], dtype=np.int32))
    erase_markers = o3d.geometry.LineSet()
    erase_markers.points = o3d.utility.Vector3dVector(np.array([[0,0,0],[0,0,0], [0,0,0],[0,0,0]]))
    erase_markers.colors = o3d.utility.Vector3dVector(np.array([[0,1,0],[1,0,0]]))
    erase_markers.lines = o3d.utility.Vector2iVector(np.array([[0,1],[2,3]], dtype=np.int32))
    vis.add_geometry(markers)
    vis.add_geometry(erase_markers)

    # to add new points each dt secs.
    dt = 0.02

    # run non-blocking visualization. 
    # To exit, press 'q' or click the 'x' of the window.
    previous_t = time.time()
    global keep_running
    while keep_running:
        # if time.time() - previous_t > dt:
        # print(f"Index: {sdf_index}")
        update_points_buffer(sdf_index, sdf_buffer, pcd, erase_markers)
            
        vis.update_geometry(pcd)
        vis.update_geometry(erase_markers)
        # previous_t = time.time()

        keep_running = vis.poll_events()
        vis.update_renderer()
    vis.destroy_window()
    sdf_shm.close()
    sdf_index_shm.close()
