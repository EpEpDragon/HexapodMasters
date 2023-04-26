import open3d as o3d
import numpy as np
import time
from multiprocessing import shared_memory, Process, Lock

def start(shr_name):
    existing_shm = shared_memory.SharedMemory(name=shr_name)    
    points_buffer = np.ndarray((int((1280*720)/4), 3,), dtype=np.float64, buffer=existing_shm.buf)

    # create visualizer and window.
    vis = o3d.visualization.Visualizer()
    vis.create_window(height=480, width=640)
    time.sleep(2)

    # control = vis.get_view_control()
    # control.set_constant_z_far(10)


    # initialize pointcloud instance.
    pcd = o3d.geometry.PointCloud()
    # *optionally* add initial points
    # points = np.random.rand(10, 3)
    pcd.points = o3d.utility.Vector3dVector(points_buffer)

    # include it in the visualizer before non-blocking visualization.
    vis.add_geometry(pcd)

    # to add new points each dt secs.
    dt = 0.02
    # number of points that will be added
    n_new = 1000

    # run non-blocking visualization. 
    # To exit, press 'q' or click the 'x' of the window.
    previous_t = time.time()
    keep_running = True
    while keep_running:
        
        if time.time() - previous_t > dt:
            # pcd.points = o3d.utility.Vector3dVector(points)
            # temp = np.array((points[:,0]; points[:,2], points[:,1]))
            pcd.points = o3d.utility.Vector3dVector(points_buffer)
            
            vis.update_geometry(pcd)
            previous_t = time.time()

        keep_running = vis.poll_events()
        vis.update_renderer()
    vis.destroy_window()
    existing_shm.close()