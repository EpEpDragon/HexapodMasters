import open3d as o3d
import numpy as np
import time
from perception import SDF_EXTENTS, EXTENTS, DIVISIOINS

from multiprocessing import shared_memory
import threading

keep_running = True

def update_points_buffer(points_buffer, pos3, sdf_index, sdf_buffer, pcd):
    while keep_running:
        for i in range(sdf_buffer.shape[0]):
            for j in range(sdf_buffer.shape[1]):
                for k in range(sdf_buffer.shape[2]):
                    x = (i-sdf_index[0])%SDF_EXTENTS 
                    y = (j-sdf_index[1])%SDF_EXTENTS
                    z = (k-sdf_index[2])%SDF_EXTENTS
                    index = int(x + y*SDF_EXTENTS + z*SDF_EXTENTS*SDF_EXTENTS)
                    if sdf_buffer[i,j,k] <= 0:
                        points_buffer[index + 12] = (np.array([x,y,z]))/DIVISIOINS - EXTENTS/2
                    else:
                        points_buffer[index + 12] = np.array([100,100,100])
        pos3[:,0] = -points_buffer[:,1]
        pos3[:,1] = points_buffer[:,2]
        pos3[:,2] = -points_buffer[:,0]
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

    # control = vis.get_view_control()
    # control.set_constant_z_far(10)


    # initialize pointcloud instance.
    pcd = o3d.geometry.PointCloud()
    # add initial points, to ensure clip planes correct
    # points = np.random.rand(10, 3)

    points_buffer = np.zeros((SDF_EXTENTS*SDF_EXTENTS*SDF_EXTENTS + 12, 3))
    pos3 = np.zeros((SDF_EXTENTS*SDF_EXTENTS*SDF_EXTENTS + 12, 3))
    # Coordinate Frame
    #-----------------------------------------------
    points_buffer[0] = np.array([-15, -25, -25])
    points_buffer[1] = np.array([-25, -15, -25])
    points_buffer[2] = np.array([-25, -25, -15])
    points_buffer[3] = np.array([-25, -25, -25])
    #-----------------------------------------------
    points_buffer[4] = np.array([15.5, 15.5, 15.5])
    points_buffer[5] = np.array([15.5, 15.5, -15.5])
    points_buffer[6] = np.array([15.5, -15.5, -15.5])
    points_buffer[7] = np.array([-15.5, 15.5, -15.5])
    points_buffer[8] = np.array([-15.5, -15.5, 15.5])
    points_buffer[9] = np.array([-15.5, -15.5, -15.5])
    points_buffer[10] = np.array([15.5, -15.5, 15.5])
    points_buffer[11] = np.array([-15.5, 15.5, 15.5])
    # Swap for correct visualisation

    pos3[:,0] = -points_buffer[:,1]
    pos3[:,1] = points_buffer[:,2]
    pos3[:,2] = -points_buffer[:,0]

    # Coordinate Frame Colors
    color_buffer = np.zeros((SDF_EXTENTS*SDF_EXTENTS*SDF_EXTENTS + 12, 3))
    color_buffer[0] = np.array([1,0,0])
    color_buffer[1] = np.array([0,1,0])
    color_buffer[2] = np.array([0,0,1])
    # color_buffer[3] = np.array([1,5,0.5])
    color_buffer[4:12] = np.array([1,0.5,0.5])

    pcd.points = o3d.utility.Vector3dVector(pos3)
    pcd.colors = o3d.utility.Vector3dVector(color_buffer)

    points_update_thread = threading.Thread(target=update_points_buffer, args=(points_buffer,pos3,sdf_index,sdf_buffer,pcd))
    points_update_thread.start()
    # include it in the visualizer before non-blocking visualization.
    vis.add_geometry(pcd)

    # to add new points each dt secs.
    dt = 0.02
    # number of points that will be added
    n_new = 1000

    # run non-blocking visualization. 
    # To exit, press 'q' or click the 'x' of the window.
    previous_t = time.time()
    global keep_running
    while keep_running:
        if time.time() - previous_t > dt:
            print(f"Index: {sdf_index}")
            # update_points_buffer(points_buffer, pos3, sdf_index, sdf_buffer, pcd)
            # pcd.points = o3d.utility.Vector3dVector(points)
            # temp = np.array((points[:,0]; points[:,2], points[:,1]))
            # pcd.points = o3d.utility.Vector3dVector(points_buffer)
            
            vis.update_geometry(pcd)
            previous_t = time.time()

        keep_running = vis.poll_events()
        vis.update_renderer()
    vis.destroy_window()
    sdf_shm.close()
    sdf_index_shm.close()
