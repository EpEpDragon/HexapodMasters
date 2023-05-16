import cv2
import numpy as np
import open3d as o3d
from perception import Perception, SDF_EXTENTS, EXTENTS, DIVISIOINS

def linearize_depth(depth, znear, zfar):
    zlinear = (znear * zfar) / (zfar + depth * (znear - zfar))
    return zlinear

def points_from_depth(depth_linear):
    cam_K = np.array([[623.533, 0,          639.5],
                     [0,        623.533,    359.5],
                     [0,        0,          1]])
    cam_x_over_z, cam_y_over_z = cv2.initInverseRectificationMap(
                cam_K, # Intrinsics
                None, # Distortion (0 for GPU rendered images)
                np.eye(3), # Rectification
                np.eye(3), # Unity rectification intrinsics (we want direction vector)
                (1280, 720), # Test all pixels in physical sensor
                m1type=cv2.CV_32FC1)
    # depth_linear = linearize_depth(depth, znear=0.4, zfar=80)
    depth_linear[depth_linear > 10 - 0.0005] = 0
    # cv2.imshow('Camera', cv2.cvtColor(depth_linear / np.max(depth_linear), cv2.COLOR_RGB2BGR))
    p_X = cam_x_over_z * depth_linear
    p_Y = cam_y_over_z * depth_linear
    p_Z = depth_linear
    return np.dstack((p_X, p_Y, p_Z))

def show_points(p):  
    vis = o3d.visualization.Visualizer()
    vis.create_window(height=480, width=640)
    pcd = o3d.geometry.PointCloud()
    if p.shape[1] != 3:
        p.resize((p.shape[0]*p.shape[1], 3))
    # pos3 = np.array(p)
    # pos3[:,0] = p[:,0]
    # pos3[:,1] = p[:,2]
    # pos3[:,2] = -p[:,1]

    pcd.points = o3d.utility.Vector3dVector(p)
    vis.add_geometry(pcd)
    keep_running = True

    while keep_running:
        # pass
        keep_running = vis.poll_events()
    vis.destroy_window()

def sdf_to_points(sdf, sdf_points):
    for i in range(sdf.shape[0]):
        for j in range(sdf.shape[1]):
            for k in range(sdf.shape[2]):
                # x = (i)%SDF_EXTENTS 
                # y = (j)%SDF_EXTENTS
                # z = (k)%SDF_EXTENTS
                index = int(i + j*SDF_EXTENTS + k*SDF_EXTENTS*SDF_EXTENTS)
                if sdf[i,j,k] <= 0:
                    sdf_points[index] = (np.array([i,j,k]))/DIVISIOINS - EXTENTS/2
                else:
                    sdf_points[index] = np.array([0,0,0])

if __name__ == '__main__':
    perception = Perception()
    
    img = (cv2.imread('/home/epep/Documents/HexapodMasters/depth_img.png')/255)[:,:,0]
    cv2.imshow('img', img)
    cv2.waitKey(0)
    depth = img*10
    p = points_from_depth(depth)
    show_points(p)
    # perception.update(np.array([0,0,0]),p)
    sdf = perception.sdf_buffer
    sdf_points = np.ones((sdf.shape[0]*sdf.shape[1]*sdf.shape[2],3))
    sdf_to_points(sdf, sdf_points)

    show_points(sdf_points)


