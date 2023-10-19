import cv2
import numpy as np
import open3d as o3d
from perception import Perception, SDF_EXTENTS, EXTENTS, DIVISIOINS
import glfw

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
    p_F = np.logical_and(np.logical_and(p_X[:][:] != 0, p_Y[:][:] != 0), p_Z[:][:] != 0)
    return np.dstack((p_X, p_Y, p_Z))[p_F]

def show_points(p):
    vis = o3d.visualization.Visualizer()
    vis.create_window(height=480, width=640)
    pcd = o3d.geometry.PointCloud()
    if p.shape[1] != 3:
        p.resize((p.shape[0]*p.shape[1], 3))
    pos3 = np.ndarray((p.shape[0]+8,p.shape[1]))
    # pos3[0:-8,0] = p[:,0]
    # pos3[0:-8,1] = p[:,2]
    # pos3[0:-8,2] = -p[:,1]
    pos3[0:-8,0] = p[:,0]
    pos3[0:-8,1] = p[:,1]
    pos3[0:-8,2] = p[:,2]
    pos3[-1] = np.array([15, 15, 15])
    pos3[-2] = np.array([15, 15, -15])
    pos3[-3] = np.array([15, -15, -15])
    pos3[-4] = np.array([-15, 15, -15])
    pos3[-5] = np.array([-15, -15, 15])
    pos3[-6] = np.array([-15, -15, -15])
    pos3[-7] = np.array([15, -15, 15])
    pos3[-8] = np.array([-15, 15, 15])

    pcd.points = o3d.utility.Vector3dVector(pos3)
    vis.add_geometry(pcd)
    # vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(15))
    keep_running = True

    while keep_running:
        # pass
        keep_running = vis.poll_events()
    vis.destroy_window()

def sdf_to_points(sdf):
    index = np.where(sdf <= 0)
    # index_flat = index[0][:] + index[1][:]*SDF_EXTENTS + index[2][:]*SDF_EXTENTS*SDF_EXTENTS
    index[0][:] = (index[0][:])%SDF_EXTENTS
    index[1][:] = (index[1][:])%SDF_EXTENTS
    index[2][:] = (index[2][:])%SDF_EXTENTS
    # sdf_points[index_flat] = np.transpose(index)/DIVISIOINS - EXTENTS/2
    return np.transpose(index)/DIVISIOINS - EXTENTS/2


if __name__ == '__main__':

    DISPLAY_WIDTH = 900
    DISPLAY_HEIGHT = 900
    glfw.init()
    glfw.window_hint(glfw.VISIBLE, False)
    window = glfw.create_window(DISPLAY_WIDTH, DISPLAY_HEIGHT, "hidden window", None, None)
    glfw.make_context_current(window)

    perception = Perception()
    perception.init_shader(int(1280*720))
    
    img = (cv2.imread('depth_img.png')/255)[:,:,0].astype(np.float32)
    # cv2.imshow('img', img)
    # cv2.waitKey(0)
    depth = img*10
    p = points_from_depth(depth)
    # show_points(p)
    perception.update_new(np.array([0, 0, 0]),np.array([1,0,0,0]),depth.reshape(1280*720))
    # perception.update(np.array([0,0,0]),np.array([1,0,0,0]),p)
    sdf = perception.sdf_buffer
    sdf_points = np.ones((sdf.shape[0]*sdf.shape[1]*sdf.shape[2],3))
    sdf_points = sdf_to_points(sdf)

    show_points(sdf_points)