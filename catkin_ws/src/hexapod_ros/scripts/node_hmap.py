#!/usr/bin/env python
import rospy

from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import message_filters
from roboMath import rotate
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from perception import Perception
from collections import deque

import cv2
import csv
import os

RES_X = int(640/2)
RES_Y = int(480/2)

test_file = os.path.join(rospy.get_param("pkg_root"),'..','..','..','Results')
color_test_file = os.path.join(test_file,'Color','')
depth_test_file = os.path.join(test_file,'Depth','')
hmap_test_file = os.path.join(test_file,'Hmap','')
pose_file = os.path.join(test_file,'PoseData.csv')

# Get data from RGBD camera and store for use
class RGBDListener:
    def __init__(self, topic_rgb, topic_d, topic_pose, perception):
        self.bridge = CvBridge()
        
        # Color sub
        rospy.Subscriber(topic_rgb, Image, self.color_callback)
        
        # Synchronise depth and pose callback
        d_sub = message_filters.Subscriber(topic_d, Image)
        pose_sub = message_filters.Subscriber(topic_pose, PoseStamped)
        ts = message_filters.TimeSynchronizer([d_sub, pose_sub], 100)
        ts.registerCallback(self.sync_callback)
		

        self.rgb = 0
        self.d = 0
        self.rgb_ready = False
        self.d_ready = False
        self.building_hmap = False
        self.d_stamp = 0
        self.position = 0
        self.orb_tilt_quat = np.array([0, np.sin(np.deg2rad(17)), 0, np.cos(np.deg2rad(17))])
        # self.depth_queue = deque()
        # self.pose_queue = deque()
        open(pose_file,'w').close()
        
    def color_callback(self, data):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(data, data.encoding).astype(np.uint8)
        except CvBridgeError as e:
            print(e)
            return
        else:
            # Save Color
            if not cv2.imwrite(color_test_file+str(data.header.stamp)+'.jpeg', self.rgb):
                print("Save color error")
            self.rgb = cv2.resize(self.rgb, (RES_X, RES_Y), interpolation=cv2.INTER_NEAREST)
            self.rgb_ready = True
            # cv2.imshow('Color', (self.rgb[:,:,::-1]).astype(np.uint8))
            # cv2.waitKey(1)

    def sync_callback(self, depth, pose):
        while self.building_hmap:
            pass
        #print("Sync!")
        self.depth_callback(depth)
        self.pose_callback(pose)
        self.building_hmap = True

    def depth_callback(self, data):
        try:
            self.d = self.bridge.imgmsg_to_cv2(data, data.encoding).astype(np.float32) / 10.0
        except CvBridgeError as e:
            print(e)
            return
        else:
            # Save Depth
            np.save(depth_test_file+str(data.header.stamp)+'.npy', self.d):
            self.d = cv2.resize(self.d, (RES_X, RES_Y), interpolation=cv2.INTER_NEAREST)
            # self.depth_queue.append([data.header.stamp, cv2.resize(self.d, (RES_X, RES_Y), interpolation=cv2.INTER_NEAREST)])
            self.d_stamp = data.header.stamp
            self.d_ready = True
            # rospy.loginfo({np.max(self.d)})
            # cv2.imshow('Depth', cm.jet(self.d / 10))
            # cv2.waitKey(1)

    def pose_callback(self, data):
        self.position = rotate(self.orb_tilt_quat, np.array([data.pose.position.z, -data.pose.position.x, -data.pose.position.y]))*10
        # self.pose_queue.append([data.header.stamp, data.pose])
        # Write pose data
        with open(pose_file, 'a') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([data.header.stamp, self.position[0], self.position[1], self.position[2],
                             data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
            csvfile.close()
            #print("Position:", data.pose.position.x, data.pose.position.y, data.pose.position.z,
            #      "Rotation:", data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)

def run():
    bridge = CvBridge()
    rospy.init_node('hexapod_heightmap_generate')
    perception = Perception(int(RES_Y*RES_X))
    angle = np.deg2rad(rospy.get_param("camera_pitch_offset"))
    cam_tilt_quat = np.array([np.sin(angle)*1, np.sin(angle)*0, np.sin(angle)*0, np.cos(angle)])
    rospy.loginfo("Initialiseing perception module...")

    if perception:
        rospy.loginfo("Initialised!")
    else:
        rospy.logerr("Initialisation failed!")
    

    pub_rgb = rospy.Publisher('rgb_data', Image, queue_size=10)
    pub_d = rospy.Publisher('d_data', Image, queue_size=10)
    pub_hmap = rospy.Publisher('hmap_data', Image, queue_size=10)
    
    rgbd_in = RGBDListener('/camera/color/image_raw', '/camera/aligned_depth_to_color/image_raw', 'orb_pose', perception)

    rate = rospy.Rate(15)
    # Camera tilt angle
    while not rospy.is_shutdown():
        t = rospy.Time.now()
        if rgbd_in.rgb_ready:
            # Publish  downsampled rgb
            pub_rgb.publish(bridge.cv2_to_imgmsg(rgbd_in.rgb))
        if rgbd_in.d_ready:
            # Publish downsampled depth and heightmap
            pub_d.publish(bridge.cv2_to_imgmsg(rgbd_in.d))
            # print(rospy.Time.now(), "push hmap")
        if rgbd_in.building_hmap:
            # Build heightmap
            print(rgbd_in.position)
            perception.update(rgbd_in.position[[1,0,2]]*np.array([-1,1,1])+np.array([0,0,4]), cam_tilt_quat, np.array([1,0,0,0]), rgbd_in.d)
            pub_hmap.publish(bridge.cv2_to_imgmsg(perception.hmap_buffer))
                
            # Save Hmap
            np.save(hmap_test_file+str(rgbd_in.d_stamp)+'.npy', perception.hmap_buffer):            
            rgbd_in.building_hmap = False
        rate.sleep()
        td = (rospy.Time.now()-t)/1000000
        # print(td)

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
