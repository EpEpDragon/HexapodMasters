#!/usr/bin/env python
import rospy

from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import message_filters

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
    def __init__(self, topic_rgb, topic_d, topic_pose):
        self.bridge = CvBridge()
        
        # Color sub
        rospy.Subscriber(topic_rgb, Image, self.color_callback)
        
        # Synchronise depth and pose callback
        d_sub = message_filters.Subscriber(topic_d, Image, self.depth_callback)
        pose_sub = message_filters.Subscriber(topic_pose,PoseStamped,self.pose_callback)
        message_filters.TimeSynchronizer([d_sub, pose_sub], self.sync_callback)

        self.rgb = 0
        self.d = 0
        self.rgb_ready = False
        self.d_ready = False
        self.d_stamp = 0
        self.pose = 0
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

    def sync_callback(self, data):
        print("Sync!")
        self.depth_callback(data)
        self.pose_callback(data)

    def depth_callback(self, data):
        try:
            self.d = self.bridge.imgmsg_to_cv2(data, data.encoding).astype(np.float32) / 10.0
        except CvBridgeError as e:
            print(e)
            return
        else:
            # Save Depth
            if not cv2.imwrite(depth_test_file+str(data.header.stamp)+'.jpeg', self.d):
                print("Save depth error")
            self.d = cv2.resize(self.d, (RES_X, RES_Y), interpolation=cv2.INTER_NEAREST)
            # self.depth_queue.append([data.header.stamp, cv2.resize(self.d, (RES_X, RES_Y), interpolation=cv2.INTER_NEAREST)])
            # self.d_stamp = data.header.stamp
            self.d_ready = True
            # rospy.loginfo({np.max(self.d)})
            # cv2.imshow('Depth', cm.jet(self.d / 10))
            # cv2.waitKey(1)

    def pose_callback(self, data):
        self.pose = data.pose
        # self.pose_queue.append([data.header.stamp, data.pose])
        # Write pose data
        with open(pose_file, 'a') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([data.header.stamp, data.pose.position.x, data.pose.position.y, data.pose.position.z,
                             data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
            csvfile.close()
            #print("Position:", data.pose.position.x, data.pose.position.y, data.pose.position.z,
            #      "Rotation:", data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)

def run():
    bridge = CvBridge()
    rospy.init_node('hexapod_heightmap_generate')

    rospy.loginfo("Initialiseing perception module...")
    perception = Perception(int(RES_Y*RES_X))

    if perception:
        rospy.loginfo("Initialised!")
    else:
        rospy.logerr("Initialisation failed!")
    

    pub_rgb = rospy.Publisher('rgb_data', Image, queue_size=10)
    pub_d = rospy.Publisher('d_data', Image, queue_size=10)
    pub_hmap = rospy.Publisher('hmap_data', Image, queue_size=10)
    
    rgbd_in = RGBDListener('/camera/color/image_raw', '/camera/aligned_depth_to_color/image_raw', 'orb_pose')

    rate = rospy.Rate(15)
    # Camera tilt angle
    angle = np.deg2rad(rospy.get_param("camera_pitch_offset"))
    while not rospy.is_shutdown():
        t = rospy.Time.now()

        if rgbd_in.rgb_ready:
            # Publish  downsampled rgb
            pass
            pub_rgb.publish(bridge.cv2_to_imgmsg(rgbd_in.rgb))
        if rgbd_in.d_ready:
            # Build heightmap
            perception.update(np.array([0,0,4]), np.array([0,0,0]), np.array([np.sin(angle)*1, np.sin(angle)*0, np.sin(angle)*0, np.cos(angle)]), np.array([1,0,0,0]), rgbd_in.d)
            
            # Save Hmap
            if not cv2.imwrite(hmap_test_file+str(rgbd_in.d_stamp)+'.jpeg', perception.hmap_buffer):
                print("Save hmap error")
            
            # Publish downsampled depth and heightmap
            pub_hmap.publish(bridge.cv2_to_imgmsg(perception.hmap_buffer))
            pub_d.publish(bridge.cv2_to_imgmsg(rgbd_in.d))
            # print(rospy.Time.now(), "push hmap")

        rate.sleep()
        td = (rospy.Time.now()-t)/1000000
        # print(td)


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
