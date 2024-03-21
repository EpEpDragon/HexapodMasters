#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from perception import Perception
import cv2
import matplotlib.cm as cm

RES_X = int(160)
RES_Y = int(90)

class RGBDListener:
    def __init__(self, topic_rgb, topic_d):
        self.bridge = CvBridge()
        self.rgb_sub = rospy.Subscriber(topic_rgb, Image, self.color_callback)
        self.d_sub = rospy.Subscriber(topic_d, Image, self.depth_callback)
        self.rgb = np.zeros(1)
        self.d = np.zeros(1)
        cv2.namedWindow('Color', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Depth', cv2.WINDOW_NORMAL)
        cv2.namedWindow('HMap', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Color', 800, 600)
        cv2.resizeWindow('Depth', 800, 600)
        cv2.resizeWindow('HMap', 600, 600)



    def color_callback(self, data):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(data, data.encoding).astype(np.float32)
            self.rgb = cv2.resize(self.rgb, (RES_X, RES_Y))
            cv2.imshow('Color', (self.rgb[:,:,::-1]).astype(np.uint8))
            cv2.waitKey(1)
        
        except CvBridgeError as e:
            print(e)
            return


    def depth_callback(self, data):
        try:
            self.d = self.bridge.imgmsg_to_cv2(data, data.encoding).astype(np.float32) / 100.0
            self.d = cv2.resize(self.d, (RES_X, RES_Y))
            # display = ((self.d).astype(np.float32))
            # rospy.loginfo({np.max(display)})

            cv2.imshow('Depth', cm.jet(self.d / 10))
            cv2.waitKey(1)
            
        except CvBridgeError as e:
            print(e)
            return


def run():
    rospy.init_node('hmap', anonymous=True)
    rospy.loginfo("Initialiseing perception module...")
    perception = Perception(int(RES_Y*RES_X))
 
    if perception:
        rospy.loginfo("Initialised!")
    else:
        rospy.logerr("Initialisation failed!")
    
    pub_hmap = rospy.Publisher('hmap_data', Image, queue_size=10)
    
    sub_rgbd = RGBDListener('/camera/color/image_raw', '/camera/aligned_depth_to_color/image_raw')

    rate = rospy.Rate(15)
    # time = 0.0
    while not rospy.is_shutdown():
        rate.sleep()
        # time += 1.0/15.0
        # rospy.loginfo(rospy.get_param("camera_pitch_offset")%360)
        # perception.update(np.array([0,0,0]), np.array([0,0,0]), np.concatenate((np.array([np.cos(np.deg2rad(time))]), np.sin(np.deg2rad(time))*np.array([0,0,1]))), np.array([1,0,0,0]), sub_rgbd.d)
        angle = np.deg2rad(rospy.get_param("camera_pitch_offset"))
        perception.update(np.array([0,0,4.4]), np.array([0,0,0]), np.array([np.sin(angle)*1, np.sin(angle)*0, np.sin(angle)*0, np.cos(angle)]), np.array([1,0,0,0]), sub_rgbd.d)

        perception._display_heightmap()
        


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass