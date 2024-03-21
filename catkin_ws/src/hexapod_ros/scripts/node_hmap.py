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

RES_X = int(212)
RES_Y = int(120)

class RGBDListener:
    def __init__(self, topic_rgb, topic_d):
        self.bridge = CvBridge()
        self.rgb_sub = rospy.Subscriber(topic_rgb, Image, self.color_callback)
        self.d_sub = rospy.Subscriber(topic_d, Image, self.depth_callback)
        self.rgb = 0
        self.d = 0
        self.rgb_ready = False
        self.d_ready = False
        


    def color_callback(self, data):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(data, data.encoding).astype(np.float32)
            self.rgb = cv2.resize(self.rgb, (RES_X, RES_Y))
            self.rgb_ready = True
            # cv2.imshow('Color', (self.rgb[:,:,::-1]).astype(np.uint8))
            # cv2.waitKey(1)
        
        except CvBridgeError as e:
            print(e)
            return


    def depth_callback(self, data):
        try:
            self.d = self.bridge.imgmsg_to_cv2(data, data.encoding).astype(np.float32) / 100.0
            self.d = cv2.resize(self.d, (RES_X, RES_Y), interpolation=cv2.INTER_NEAREST)
            self.d_ready = True
            # display = ((self.d).astype(np.float32))
            # rospy.loginfo({np.max(display)})
            # cv2.imshow('Depth', cm.jet(self.d / 10))
            # cv2.waitKey(1)
            
        except CvBridgeError as e:
            print(e)
            return


def _init_rgbd_display():
    cv2.namedWindow('Color', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Depth', cv2.WINDOW_NORMAL)
    cv2.namedWindow('HMap', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Color', 800, 600)
    cv2.resizeWindow('Depth', 800, 600)
    cv2.resizeWindow('HMap', 600, 600)

def _display_rgbd(rgbd_in):
    if rgbd_in.rgb_ready:
        cv2.imshow('Color', (rgbd_in.rgb[:,:,::-1]).astype(np.uint8))
        cv2.waitKey(1)

    if rgbd_in.d_ready:
        display = ((rgbd_in.d).astype(np.float32))
        # rospy.loginfo({np.max(display)})
        cv2.imshow('Depth', cm.jet(rgbd_in.d / 10))
        cv2.waitKey(1)


def run():
    rospy.init_node('hmap', anonymous=True)
    _init_rgbd_display()

    rospy.loginfo("Initialiseing perception module...")
    perception = Perception(int(RES_Y*RES_X))

    if perception:
        rospy.loginfo("Initialised!")
    else:
        rospy.logerr("Initialisation failed!")
    
    pub_hmap = rospy.Publisher('hmap_data', Image, queue_size=10)
    
    rgbd_in = RGBDListener('/camera/color/image_raw', '/camera/aligned_depth_to_color/image_raw')

    rate = rospy.Rate(15)
    angle = np.deg2rad(rospy.get_param("camera_pitch_offset"))
    # angle = np.deg2rad(20.0)
    while not rospy.is_shutdown():
        rate.sleep()
        # angle += np.deg2rad(0.1)
        print((np.rad2deg(angle)%360))
        if rgbd_in.d_ready:
            perception.update(np.array([0,0,4]), np.array([0,0,0]), np.array([np.sin(angle)*1, np.sin(angle)*0, np.sin(angle)*0, np.cos(angle)]), np.array([1,0,0,0]), rgbd_in.d)
        
        _display_rgbd(rgbd_in)
        perception._display_heightmap()
        


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass