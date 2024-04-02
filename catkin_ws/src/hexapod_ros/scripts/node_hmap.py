#!/usr/bin/env python

import rospy
from hexapod_ros.msg import DMatrixFlat, RGBMatrixFlat


from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from perception import Perception

import cv2
import matplotlib.pyplot as plt

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
            self.rgb = self.bridge.imgmsg_to_cv2(data, data.encoding).astype(np.uint8)
            self.rgb = cv2.resize(self.rgb, (RES_X, RES_Y), interpolation=cv2.INTER_NEAREST)
            self.rgb_ready = True
        
        except CvBridgeError as e:
            print(e)
            return


    def depth_callback(self, data):
        try:
            self.d = self.bridge.imgmsg_to_cv2(data, data.encoding).astype(np.float32) / 10.0
            self.d = cv2.resize(self.d, (RES_X, RES_Y), interpolation=cv2.INTER_NEAREST)
            self.d_ready = True
            
        except CvBridgeError as e:
            print(e)
            return

    
def run():
    bridge = CvBridge()
    rospy.init_node('hmap', anonymous=True)

    rospy.loginfo("Initialiseing perception module...")
    perception = Perception(int(RES_Y*RES_X))

    if perception:
        rospy.loginfo("Initialised!")
    else:
        rospy.logerr("Initialisation failed!")
    

    pub_rgb = rospy.Publisher('rgb_data', Image, queue_size=10)
    pub_d = rospy.Publisher('d_data', Image, queue_size=10)
    pub_hmap = rospy.Publisher('hmap_data', Image, queue_size=10)
    
    
    rgbd_in = RGBDListener('/camera/color/image_raw', '/camera/aligned_depth_to_color/image_raw')
          
    rospy.loginfo("Feed found!")

    rate = rospy.Rate(15)
    angle = np.deg2rad(rospy.get_param("camera_pitch_offset"))
    while not rospy.is_shutdown():
        t = rospy.Time.now()
        if rgbd_in.rgb_ready:
            pub_rgb.publish(bridge.cv2_to_imgmsg(rgbd_in.rgb))
        if rgbd_in.d_ready:
            perception.update(np.array([0,0,4]), np.array([0,0,0]), np.array([np.sin(angle)*1, np.sin(angle)*0, np.sin(angle)*0, np.cos(angle)]), np.array([1,0,0,0]), rgbd_in.d)
            pub_d.publish(bridge.cv2_to_imgmsg(rgbd_in.d))
            pub_hmap.publish(bridge.cv2_to_imgmsg(perception.hmap_buffer))
            print(str(rospy.Time.now())+"push hmap")
        rate.sleep()
        td = (rospy.Time.now()-t)/1000000
        # print(td)


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass