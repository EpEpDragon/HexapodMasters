#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from perception import Perception
import cv2

RES_X = int(160)
RES_Y = int(90)

class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, Image, self.imageDepthCallback)

    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            cv_image = cv_image[:,:,::-1]
            # rospy.loginfo("Image! Shape:  Width: %s Height %s" % (cv_image.shape,data.width, data.height))
            # rospy.loginfo(data.encoding)
            cv2.imshow('SDF Slice', (cv_image).astype(np.uint8))
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
    
    pub_hmap = rospy.Publisher('hmap_data', String, queue_size=10)
    
    sub_image = ImageListener('/camera/aligned_depth_to_color/image_raw')

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        pub_hmap.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass