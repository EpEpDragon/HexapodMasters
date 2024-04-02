#!/usr/bin/env python

import rospy

from hexapod_ros.msg import RGBMatrixFlat, DMatrixFlat

# from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from perception import Perception

import cv2
import matplotlib.pyplot as plt

RES_X = int(212)
RES_Y = int(120)

class DataListner:
    def __init__(self, topic_hmap):
        self.bridge = CvBridge()
        rospy.Subscriber(topic_hmap, Image, self.hmap_callback)

        self.hmap = 0
        self.hmap_ready = False
        

    def hmap_callback(self, data):
        try:
            print("got hmap data")
            self.hmap = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.hmap_ready = True
            # cv2.imshow('Color', (self.rgb[:,:,::-1]).astype(np.uint8))
            # cv2.waitKey(1)
        
        except CvBridgeError as e:
            print(e)
            return


    def depth_callback(self, data):
        try:
            self.d = self.bridge.imgmsg_to_cv2(data, data.encoding).astype(np.float32) / 10.0
            self.d = cv2.resize(self.d, (RES_X, RES_Y), interpolation=cv2.INTER_NEAREST)
            self.d_ready = True
            # rospy.loginfo({np.max(self.d)})
            # cv2.imshow('Depth', cm.jet(self.d / 10))
            # cv2.waitKey(1)
            
        except CvBridgeError as e:
            print(e)
            return



def _init_rgbd_display():
    cv2.namedWindow('Color', cv2.WINDOW_NORMAL)
    cv2.namedWindow('HMap', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Color', 800, 600)
    cv2.resizeWindow('HMap', 600, 600)
    
    # Matplot
    plt.ion()
    f, axarr = plt.subplots(3,1) 
    img_rgb = axarr[0].imshow(np.zeros((RES_Y,RES_X,3)).astype(np.uint8))
    img_d = axarr[1].imshow(np.zeros((RES_Y,RES_X)), cmap="jet", vmin=0, vmax=50, interpolation="nearest")
    img_hmap = axarr[2].imshow(np.zeros((192,192)), cmap="jet", vmin=0, vmax=25, interpolation="nearest")

    plt.colorbar(img_d, ax=axarr[1])
    plt.colorbar(img_hmap, ax=axarr[2])
    plt.subplots_adjust(left=0.04, bottom=0.02, right=1, top=0.99, wspace=0.2, hspace=0.08)
  
    return img_rgb, img_d, img_hmap


def run():
    rospy.init_node('hexapod_data_viewer', anonymous=True)
    
    data_in = DataListner('hmap_data')
    img_rgb, img_d, img_hmap = _init_rgbd_display()
    
    rospy.loginfo("Feed found!")
    # plt.show()


    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        t = rospy.Time.now()
        
        if data_in.hmap_ready:
            img_hmap.set_data(data_in.hmap*10)
            print("diaplay")
            # print(data_in.hmap_ready)

        plt.pause(0.0001)

        rate.sleep()
        td = (rospy.Time.now()-t)/1000000
        # print(td)
        # perception._display_heightmap()
        


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass