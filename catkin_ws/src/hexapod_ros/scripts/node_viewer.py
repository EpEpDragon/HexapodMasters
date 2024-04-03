#!/usr/bin/env python

import rospy
import pygame
import math

from sensor_msgs.msg import Image
from hexapod_ros.msg import HexapodCommands

from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import cv2
# import matplotlib.pyplot as plt
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui


RES_X = int(212)
RES_Y = int(120)

# Get and store data from hexapod
class DataListner:
    def __init__(self, topic_rgb, topic_d, topic_hmap):
        self.bridge = CvBridge()
        rospy.Subscriber(topic_rgb, Image, self.color_callback)
        rospy.Subscriber(topic_d, Image, self.depth_callback)
        rospy.Subscriber(topic_hmap, Image, self.hmap_callback)

        self.rgb = 0
        self.d = 0
        self.hmap = 0
        self.rgb_ready = False
        self.d_ready = False
        self.hmap_ready = False
        
    def color_callback(self, data):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.rgb = cv2.resize(self.rgb, (RES_X, RES_Y), interpolation=cv2.INTER_NEAREST)
            self.rgb_ready = True
        
        except CvBridgeError as e:
            print(e)
            return

    def depth_callback(self, data):
        try:
            self.d = self.bridge.imgmsg_to_cv2(data, data.encoding).astype(np.float32)
            self.d = cv2.resize(self.d, (RES_X, RES_Y), interpolation=cv2.INTER_NEAREST)
            self.d_ready = True
            
        except CvBridgeError as e:
            print(e)
            return
    
    def hmap_callback(self, data):
        try:
            self.hmap = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.hmap_ready = True

        except CvBridgeError as e:
            print(e)
            return


def relative_to_absolute(pos, size):
    return (pos[0]*size[0], pos[1]*size[1])
def absolute_to_relative(pos, size):
    return (float(pos[0])/size[0], float(pos[1])/size[1])
def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm
class ControlInterface():
    screen_color = (60,25,60)
    ##### UI ELements Def ####
    class Line(object):
        def __init__(self, start, end, color, thicc):
            self.start = start
            self.end = end
            self.color = color
            self.thicc = thicc
        
        def draw(self, screen):
            start = relative_to_absolute(self.start, screen.get_size())
            end = relative_to_absolute(self.end,screen.get_size())
            return pygame.draw.line(screen, self.color, start, end, self.thicc)
    
    # A line that points at the mouse cursor
    class PointLine(Line):
        def __init__(self, start, length, color, thicc):
            super(ControlInterface.PointLine, self).__init__(start, (0,0), color, thicc)
            self.length = length

            # For clear rect
            self.left_top = tuple(np.array(start)-np.array(length))
            self.width_height = (length*2+0.01, length*2+0.01)
            self.u_dir = [0.0, 0.0]
        
        
        def set_rect(self, start, length):
            self.rect = pygame.Rect(tuple(np.array(start)-np.array(length)), (length*2+2, length*2+2))
        

        def draw(self, screen):
            mouse_pos = np.array(pygame.mouse.get_pos())
            start = relative_to_absolute(self.start, screen.get_size())
            diff = mouse_pos - start
            
            screen_size = screen.get_size()
            length = 0
            # Scale length based on largest screen dim
            if screen_size[0] > screen_size[1]:
                length = self.length * screen_size[0]
            else:
                length = self.length * screen_size[1]
                
            dists = (screen_size[0]-start[0], screen_size[1]-start[1])

            # Clamp line dist if overflow
            if (start[0]-length < 0):
                length = start[0]-10
            if (start[1]-length < 0):
                length = min(start[1]-10, length)
            if start[0] + length > screen_size[0]:
                length = min(screen_size[0] - start[0]-10, length)
            if start[1] + length > screen_size[1]:
                length = min(screen_size[1] - start[1]-10, length)

            self.u_dir = normalize(diff)
            end = tuple(start + self.u_dir * length)
            
            self.angle = math.tan(self.u_dir[0]/1.0)

            self.set_rect(start, length)
            pygame.display.update(screen.fill(ControlInterface.screen_color, self.rect))
            return pygame.draw.line(screen, self.color, start, end, self.thicc)

    class Box:
        def __init__(self, start, end, color):
            self.start = start
            self.end = end
            self.color = color
        
        def draw(self, screen):
            start = relative_to_absolute(self.start, screen.get_size())
            end = relative_to_absolute(self.end, screen.get_size())
            return pygame.draw.rect(screen, self.color, pygame.Rect(start,end))

    class Button:
        def __init__(self):
            pass
        def draw(self, screen):
            pass
    
    class Text:
        def __init__(self, text, font, color):
            pass
        def draw(self, screen):
            pass
    
    class Image:
        def __init__(self, start, max_size, data = np.random.rand(100, 200, 3)):
            self.start = start
            self.surface = pygame.surfarray.make_surface(data)
            self.max_size = max_size
            self.set_data(data)
        
        def set_data(self, data):
            print(data.shape)
            self.data = np.transpose(data, (1,0,2))
            self.ratio = float(data.shape[0])/data.shape[1]
            print(self.ratio)

        def draw(self, screen):
            # Resize image to fit insize max_size, maintains aspect ratio
            size = (self.max_size[0])*screen.get_size()[0]
            resize = (size, size*self.ratio)
            
            print(resize)
            print("max " + str(self.max_size[1]*screen.get_size()[1]))
            if resize[1] > self.max_size[1]*screen.get_size()[1]:
                print("dasd")
                size = (self.max_size[1])*screen.get_size()[1]
                resize = (size/self.ratio, size)                
            
            # Invert because stupid
            resize = (int(resize[1]), int(resize[0]))
            
            # print(resize)
            data = cv2.resize(self.data, resize, interpolation=cv2.INTER_NEAREST)
            self.surface = pygame.surfarray.make_surface(data)
            return screen.blit(self.surface, relative_to_absolute(self.start, screen.get_size()))
    ##########################

    def __init__(self):
        pygame.init()
        self.running = True

        # Make window resiasable
        self.screen = pygame.display.set_mode((512,512), pygame.RESIZABLE)
        
        self.smallfont = pygame.font.SysFont('Corbel',35) 

        # Elements in GUI to update
        self.elements = []
        self.redraw_display()
    
    def redraw_display(self):
        self.screen.fill(ControlInterface.screen_color)
        pygame.display.flip()

    def add_element(self, element):
        self.elements.append(element)
        return element
        

    def check_input(self):
        for ev in pygame.event.get():
            
            if ev.type == pygame.VIDEORESIZE:
                self.redraw_display()
            if ev.type == pygame.QUIT:
                self.running = False
                pygame.quit()
            
            if ev.type == pygame.MOUSEBUTTONDOWN:
                print("test")

    def update(self):
        width = self.screen.get_width()
        height = self.screen.get_height()

        for e in self.elements:
            pygame.display.update(e.draw(self.screen))
        
        self.check_input()


# def _init_rgbd_display():
    # app = pg.mkQApp()
    # win = QtGui.QMainWindow()

    # container widget with a layout to add QWidgets to
    # cw = QtGui.QWidget()
    # win.setCentralWidget(cw)
    # layout = QtGui.QVBoxLayout()
    # cw.setLayout(layout)

    # im1 = pg.image()
    # im1.setImage(np.random.rand(10, 10))
    # # layout.addWidget(im1)

    # im2 = pg.image()
    # im2.setImage(np.random.rand(100, 100))
    # layout.addWidget(im2)

    # win.show()
    # app.exec_()
    
    # Matplot
    # plt.ion()
    # f, axarr = plt.subplots(3,1) 
    # img_rgb = axarr[0].imshow(np.zeros((RES_Y,RES_X,3)).astype(np.uint8))
    # img_d = axarr[1].imshow(np.zeros((RES_Y,RES_X)), cmap="jet", vmin=0, vmax=50, interpolation="nearest")
    # img_hmap = axarr[2].imshow(np.zeros((192,192)), cmap="jet", vmin=0, vmax=25, interpolation="nearest")

    # plt.colorbar(img_d, ax=axarr[1])
    # plt.colorbar(img_hmap, ax=axarr[2])
    # plt.subplots_adjust(left=0.04, bottom=0.02, right=1, top=0.99, wspace=0.2, hspace=0.08)
    
    # return img_rgb, img_d, img_hmap


def run():
    rospy.init_node('hexapod_data_viewer')
    
    command_pub = rospy.Publisher('hexapod_command_data', HexapodCommands, queue_size=10)
    command_msg = HexapodCommands([0,0],0,0)

    data_in = DataListner('rgb_data', 'd_data', 'hmap_data')
    # img_rgb, img_d, img_hmap = _init_rgbd_display()


    ######## Interface Elements ##########
    control_interface = ControlInterface()
    # control_interface.add_element(ControlInterface.Box(start=(0.65, 0.05), end=(0.95, 0.4),color=(255,255,255)))
    dir_pick = control_interface.add_element(ControlInterface.PointLine(start=(0.8,0.25), length=0.15, color=(255,0,0), thicc=2))
    control_interface.add_element(ControlInterface.Line(start=(0.6,0.05), end=(0.6,0.95), color=(255,255,255), thicc=2))
    img_rgb = control_interface.add_element(ControlInterface.Image(start=(0.01,0.01), max_size=(0.5,0.5)))
    # img_d = control_interface.add_element(ControlInterface.Image(start=(0.05,0.25), max_size=(0.1,0.2)))
    # img_hmap = control_interface.add_element(ControlInterface.Image(start=(0.05,0.5), max_size=(0.1,0.2)))
    #####################################

    rate = rospy.Rate(30)
    while not rospy.is_shutdown() and control_interface.running:
        t = rospy.Time.now()
        
        if data_in.rgb_ready:
            img_rgb.set_data(data_in.rgb)
        # if data_in.d_ready:
        #     img_d.set_data(data_in.d)
        # if data_in.hmap_ready:
        #     img_hmap.set_data(data_in.hmap*10)

        control_interface.update()
        
        ############## Push Hexapod Commands ##############
        command_msg.walk_dir = dir_pick.u_dir
        command_pub.publish(command_msg)
        ###################################################

        rate.sleep()
        td = (rospy.Time.now()-t)/1000000
        print(str(td)+" ms")


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass