#!/usr/bin/env python

import rospy
import pygame
import math

from sensor_msgs.msg import Image
from hexapod_ros.msg import HexapodCommands

from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import cv2
import matplotlib
import matplotlib.cm as cm
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

def to_u8(im):
    im = (255 * (im / im.max())).astype(np.uint8)
    return im
def gray(im):
    im = 255 * (im / im.max())
    w, h = im.shape
    ret = np.empty((w, h, 3), dtype=np.uint8)
    ret[:, :, 2] = ret[:, :, 1] = ret[:, :, 0] = im
    return ret
def pointInRect(point,rect):
    x1, y1, w, h = rect
    x2, y2 = x1+w, y1+h
    x, y = point
    if (x1 < x and x < x2):
        if (y1 < y and y < y2):
            return True
    return False

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
            self.u_dir = np.array([0.0, 0.0])
        
        
        def set_rect(self, start, length):
            self.rect = pygame.Rect(tuple(np.array(start)-np.array(length)), (length*2+2, length*2+2))
        
        def set_dir(self, screen):
            if pygame.mouse.get_pressed()[0]:
                mouse_pos = np.array(pygame.mouse.get_pos())
                left_top = relative_to_absolute(self.left_top, screen.get_size())
                width_height = relative_to_absolute(self.width_height, screen.get_size())
                if ((left_top[0] < mouse_pos[0] and mouse_pos[0] < left_top[0]+width_height[0]) and
                    (left_top[1] < mouse_pos[1] and mouse_pos[1] < left_top[1]+width_height[1])):
                        self.u_dir = normalize(mouse_pos - np.array(relative_to_absolute(self.start, screen.get_size())))

        def draw(self, screen):
            self.set_dir(screen)
            start = relative_to_absolute(self.start, screen.get_size())
            # diff = mouse_pos - start
            
            screen_size = screen.get_size()
            length = 0
            # Scale length based on largest screen dim
            if screen_size[0] > screen_size[1]:
                length = self.length * screen_size[0]
            else:
                length = self.length * screen_size[1]
                
            # dists = (screen_size[0]-start[0], screen_size[1]-start[1])

            # Clamp line dist if overflow
            if (start[0]-length < 0):
                length = start[0]-10
            if (start[1]-length < 0):
                length = min(start[1]-10, length)
            if start[0] + length > screen_size[0]:
                length = min(screen_size[0] - start[0]-10, length)
            if start[1] + length > screen_size[1]:
                length = min(screen_size[1] - start[1]-10, length)

            # self.u_dir = normalize(diff)
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
        def __init__(self, start=(0,0), prefix="", font=pygame.font.SysFont, color=(255,255,255)):
            self.start = start
            self.prefix = prefix
            self.text = prefix
            self.color = color
            self.font = font
            self.rect_prev = pygame.Rect((0,0),(0,0))
        
        def set_text(self, text):
            self.text = self.prefix + text
        
        def draw(self, screen):
            text_surface = self.font.render(self.text, True, self.color)
            screen.fill(ControlInterface.screen_color, self.rect_prev)
            self.rect_prev = screen.blit(text_surface, relative_to_absolute(self.start, screen.get_size()))
            return self.rect_prev
    
    class Image:
        def __init__(self, start, max_size, data = np.random.rand(100, 200), colormap=None, vmin=0, vmax=1, text_box=None):
            self.start = start
            self.surface = pygame.surfarray.make_surface(data)
            self.max_size = max_size
            self.rect = pygame.Rect((0,0),(0,0))
            self.set_data(data)
            self.text_box = text_box
            
            self.vmax = vmax
            self.vmin = vmin
            if colormap != None:
                self.cm = cm.get_cmap(colormap)
            else:
                self.cm = None

        def norm_to_range(self):
            return ((self.data-self.vmin)/(self.vmax-self.vmin)).clip(0, 1.0)

        def set_data(self, data):
            # Transpose because data in is as (y,x)
            if len(data.shape) == 2:
                self.data = np.transpose(data)
            else: 
                self.data = np.transpose(data, (1,0,2))

            self.ratio = float(data.shape[0])/data.shape[1]

        def get_value(self, pos):
            x1, y1, w, h = self.rect
            U = float(pos[0]-x1)/w
            V = float(pos[1]-y1)/h
            return self.data[int(U*self.data.shape[0]), int(V*self.data.shape[1])]


        def update_textbox(self, screen):
            self.text_box.start = absolute_to_relative((self.rect.x, self.rect.y + self.rect.h), screen.get_size())
            mouse_pos = pygame.mouse.get_pos()
            if pointInRect(mouse_pos, self.rect):
                self.text_box.set_text(str(self.get_value(mouse_pos)))

        def draw(self, screen):
            if self.text_box != None:
                self.update_textbox(screen)
            # Resize image to fit insize max_size, maintains aspect ratio
            size = (self.max_size[0])*screen.get_size()[0]
            resize = ( size, size*self.ratio )
            
            if resize[1] > self.max_size[1]*screen.get_size()[1]:
                size = (self.max_size[1])*screen.get_size()[1]
                resize = (size/self.ratio, size)

            # Flip because CV2 does (y,x)
            resize = (int(resize[1]), int(resize[0]))
            
            data = self.data
            if self.cm != None:
                data = self.norm_to_range()
                data = self.cm(data)[:,:,:3]
                data = to_u8(data)
                

            # print(data.shape)
            data = cv2.resize(data, resize, interpolation=cv2.INTER_NEAREST)
            
            self.surface = pygame.surfarray.make_surface(data)
            self.rect = screen.blit(self.surface, relative_to_absolute(self.start, screen.get_size()))
            return self.rect
    ##########################

    def __init__(self):
        pygame.init()
        self.running = True

        # Make window resiasable
        self.screen = pygame.display.set_mode((512,512), pygame.RESIZABLE)
        
        self.font = pygame.font.SysFont('Corbel',25)

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
    dir_pick = control_interface.add_element(ControlInterface.PointLine(start=(0.8,0.15), length=0.15, color=(255,0,0), thicc=2))
    text_box_dir = control_interface.add_element(ControlInterface.Text(start=(0.65,0.30),prefix="Testing", font=control_interface.font, color=(252, 194, 3)))
    control_interface.add_element(ControlInterface.Line(start=(0.6,0.05), end=(0.6,0.95), color=(255,255,255), thicc=2))

    text_box_rgb = control_interface.add_element(ControlInterface.Text(prefix="Color: ", font=control_interface.font, color=(252, 194, 3)))
    img_rgb = control_interface.add_element(ControlInterface.Image(start=(0.02,0.02), max_size=(0.55, 0.3), text_box=text_box_rgb))

    text_box_depth = control_interface.add_element(ControlInterface.Text(prefix="Depth: ", font=control_interface.font, color=(252, 194, 3)))
    img_d = control_interface.add_element(ControlInterface.Image(start=(0.02,0.33), max_size=(0.55,0.3), colormap="plasma",vmin=10, vmax=60, text_box=text_box_depth))

    text_box_height = control_interface.add_element(ControlInterface.Text(prefix="Height: ", font=control_interface.font, color=(252, 194, 3)))
    img_hmap = control_interface.add_element(ControlInterface.Image(start=(0.02,0.66), max_size=(0.7,0.3), colormap="plasma",vmin=0, vmax=33, text_box=text_box_height))
    #####################################

    rate = rospy.Rate(30)
    while not rospy.is_shutdown() and control_interface.running:
        t = rospy.Time.now()
        
        text_box_dir.text = "Direction: " + str(dir_pick.u_dir)
        if data_in.rgb_ready:
            img_rgb.set_data(data_in.rgb)
        if data_in.d_ready:
            img_d.set_data(data_in.d)
        if data_in.hmap_ready:
            img_hmap.set_data(data_in.hmap*10)

        control_interface.update()
        
        ############## Push Hexapod Commands ##############
        command_msg.walk_dir = dir_pick.u_dir
        command_pub.publish(command_msg)
        ###################################################

        rate.sleep()
        td = (rospy.Time.now()-t)/1000000
        # print(str(td)+" ms")


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass