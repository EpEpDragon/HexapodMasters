from pyray import *
from raylib import colors as Color
from walkStateMachine import SPEED_MAX,HEIGHT_MAX

import windowFuncs as wf
import time

from sys import platform
import warnings
from simLaunch import READ_CAMERA
from math import cos, sin

from raylib import (
    MOUSE_BUTTON_LEFT,
    MOUSE_BUTTON_RIGHT,
    KEY_LEFT_SHIFT,
    KEY_SPACE,
    KEY_V,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_UP,
    KEY_DOWN,
    KEY_Z,
)

from numpy import (
    deg2rad, rad2deg,
    array as a
)

SCREEN_SCALE = 80
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 450

BODY_RADIUS = 20
FOOT_RADIUS = 10

    
def mouse_in_box(x,y,x2,y2):
    mouse_p = get_mouse_position()
    return (x < mouse_p.x) and (mouse_p.x < x2) and (y < mouse_p.y) and (mouse_p.y < y2)

class ProgressBar():
    def __init__(self,x,y,w,h,tab_x,c_front,c_back, lable, t_color=Color.WHITE, l_size=20,v_size=20) -> None:
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.c_front = c_front
        self.c_back = c_back
        self.tab_x = tab_x
        self.lable = lable
        self.t_color = t_color
        self.l_size = l_size
        self.l_y = int(self.y + self.h*0.5 - l_size/2)
        self.v_size = v_size
        self.v_x = int(self.x + self.w*0.1 + tab_x)
        self.v_y = int(self.y + self.h*0.5 - v_size/2)
    
    def update(self, p, v):
        draw_rectangle(self.x + self.tab_x, self.y,self.w, self.h, self.c_back)         # Background bar
        draw_rectangle(self.x + self.tab_x, self.y, int(self.w*p), self.h,self.c_front) # Foreground bar
        draw_text(self.lable, self.x, self.l_y, self.l_size, self.t_color)              # Lable text
        draw_text(v, self.v_x, self.v_y, self.v_size, self.t_color)                     # Value text


def start_interface(walk_machine, view):
    control_interface = ControInterface(walk_machine, view)
    if platform in ['Windows','win32','cywin']:
        warnings.warn("Not implemented on Windows OS")
    else:
        time.sleep(3)
        margins = wf.get_screen_margins()
        ctrl_x_rel = 370/(wf.get_monitor(0).width - margins[0])
        if READ_CAMERA:
            cam_size = wf.get_window_size('Camera')
            centerX = (wf.get_monitor(0).width - margins[0] - cam_size[0])/(wf.get_monitor(0).width - margins[0]) - ctrl_x_rel
            centerY = (wf.get_monitor(0).height - margins[1] - cam_size[1])/(wf.get_monitor(0).height - margins[1])
            wf.move_size_window("MuJoCo : MuJoCo Model", -1, 0, 0, centerX, 1)
            wf.move_size_window("Open3D", -1, centerX, 0, 1-centerX-ctrl_x_rel, centerY)
            wf.move_size_window("Control Interface", -1, 1-ctrl_x_rel, 0, ctrl_x_rel, 1)
            wf.move_size_window("Camera", -1, centerX, centerY, is_cv2=True)
        else:
            wf.move_size_window("MuJoCo : MuJoCo Model", -1, 0, 0, 1-ctrl_x_rel, 1)
            wf.move_size_window("Control Interface", -1, 1-ctrl_x_rel, 0, ctrl_x_rel, 1)
        
        
    control_interface.run()


class ControInterface():
    def __init__(self, walk_machine, view) -> None:
        self.walk_machine = walk_machine
        # self.cloud_vis = CloudVis()
        self.walk_direction = Vector2(0,0)
        self.speed_bar = ProgressBar(x=10, y=870, w=200, h=25, tab_x=100, c_front=Color.PURPLE, c_back=Color.DARKPURPLE, lable='Speed')
        self.height_bar = ProgressBar(x=10, y=900, w=200, h=25, tab_x=100, c_front=Color.PURPLE, c_back=Color.DARKPURPLE, lable='Height')
        self.image = load_image("machine.png")
        self.view = view
        set_target_fps(60)
        set_config_flags(ConfigFlags.FLAG_WINDOW_RESIZABLE)
        init_window(SCREEN_WIDTH, SCREEN_HEIGHT, "Control Interface")


    def run(self):
        while True:
            # Input
            # ----------------------------------------------------------------------------------
            if is_key_down(KEY_SPACE):
                self.walk_machine.adjust_height(0.01)
            elif is_key_down(KEY_LEFT_SHIFT):
                self.walk_machine.adjust_height(-0.01)
            if is_key_down(KEY_LEFT):
                self.walk_machine.adjust_local_yaw(-0.01)
            elif is_key_down(KEY_RIGHT):
                self.walk_machine.adjust_local_yaw(0.01)
            if is_key_down(KEY_UP):
                self.walk_machine.adjust_pitch(0.01)
            elif is_key_down(KEY_DOWN):
                self.walk_machine.adjust_pitch(-0.01)
            if is_key_down(KEY_Z):
                self.walk_machine.centering_yaw[:] = True
            if is_key_pressed(KEY_V):
                self.view[0] += 1
                self.view[0] = self.view[0] % (2)

            self.walk_machine.set_speed(self.walk_machine.speed + get_mouse_wheel_move()*0.1)
            body_pos = Vector2(get_screen_width() / 2.0 , 210)
            if is_mouse_button_down(MOUSE_BUTTON_LEFT):
                if mouse_in_box(0,0,get_screen_width(),420):
                    self.walk_direction = vector2_normalize(vector2_subtract(get_mouse_position(), body_pos))
            if is_mouse_button_down(MOUSE_BUTTON_RIGHT):
                if mouse_in_box(0,0,get_screen_width(),420):
                    self.walk_direction = Vector2(0,0)
            self.walk_machine.set_walk_direction(a([self.walk_direction.x, -self.walk_direction.y, 0.0]))
            # ----------------------------------------------------------------------------------

            # draw
            # ----------------------------------------------------------------------------------
            # self.cloud_vis.update()
            begin_drawing()
            clear_background((64, 64, 64, 255))
            # Draw Legs
            for id in range(6):
                if self.walk_machine.active[id]:
                    color = Color.GREEN
                else:
                    color = Color.RED
                foot_pos_screen = self.walk_machine.foot_pos_pre_yaw[id][0:2]*SCREEN_SCALE*a([1,-1]) + a([body_pos.x, body_pos.y])
                foot_pos_screen = Vector2(foot_pos_screen[0], foot_pos_screen[1])
                foot_target_screen = self.walk_machine.targets[id][0:2]*SCREEN_SCALE*a([1,-1]) + a([body_pos.x, body_pos.y])
                draw_line_ex(body_pos, foot_pos_screen,2, color)
                draw_circle_v(Vector2(foot_target_screen[0], foot_target_screen[1]), FOOT_RADIUS, Color.GRAY)
                draw_circle_v(foot_pos_screen, FOOT_RADIUS, color)

            # Draw direction
            if not vector2_equals(self.walk_direction, Vector2(0,0)):
                draw_line_ex(body_pos, vector2_add(body_pos, vector2_scale(self.walk_direction, 100)),4,Color.LIGHTGRAY)

            # Draw body
            draw_circle_v(body_pos, BODY_RADIUS, Color.YELLOW)
            # Local yaw direction
            yaw = max(self.walk_machine.current_yaw_local)
            draw_line_ex(body_pos, vector2_add(body_pos, Vector2(cos(yaw)*100, sin(yaw)*100)),3, Color.YELLOW)

            # Divider
            draw_line(10,820,get_screen_width()-10,820,Color.GRAY)
            draw_line(10,830,get_screen_width()-10,830,Color.GRAY)

            # Draw text
            if self.walk_machine.current_state == self.walk_machine.rest:
                color = Color.YELLOW
            elif self.walk_machine.current_state == self.walk_machine.stepping:
                color = Color.GREEN
            draw_text(self.walk_machine.current_state.name, 10, 840, 20, color)
            self.speed_bar.update(self.walk_machine.speed/SPEED_MAX, '%.2f' % self.walk_machine.speed)
            self.height_bar.update(self.walk_machine.height/HEIGHT_MAX, '%.2f' % self.walk_machine.height)

            end_drawing()
            # ----------------------------------------------------------------------------------


    def set_window_size(self, width: int,height: int):
        set_window_size(width, height)


