from pyray import *
from raylib import colors as Color
from walkStateMachine import SPEED_MAX,HEIGHT_MAX
# from raylib import FontType

from raylib import (
    MOUSE_BUTTON_LEFT,
    MOUSE_BUTTON_RIGHT,
    KEY_LEFT_SHIFT,
    KEY_SPACE,
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

class ControInterface():
    def __init__(self) -> None:
        self.walk_direction = Vector2(0,0)
        self.speed_bar = ProgressBar(10, 470, 300, 25, 200, Color.PURPLE, Color.DARKPURPLE, 'Speed')
        self.height_bar = ProgressBar(10, 500, 300, 25, 200, Color.PURPLE, Color.DARKPURPLE, 'Height')
        set_config_flags(ConfigFlags.FLAG_WINDOW_RESIZABLE)
        init_window(SCREEN_WIDTH, SCREEN_HEIGHT, "Control Interface")
    
    def update(self, walk_machine):
        # Input
        # ----------------------------------------------------------------------------------
        walk_machine.set_speed(walk_machine.speed + get_mouse_wheel_move()*0.1)
        if is_key_down(KEY_SPACE):
            walk_machine.set_height(walk_machine.height + 0.01)
        elif is_key_down(KEY_LEFT_SHIFT):
            walk_machine.set_height(walk_machine.height - 0.01)

        # Walk direction
        body_pos = Vector2(get_screen_width() / 2.0 , 210)
        if is_mouse_button_down(MOUSE_BUTTON_LEFT):
            if mouse_in_box(0,0,get_screen_width(),420):
                self.walk_direction = vector2_normalize(vector2_subtract(get_mouse_position(), body_pos))
        if is_mouse_button_down(MOUSE_BUTTON_RIGHT):
            if mouse_in_box(0,0,get_screen_width(),420):
                self.walk_direction = Vector2(0,0)
        walk_machine.walk_direction = a([self.walk_direction.x, -self.walk_direction.y, 0.0])
        # ----------------------------------------------------------------------------------

        # draw
        # ----------------------------------------------------------------------------------
        begin_drawing()
        clear_background((64, 64, 64, 255))
        # Draw Legs
        for id in range(6):
            if walk_machine.active[id]:
                color = Color.GREEN
            else:
                color = Color.RED
            foot_pos_screen = walk_machine.foot_pos[id][0:2]*SCREEN_SCALE*a([1,-1]) + a([body_pos.x, body_pos.y])
            foot_pos_screen = Vector2(foot_pos_screen[0], foot_pos_screen[1])
            foot_target_screen = walk_machine.targets[id][0:2]*SCREEN_SCALE*a([1,-1]) + a([body_pos.x, body_pos.y])
            draw_line_ex(body_pos, foot_pos_screen,2, color)
            draw_circle_v(Vector2(foot_target_screen[0], foot_target_screen[1]), FOOT_RADIUS, Color.GRAY)
            draw_circle_v(foot_pos_screen, FOOT_RADIUS, color)

        # Draw direction
        if not vector2_equals(self.walk_direction, Vector2(0,0)):
            draw_line_ex(body_pos, vector2_add(body_pos, vector2_scale(self.walk_direction, 100)),4,Color.LIGHTGRAY)

        # Draw body
        draw_circle_v(body_pos, BODY_RADIUS, Color.YELLOW)

        # Divider
        draw_line(10,420,get_screen_width()-10,420,Color.GRAY)
        draw_line(10,430,get_screen_width()-10,430,Color.GRAY)

        # Draw text
        if walk_machine.current_state == walk_machine.rest:
            color = Color.YELLOW
        elif walk_machine.current_state == walk_machine.stepping:
            color = Color.GREEN
        draw_text(walk_machine.current_state.name, 10, 440, 20, color)
        # draw_text("Speed: %.2f" % walk_machine.speed, 10, 470, 20, Color.LIGHTGRAY)
        self.speed_bar.update(walk_machine.speed/SPEED_MAX, '%.2f' % walk_machine.speed)
        self.height_bar.update(walk_machine.height/HEIGHT_MAX, '%.2f' % walk_machine.height)

        end_drawing()
        # ----------------------------------------------------------------------------------

    def set_window_size(self, width: int,height: int):
        set_window_size(width, height)


