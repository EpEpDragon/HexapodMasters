from pyray import *
from raylib import colors as Color

from raylib import (
    MOUSE_BUTTON_LEFT,
    MOUSE_BUTTON_MIDDLE,
    MOUSE_BUTTON_RIGHT,
    MOUSE_BUTTON_SIDE,
    MOUSE_BUTTON_EXTRA,
    MOUSE_BUTTON_FORWARD,
    MOUSE_BUTTON_BACK
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


class ControInterface():
    def __init__(self) -> None:
        self.walk_direction = Vector2(0,0)
        set_config_flags(ConfigFlags.FLAG_WINDOW_RESIZABLE)
        init_window(SCREEN_WIDTH, SCREEN_HEIGHT, "Control Interface")
    
    def update(self, walk_machine):
        # Input
        # ----------------------------------------------------------------------------------
        walk_machine.speed = walk_machine.speed + get_mouse_wheel_move()*0.1
        # Walk direction 
        body_pos = Vector2(get_screen_width() / 2.0 , get_screen_height() / 2.0)
        if is_mouse_button_down(MOUSE_BUTTON_LEFT):
            self.walk_direction = vector2_normalize(vector2_subtract(get_mouse_position(), body_pos))
        if is_mouse_button_down(MOUSE_BUTTON_RIGHT):
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

        # Draw text
        if walk_machine.current_state == walk_machine.rest:
            color = Color.YELLOW
        elif walk_machine.current_state == walk_machine.stepping:
            color = Color.GREEN
        draw_text(walk_machine.current_state.name, 10, 10, 20, color)
        draw_text("Speed: %.2f" % walk_machine.speed, 10, 30, 20, Color.LIGHTGRAY)
        end_drawing()
        # ----------------------------------------------------------------------------------
    
    def set_window_size(self, width: int,height: int):
        set_window_size(width, height)


