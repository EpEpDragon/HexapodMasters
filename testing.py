"""

raylib [shapes] example - Following Eyes

"""
from walkStateMachine import WalkCycleMachine

from pyray import *
from raylib import colors as color
from numpy import deg2rad
from math import (
    atan2,
    cos,
    sin
)

from raylib import (
    MOUSE_BUTTON_LEFT,
    MOUSE_BUTTON_MIDDLE,
    MOUSE_BUTTON_RIGHT,
    MOUSE_BUTTON_SIDE,
    MOUSE_BUTTON_EXTRA,
    MOUSE_BUTTON_FORWARD,
    MOUSE_BUTTON_BACK
)

# Initialization
# ----------------------------------------------------------------------------------
screenWidth = 800
screenHeight = 450

body_radius = 20
foot_radius = 10

body_pos = Vector2(screenWidth / 2.0 , screenHeight / 2.0)
foot_pos = [Vector2(86.6, -50.0), Vector2(86.6, 50.0), 
            Vector2(0, -100.0), Vector2(0, 100.0), 
            Vector2(-86.6, -50.0), Vector2(-86.6, 50.0)]

walk_direction = vector2_normalize(vector2_subtract(get_mouse_position(), body_pos))
                               
walk_machine = WalkCycleMachine()
walk_machine.active

def find_active(walk_direction):
    if walk_direction is None:
        print("Rest")
    else:
        angle = vector2_angle(walk_direction, Vector2(1,0))
        if (0.0 < angle and angle < deg2rad(60)) or (deg2rad(120) < angle and angle < deg2rad(180)) or (deg2rad(240) < angle and angle < deg2rad(300)):
            walk_machine.active = [0,3,4]
        else:
            walk_machine.active= [1,2,5]
        print(angle)

init_window(screenWidth, screenHeight, "Walk cycle test")

set_target_fps(60)
# ----------------------------------------------------------------------------------

# Main game loop
while not window_should_close():  # Detect window close button or ESC key
    # Update
    # ----------------------------------------------------------------------------------
    if is_mouse_button_down(MOUSE_BUTTON_LEFT):
        walk_direction = vector2_normalize(vector2_subtract(get_mouse_position(), body_pos))
    elif is_mouse_button_down(MOUSE_BUTTON_RIGHT):
        walk_direction = None
    find_active(walk_direction)
    # ----------------------------------------------------------------------------------

    # draw
    # ----------------------------------------------------------------------------------
    begin_drawing()

    clear_background(color.RAYWHITE)

    draw_circle_v(body_pos, body_radius, color.RED)

    if walk_machine.active[0] == 0:
        draw_circle_v(vector2_add(foot_pos[0], body_pos), foot_radius, color.GREEN)
        draw_circle_v(vector2_add(foot_pos[1], body_pos), foot_radius, color.RED)
        draw_circle_v(vector2_add(foot_pos[2], body_pos), foot_radius, color.RED)
        draw_circle_v(vector2_add(foot_pos[3], body_pos), foot_radius, color.GREEN)
        draw_circle_v(vector2_add(foot_pos[4], body_pos), foot_radius, color.GREEN)
        draw_circle_v(vector2_add(foot_pos[5], body_pos), foot_radius, color.RED)
    else:
        draw_circle_v(vector2_add(foot_pos[0], body_pos), foot_radius, color.RED)
        draw_circle_v(vector2_add(foot_pos[1], body_pos), foot_radius, color.GREEN)
        draw_circle_v(vector2_add(foot_pos[2], body_pos), foot_radius, color.GREEN)
        draw_circle_v(vector2_add(foot_pos[3], body_pos), foot_radius, color.RED)
        draw_circle_v(vector2_add(foot_pos[4], body_pos), foot_radius, color.RED)
        draw_circle_v(vector2_add(foot_pos[5], body_pos), foot_radius, color.GREEN)

    if walk_direction is not None:
        draw_line_ex(body_pos, vector2_add(body_pos, vector2_scale(walk_direction, 100)),5,color.RED)
    
    draw_fps(10, 10)

    end_drawing()

# De-Initialization
close_window()  # Close window and OpenGL context
