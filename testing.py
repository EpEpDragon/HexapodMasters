"""

raylib [shapes] example - Following Eyes

"""
from walkStateMachine import WalkCycleMachine
from walkStateMachine import rest_pos

from pyray import *
from raylib import colors as Color
from numpy import deg2rad, rad2deg
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

speed = 5
body_radius = 20
foot_radius = 10

body_pos = Vector2(screenWidth / 2.0 , screenHeight / 2.0)
# foot_pos = [Vector2(86.6, -50.0), Vector2(86.6, 50.0),
#             Vector2(0, -100.0), Vector2(0, 100.0),
#             Vector2(-86.6, -50.0), Vector2(-86.6, 50.0)]

# walk_direction = vector2_normalize(vector2_subtract(get_mouse_position(), body_pos))
angle = 0.0

walk_machine = WalkCycleMachine()
walk_machine.active

def find_active(angle):
    if walk_machine.walk_direction is not None:
        if (0.0 < angle and angle < deg2rad(60)) or (deg2rad(120) < angle and angle < deg2rad(180)) or (deg2rad(-120) < angle and angle < deg2rad(-60)):
            walk_machine.active = [0,3,4]
            walk_machine.inactive = [1,2,5]
        else:
            walk_machine.active = [1,2,5]
            walk_machine.inactive = [0,3,4]

init_window(screenWidth, screenHeight, "Walk cycle test")

set_target_fps(60)
# ----------------------------------------------------------------------------------

# Main game loop
while not window_should_close():  # Detect window close button or ESC key
    # Update
    # ----------------------------------------------------------------------------------
    dt = get_frame_time()
    if is_mouse_button_down(MOUSE_BUTTON_LEFT):
        walk_machine.walk_direction = vector2_normalize(vector2_subtract(get_mouse_position(), body_pos))
    elif is_mouse_button_down(MOUSE_BUTTON_RIGHT):
        walk_machine.walk_direction = Vector2(0,0)
    angle = vector2_angle(walk_machine.walk_direction, Vector2(1,0))

    find_active(angle)
    # if walk_machine.has_speed() and walk_machine.step_finished():
    walk_machine.walk()

    if walk_machine.current_state == walk_machine.stepping:
        for i in range(6):
            walk_machine.foot_pos[i] = vector2_add(walk_machine.foot_pos[i], vector2_scale(vector2_subtract(walk_machine.targets[i], walk_machine.foot_pos[i]), speed*dt))
   
    # ----------------------------------------------------------------------------------

    # draw
    # ----------------------------------------------------------------------------------
    begin_drawing()

    clear_background(Color.RAYWHITE)

    # Draw body
    draw_circle_v(body_pos, body_radius, Color.RED)
    
    # Draw feet
    for i in walk_machine.active:
        draw_circle_v(vector2_add(walk_machine.targets[i], body_pos), foot_radius, Color.GRAY)
        draw_circle_v(vector2_add(walk_machine.foot_pos[i], body_pos), foot_radius, Color.GREEN)

    for i in walk_machine.inactive:
        draw_circle_v(vector2_add(walk_machine.targets[i], body_pos), foot_radius, Color.GRAY)
        draw_circle_v(vector2_add(walk_machine.foot_pos[i], body_pos), foot_radius, Color.RED)
    
    
    # Draw direction
    if not vector2_equals(walk_machine.walk_direction, Vector2(0,0)) :
        draw_line_ex(body_pos, vector2_add(body_pos, vector2_scale(walk_machine.walk_direction, 100)),5,Color.RED)
    
    # Draw text
    draw_fps(10, 10)
    if walk_machine.current_state == walk_machine.rest:
        color = Color.RED
    elif walk_machine.current_state == walk_machine.stepping:
        color = Color.GREEN
    draw_text(walk_machine.current_state.name, 10, 30, 20, color)
    draw_text(str(rad2deg(angle)), 10, 50, 20, Color.GRAY)
    draw_text(str(walk_machine.step_finished()), 10, 70, 20, Color.GRAY)

    end_drawing()

# De-Initialization
close_window()  # Close window and OpenGL context
