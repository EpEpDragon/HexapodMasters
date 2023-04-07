"""

raylib [shapes] example - Following Eyes

"""
from walkStateMachine import WalkCycleMachine
from walkStateMachine import REST_POS

from pyray import *
from raylib import colors as Color
from numpy import (
    deg2rad, rad2deg,
    array as a
)

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
# foot_pos = [Vector2(86.6, -50.0), Vector2(86.6, 50.0),
#             Vector2(0, -100.0), Vector2(0, 100.0),
#             Vector2(-86.6, -50.0), Vector2(-86.6, 50.0)]

# walk_direction = vector2_normalize(vector2_subtract(get_mouse_position(), body_pos))
# angle = 0.0

walk_machine = WalkCycleMachine()
walk_machine.active



init_window(screenWidth, screenHeight, "Walk cycle test")

set_target_fps(60)
# ----------------------------------------------------------------------------------

# Main game loop
walk_direction = Vector2(0,0)
while not window_should_close():  # Detect window close button or ESC key
    # Input
    # ----------------------------------------------------------------------------------
    speed = walk_machine.speed + get_mouse_wheel_move()*5
    if is_mouse_button_down(MOUSE_BUTTON_LEFT):
        walk_direction = vector2_normalize(vector2_subtract(get_mouse_position(), body_pos))
    if is_mouse_button_down(MOUSE_BUTTON_RIGHT):
        walk_direction = Vector2(0,0)

    # Update
    # ----------------------------------------------------------------------------------
    dt = get_frame_time()
    walk_machine.update(a([walk_direction.x, walk_direction.y]), speed, dt)
    # ----------------------------------------------------------------------------------

    # draw
    # ----------------------------------------------------------------------------------
    begin_drawing()

    clear_background(Color.RAYWHITE)
    
    # Draw Legs
    for i in range(6):
        if walk_machine.active[i]:
            color = Color.GREEN
        else:
            color = Color.RED
        foot_pos_screen = walk_machine.foot_pos[i] + a([body_pos.x, body_pos.y])
        foot_pos_screen = Vector2(foot_pos_screen[0], foot_pos_screen[1])
        foot_target_screen = walk_machine.targets[i] + a([body_pos.x, body_pos.y])
        draw_line_ex(body_pos, foot_pos_screen,2, color)
        draw_circle_v(Vector2(foot_target_screen[0], foot_target_screen[1]), foot_radius, Color.GRAY)
        draw_circle_v(foot_pos_screen, foot_radius, color)
    
    # Draw body
    draw_circle_v(body_pos, body_radius, Color.RED)

    # Draw direction
    if not vector2_equals(walk_direction, Vector2(0,0)) :
        draw_line_ex(body_pos, vector2_add(body_pos, vector2_scale(walk_direction, 100)),5,Color.RED)
    
    # Draw text
    if walk_machine.current_state == walk_machine.rest:
        color = Color.RED
    elif walk_machine.current_state == walk_machine.stepping:
        color = Color.GREEN
    draw_text(walk_machine.current_state.name, 10, 10, 20, color)
    draw_text("Speed: %.2f" % walk_machine.speed, 10, 30, 20, Color.BLACK)
    draw_text("Angle: %.2f" % rad2deg(walk_machine.angle), 10, 50, 20, Color.BLACK)


    end_drawing()

# De-Initialization
close_window()  # Close window and OpenGL context
