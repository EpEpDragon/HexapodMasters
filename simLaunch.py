import mujoco
from mujoco import viewer
import time
from math import sin, cos, tan, pi
from numpy import deg2rad, rad2deg
import numpy as np

from walkStateMachine import WalkCycleMachine
from walkStateMachine import REST_POS

import keyboard
import windowFuncs

import motion
from motion import MoveType

def input(event):

    # print(get_active_window_title())
    window = windowFuncs.get_active_window_title()
    if window == "MuJoCo : MuJoCo Model":
        if event.scan_code == keyboard.key_to_scan_codes("1")[0]:
            walk_machine.walk_direction = np.array([0,1])
        if event.scan_code == keyboard.key_to_scan_codes("2")[0]:
            walk_machine.walk_direction = np.array([0,0])
        #     # motion.stand(movement_handler)
        #     movement_handler.move_foot(np.array([0.707*2,0.707*2,0.0]),0,3, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([0.707*2,-0.707*2,0.0]),1,3, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([0,2,0.0]),2,3, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([0,-2,0.0]),3,3, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([-0.707*2,0.707*2,0.0]),4,3, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([-0.707*2,-0.707*2,0.0]),5,3, MoveType.LINEAR)
        # if event.scan_code == keyboard.key_to_scan_codes("2")[0]:
        #     movement_handler.move_foot(np.array([0.707*2,0.707*2,0.5]),0,3, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([0.707*2,-0.707*2,0.5]),1,3, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([0,2,0.5]),2,3, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([0,-2,0.5]),3,3, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([-0.707*2,0.707*2,0.5]),4,3, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([-0.707*2,-0.707*2,0.5]),5,3, MoveType.LINEAR)
        # # Rise
        # if event.scan_code == keyboard.key_to_scan_codes("3")[0]:
        #     # Active
        #     movement_handler.move_foot(np.array([0.707*2,0.707*2,0.7]),0,0.5, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([0,-2,0.7]),3,0.5, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([-0.707*2,0.707*2,0.7]),4,0.5, MoveType.LINEAR)

        #     # Inactive
        #     movement_handler.move_foot(np.array([0.707*2,-0.707*2,0.5]),1,0.5, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([0,2,0.5]),2,0.5, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([-0.707*2,-0.707*2,0.5]),5,0.5, MoveType.LINEAR)
        # # Forward
        # if event.scan_code == keyboard.key_to_scan_codes("4")[0]:
        #     # Active
        #     movement_handler.move_foot(np.array([0.707*2+0.3,0.707*2,0.7]),0,3, MoveType.LINEAR)
            
        #     # Inactive
        #     movement_handler.move_foot(np.array([0.707*2-0.3,-0.707*2,0.5]),1,3, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([0-0.3,2,0.5]),2,3, MoveType.LINEAR)
            
        #     # Active
        #     movement_handler.move_foot(np.array([0+0.3,-2,0.7]),3,3, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([-0.707*2+0.3,0.707*2,0.7]),4,3, MoveType.LINEAR)
            
        #     # Inactive
        #     movement_handler.move_foot(np.array([-0.707*2-0.3,-0.707*2,0.5]),5,3, MoveType.LINEAR)
        # # Fall
        # if event.scan_code == keyboard.key_to_scan_codes("5")[0]:
        #     # Active
        #     movement_handler.move_foot(np.array([0.707*2+0.3,0.707*2,0.5]),0,0.5, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([0+0.3,-2,0.5]),3,0.5, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([-0.707*2+0.3,0.707*2,0.5]),4,0.5, MoveType.LINEAR)

        #     # Inactive
        #     movement_handler.move_foot(np.array([0.707*2-0.3,-0.707*2,0.5]),1,0.5, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([0-0.3,2,0.5]),2,0.5, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([-0.707*2-0.3,-0.707*2,0.5]),5,0.5, MoveType.LINEAR)
        
        # # Rise2
        # if event.scan_code == keyboard.key_to_scan_codes("6")[0]:
        #     # Active
        #     movement_handler.move_foot(np.array([0.707*2-0.3,-0.707*2,0.7]),1,0.5, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([0-0.3,2,0.7]),2,0.5, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([-0.707*2-0.3,-0.707*2,0.7]),5,0.5, MoveType.LINEAR)

        #     # Inactive
        #     movement_handler.move_foot(np.array([0.707*2+0.3,0.707*2,0.5]),0,0.5, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([0+0.3,-2,0.5]),3,0.5, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([-0.707*2+0.3,0.707*2,0.5]),4,0.5, MoveType.LINEAR)
        # # Forward2
        # if event.scan_code == keyboard.key_to_scan_codes("7")[0]:
        #     # Active
        #     movement_handler.move_foot(np.array([0.707*2+0.3,-0.707*2,0.7]),1,3, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([0+0.3,2,0.7]),2,3, MoveType.LINEAR)
            
        #     # Inactive
        #     movement_handler.move_foot(np.array([0.707*2-0.3,0.707*2,0.5]),0,3, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([0-0.3,-2,0.5]),3,3, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([-0.707*2-0.3,0.707*2,0.5]),4,3, MoveType.LINEAR)
            
        #     # Active
        #     movement_handler.move_foot(np.array([-0.707*2+0.3,-0.707*2,0.7]),5,3, MoveType.LINEAR)
        # # Fall2
        # if event.scan_code == keyboard.key_to_scan_codes("8")[0]:
        #     # Active
        #     movement_handler.move_foot(np.array([0.707*2+0.3,-0.707*2,0.5]),1,1.5, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([0+0.3,2,0.5]),2,1.5, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([-0.707*2+0.3,-0.707*2,0.5]),5,1.5, MoveType.LINEAR)

        #     # Inactive
        #     movement_handler.move_foot(np.array([0.707*2-0.3,0.707*2,0.5]),0,1.5, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([0-0.3,-2,0.5]),3,1.5, MoveType.LINEAR)
        #     movement_handler.move_foot(np.array([-0.707*2-0.3,0.707*2,0.5]),4,1.5, MoveType.LINEAR)

if __name__ == '__main__':
    walk_machine = WalkCycleMachine()
    walk_machine.speed = 0.5

    keyboard.on_press(input)
    model = mujoco.MjModel.from_xml_path("hexapod.xml")
    mujoco.GLContext

    data = mujoco.MjData(model)
    timestep = model.opt.timestep
    error = 0.0 # Timestep error integrator

    movement_handler = motion.MovementHandler(data.ctrl, data.qpos)
    
    viewer.launch_passive(model, data)
    
    scene = mujoco.MjvScene(model, 20)
    start_time = time.perf_counter()
    dt = 0.0
    while True:
        step_start = time.perf_counter()
        walk_machine.update(walk_machine.walk_direction,walk_machine.speed,timestep)
        # Move actuators
        movement_handler.set_targets(walk_machine.foot_pos)
        movement_handler.update_moves(timestep)

        # Step by integrating timestep error to simulation in (approximatley) real time
        mujoco.mj_step(model, data)
        step_elapse = time.perf_counter() - step_start
        time.sleep(max(min(timestep - step_elapse - error, 1), 0))  # Delay remaining timestep - error
        dt = time.perf_counter() - step_start
        error +=  dt - timestep        # Integrate error
        # print(dt)
