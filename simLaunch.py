import mujoco
from mujoco import viewer
import time

from math import sin, cos, tan, pi
from numpy import deg2rad, rad2deg
from numpy import array as nparr

import keyboard
from activeWindow import get_active_window_title

import motion
from motion import MoveType

DT = 0.002

def input(event):
    # print(get_active_window_title())
    window = get_active_window_title()
    if str(window) == "b'MuJoCo : MuJoCo Model'":
        if event.scan_code == keyboard.key_to_scan_codes("1")[0]:
            # motion.stand(movement_handler)
            target = nparr([1.5,0,0.01])
            movement_handler.moveFoot(target,0,3, MoveType.LINEAR)
            movement_handler.moveFoot(target,1,3, MoveType.LINEAR)
            movement_handler.moveFoot(target,2,3, MoveType.LINEAR)
            movement_handler.moveFoot(target,3,3, MoveType.LINEAR)
            movement_handler.moveFoot(target,4,3, MoveType.LINEAR)
            movement_handler.moveFoot(target,5,3, MoveType.LINEAR)
        if event.scan_code == keyboard.key_to_scan_codes("2")[0]:
            target = nparr([1.5,0,1])
            movement_handler.moveFoot(target,0,3, MoveType.LINEAR)
            movement_handler.moveFoot(target,1,3, MoveType.LINEAR)
            movement_handler.moveFoot(target,2,3, MoveType.LINEAR)
            movement_handler.moveFoot(target,3,3, MoveType.LINEAR)
            movement_handler.moveFoot(target,4,3, MoveType.LINEAR)
            movement_handler.moveFoot(target,5,3, MoveType.LINEAR)
        if event.scan_code == keyboard.key_to_scan_codes("3")[0]:
            movement_handler.moveFoot(nparr([1.8,0,-0.6]), 0,3, MoveType.LINEAR)
            movement_handler.moveFoot(nparr([1.7,-0.5,-0.4]), 0,1.5, MoveType.LINEAR)
            movement_handler.moveFoot(nparr([1.8,0,-0.6]), 0,1.5, MoveType.LINEAR)
            movement_handler.moveFoot(nparr([1.8,0.5,-0.4]), 0,1.5, MoveType.LINEAR)
            movement_handler.moveFoot(nparr([1.7,0,-0.6]), 0,1.5, MoveType.LINEAR)
            movement_handler.moveFoot(nparr([1.5,0,1]), 0,3, MoveType.LINEAR)
        if event.scan_code == keyboard.key_to_scan_codes("4")[0]:
            movement_handler.moveFoot(nparr([1.8,0,-0.6]), 0,3, MoveType.LINEAR)
            movement_handler.moveFoot(nparr([1.7,-0.5,-0.4]), 0,1.5, MoveType.LINEAR)
            
        # if event.scan_code == keyboard.key_to_scan_codes("5")[0]:
            
        

if __name__ == '__main__':
    keyboard.on_press(input)
    model = mujoco.MjModel.from_xml_path("hexapod.xml")
    data = mujoco.MjData(model)
    timestep = model.opt.timestep
    error = 0.0 # Timestep error integrator

    movement_handler = motion.MovementHandler(data.ctrl, data.qpos)
    
    viewer.launch_passive(model, data)
    start_time = time.perf_counter()
    dt = 0.0
    while True:
        step_start = time.perf_counter()

        # Move actuators
        movement_handler.updateMoves(DT)

        # Step by integrating timestep error to simulation in (approximatley) real time
        mujoco.mj_step(model, data)
        step_elapse = time.perf_counter() - step_start
        time.sleep(max(min(timestep - step_elapse - error, 1), 0))  # Delay remaining timestep - error
        dt = time.perf_counter() - step_start
        error +=  dt - timestep        # Integrate error
        # print(dt)
