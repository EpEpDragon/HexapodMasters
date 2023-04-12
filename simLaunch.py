import mujoco
from mujoco import viewer
import time
from math import sin, cos, tan, pi
from numpy import deg2rad, rad2deg
from numpy import array as a

import keyboard
import windowFuncs

from walkStateMachine import WalkCycleMachine

from controlInterface import ControInterface

import motion
from motion import MoveType

from roboMath import rotate_vec

def input(event):
    # print(get_active_window_title())
    window = windowFuncs.get_active_window_title()
    if window == "MuJoCo : MuJoCo Model":
        if event.scan_code == keyboard.key_to_scan_codes("1")[0]:
            walk_machine.walk_direction = a([1, 1, 0])
        if event.scan_code == keyboard.key_to_scan_codes("2")[0]:
            walk_machine.walk_direction = a([0, 0, 0])
        if event.scan_code == keyboard.key_to_scan_codes(",")[0]:
            walk_machine.walk_direction = rotate_vec(walk_machine.walk_direction, a([0,0,1]),deg2rad(10))
        if event.scan_code == keyboard.key_to_scan_codes(".")[0]:
            walk_machine.walk_direction = rotate_vec(walk_machine.walk_direction, a([0,0,1]),deg2rad(-10))



if __name__ == '__main__':
    # Setup model
    model = mujoco.MjModel.from_xml_path("hexapod.xml")
    data = mujoco.MjData(model)
    timestep = model.opt.timestep
    
    # Start walkin state machine
    walk_machine = WalkCycleMachine()
    walk_machine.speed = 0.5

    # Start contorl interface
    # control_interface = ControInterface()
    keyboard.on_press(input)
    
    # Start movement handler
    movement_handler = motion.MovementHandler(data.ctrl, data.qpos)    

    # Start simulation
    viewer.launch_passive(model, data)
    time.sleep(1)
    windowFuncs.move_size_window("MuJoCo : MuJoCo Model",1,0,0,0.8,1)
    windowFuncs.move_size_window("Control Interface",1,0.8,0,0.2,1)
    # Used for real time sim
    error = 0.0 # Timestep error integrator
    start_time = time.perf_counter()
    dt = 0.0
    
    while True:
        # control_interface.update(walk_machine)
        step_start = time.perf_counter()
        walk_machine.update(walk_machine.walk_direction,walk_machine.speed,timestep)
        # Move actuators
        movement_handler.set_targets(walk_machine.foot_pos)
        movement_handler.update_moves(timestep)

        # Step by integrating timestep error to simulation in (approximatley) real time
        mujoco.mj_step(model, data)
        step_elapse = time.perf_counter() - step_start
        time.sleep(max(min(timestep - step_elapse - error, 1), 0)) # Delay remaining timestep - error
        dt = time.perf_counter() - step_start
        error +=  dt - timestep # Integrate error
