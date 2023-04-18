import mujoco
from mujoco import viewer
import time
from math import sin, cos, tan, pi
from numpy import deg2rad, rad2deg
from numpy import array as a
import os

import keyboard
import windowFuncs

from walkStateMachine import WalkCycleMachine
from controlInterface import ControInterface
import motion
from roboMath import rotate_vec

is_sim_running = True

def input(event):
    # print(get_active_window_title())
    window = windowFuncs.get_active_window_title()
    if window == "MuJoCo : MuJoCo Model" or  window == "Control Interface":
        if event.scan_code == keyboard.key_to_scan_codes("1")[0]:
            walk_machine.walk_direction = a([1, 1, 0])
        if event.scan_code == keyboard.key_to_scan_codes("2")[0]:
            walk_machine.walk_direction = a([0, 0, 0])
        if event.scan_code == keyboard.key_to_scan_codes(",")[0]:
            walk_machine.walk_direction = rotate_vec(walk_machine.walk_direction, a([0,0,1]),deg2rad(10))
        if event.scan_code == keyboard.key_to_scan_codes(".")[0]:
            walk_machine.walk_direction = rotate_vec(walk_machine.walk_direction, a([0,0,1]),deg2rad(-10))
        if event.scan_code == 1:
            os._exit(os.EX_OK)



if __name__ == '__main__':
    # Setup model
    model = mujoco.MjModel.from_xml_path("hexapod.xml")
    data = mujoco.MjData(model)
    timestep = model.opt.timestep

    # Start walkin state machine
    walk_machine = WalkCycleMachine()
    walk_machine.speed = 0.5

    # Start contorl interface
    control_interface = ControInterface(walk_machine)
    keyboard.on_press(input)

    # Start movement handler
    movement_handler = motion.MovementHandler(data.ctrl, data.qpos)

    # Start simulation
    viewer.launch_passive(model, data)
    time.sleep(1)
    windowFuncs.move_size_window("MuJoCo : MuJoCo Model",0,0,0,0.8,0.9)
    windowFuncs.move_size_window("Control Interface",0,0.8,0,0.2,0.9)
    # Used for real time sim
    error = 0.0 # Timestep error integrator
    start_time = time.perf_counter()
    dt = 0.0


    k = 0
    while is_sim_running:
        control_interface.update_input()
        if k % 20 == 0:
            control_interface.update()
            k = 0
        k += 1

        step_start = time.perf_counter()
        walk_machine.update(timestep)
        # Move actuators
        movement_handler.set_targets(walk_machine.foot_pos)
        movement_handler.update_moves(timestep)

        # Step by integrating timestep error to simulation in (approximatley) real time
        mujoco.mj_step(model, data)
        step_elapse = time.perf_counter() - step_start
        time.sleep(max(min(timestep - step_elapse - error, 1), 0)) # Delay remaining timestep - error
        dt = time.perf_counter() - step_start
        error +=  dt - timestep # Integrate error
