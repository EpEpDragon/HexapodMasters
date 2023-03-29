import mujoco
from mujoco import viewer
import time

from math import sin, cos, tan, pi


if __name__ == '__main__':
    model = mujoco.MjModel.from_xml_path("hexapod.xml")
    data = mujoco.MjData(model)
    timestep = model.opt.timestep
    error = 0.0 # Timestep error integrator

    viewer.launch_passive(model, data)
    start_time = time.perf_counter()

    while True:
        step_start = time.perf_counter()

        # Move test
        angle = sin(data.time)*(pi/8)+pi/8
        angle2 = sin(data.time*3)*(pi/8)+pi/8
        data.ctrl[1] = angle
        data.ctrl[2] = angle2
        data.ctrl[4] = angle
        data.ctrl[5] = angle2
        data.ctrl[7] = angle
        data.ctrl[8] = angle2
        data.ctrl[10] = angle
        data.ctrl[11] = angle2
        data.ctrl[13] = angle
        data.ctrl[14] = angle2
        data.ctrl[16] = angle
        data.ctrl[17] = angle2

        # Step by integrating timestep error to simulation in (approximatley) real time
        mujoco.mj_step(model, data)
        step_elapse = time.perf_counter() - step_start
        time.sleep(max(min(timestep - step_elapse - error, 1), 0))  # Delay remaining timestep - error
        error += time.perf_counter() - step_start - timestep        # Integrate error
