from math import sin, asin, cos, acos, tan, atan, pi
from math import copysign
from math import sqrt

from numpy import deg2rad, array
from typing import NamedTuple
from collections import deque

NUM_ACTUATORS = 18

UPPER_LEG = 1
UPPER_LEG_2 = UPPER_LEG*UPPER_LEG
LOWER_LEG = 0.9
LOWER_LEG_2 = LOWER_LEG*LOWER_LEG

class Movement(NamedTuple):
    target: float
    speed: float
    next = None

def lerp(a,b,t):
    return a + (b-a)*t


def solveIK(x,y,z) -> list[float]:
    # Root to target distance squared
    dist2 = x*x + y*y + z*z
    
    # Yaw angle
    yaw = atan(y/x)
    
    # Knee angle
    knee = acos((UPPER_LEG_2 + LOWER_LEG_2 - dist2) / (2*UPPER_LEG*LOWER_LEG))
    
    # Pitch angle
    full_pitch = acos((z*z) / dist2)
    s_knee = sin(knee)
    inner_pitch = asin((s_knee*s_knee*LOWER_LEG)/dist2)
    pitch = full_pitch - inner_pitch

    return [yaw, pitch, knee]


class MovementHandler:
    def __init__(self, ctrl, qpos) -> None:
        self.qpos = qpos
        self.qpos_start_id = qpos.size - NUM_ACTUATORS
        self.ctrl = ctrl
        self.movements = []
        for i in range(NUM_ACTUATORS) : self.movements.append(deque())

    def updateMoves(self, dt):
         for id in range(NUM_ACTUATORS):
            if len(self.movements[id]) == 0 :
                continue

            self.ctrl[id] = self.ctrl[id] + copysign(self.movements[id][0].speed, self.movements[id][0].target - self.qpos[id + self.qpos_start_id]) * dt
            if abs(self.ctrl[id] - self.movements[id][0].target) < 0.02:
                self.movements[id].popleft()

    def addMovementS(self, target, id, speed):
        self.movements[id].append(Movement(target, speed))
    
    def addMovementT(self, target, id, time):
        self.movements[id].append(Movement(target, abs(self.qpos[id + self.qpos_start_id] - target)/time))
        


def stand(movement_handler):
    for id in range(int(NUM_ACTUATORS/3)):
        movement_handler.addMovementT(deg2rad(90),id*3+2,0.5)
        movement_handler.addMovementS(deg2rad(55),id*3+2,(pi/13)*3)

        movement_handler.addMovementT(deg2rad(-43),id*3+1,0.5)
        movement_handler.addMovementS(deg2rad(10),id*3+1,(pi/8)*3)