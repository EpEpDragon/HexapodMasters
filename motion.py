from math import sin, cos, tan, pi, copysign
from numpy import deg2rad
from typing import NamedTuple
from collections import deque

NUM_ACTUATORS = 18

class Movement(NamedTuple):
    target: float
    speed: float
    next = None

def lerp(a,b,t):
    return a + (b-a)*t


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