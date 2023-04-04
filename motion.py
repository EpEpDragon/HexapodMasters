from math import sin, asin, cos, acos, tan, atan, pi
from math import copysign
from math import sqrt

from numpy import deg2rad, rad2deg
from numpy import array as nparr
from typing import NamedTuple
from collections import deque

from dataclasses import dataclass
NUM_ACTUATORS = 6

UPPER_LEG = 1
UPPER_LEG_2 = UPPER_LEG*UPPER_LEG
LOWER_LEG = 1
LOWER_LEG_2 = LOWER_LEG*LOWER_LEG

@dataclass
class Movement:
    target : nparr
    duration : float
    start : nparr = nparr([0,0])
    t : float = 0.0
    next = None

def lerp(a,b,t):
    return a + (b-a)*t

# TODO Try make IK this work wothout sqrt
def solveIK(x,y,z) -> list[float]:
    # Root to target distance squared
    dist2 = x*x + y*y + z*z
    dist = sqrt(dist2)
    
    # Yaw angle
    yaw = atan(y/x)
    
    # Knee angle
    knee = acos((UPPER_LEG_2 + LOWER_LEG_2 - dist2) / (2*UPPER_LEG*LOWER_LEG))
    
    # Pitch angle
    full_pitch = acos(z / dist)
    # full_pitch = atan(z/(x*x + y*y))
    s_knee = sin(knee)
    inner_pitch = asin((s_knee*LOWER_LEG)/dist)
    pitch = pi/2 - full_pitch - inner_pitch

    return [yaw, pitch, pi - knee]


class MovementHandler:
    def __init__(self, ctrl, qpos) -> None:
        self.qpos = qpos
        self.qpos_start_id = qpos.size - NUM_ACTUATORS*3
        self.ctrl = ctrl
        self.movements = []
        self.height = 0.0
        for i in range(NUM_ACTUATORS) : self.movements.append(deque())

    def find_foot_pos(self, id):
        curr_yaw = self.ctrl[id*3]
        curr_pitch = self.ctrl[id*3 + 1]
        curr_knee = self.ctrl[id*3 + 2]

        knee_rel = nparr([cos(curr_pitch)*cos(curr_yaw), cos(curr_pitch)*sin(curr_yaw), sin(curr_pitch)])*UPPER_LEG
        foot_rel = nparr([cos(curr_pitch+curr_knee)*cos(curr_yaw), cos(curr_pitch+curr_knee)*sin(curr_yaw), sin(curr_pitch+curr_knee)])*LOWER_LEG
        return  knee_rel + foot_rel
        
    def updateMoves(self, dt):
         for id in range(NUM_ACTUATORS):
            if len(self.movements[id]) == 0:
                continue

            curr_target = lerp(self.movements[id][0].start, self.movements[id][0].target, self.movements[id][0].t)
            print(curr_target)
            if max(abs(curr_target - self.movements[id][0].target)) < 0.01:
                self.movements[id].popleft()
                if len(self.movements[id]) != 0:
                    self.movements[id][0].start = self.find_foot_pos(id)
                continue

            [yaw,pitch,knee] = solveIK(curr_target[0], curr_target[1], curr_target[2])
            
            self.ctrl[id*3] = yaw
            self.ctrl[id*3 + 1] = pitch
            self.ctrl[id*3 + 2] = knee
            self.movements[id][0].t += dt/self.movements[id][0].duration
            # self.ctrl[id] = self.ctrl[id] + copysign(self.movements[id][0].speed, self.movements[id][0].target - self.qpos[id + self.qpos_start_id]) * dt
            # if self.movements[id][0].t > 1:
            #     self.movements[id].popleft()

    def addMovementS(self, target, id, speed):
        self.movements[id].append(Movement(target, speed))
    
    def addMovementT(self, target, id, time):
        self.movements[id].append(Movement(target, abs(self.qpos[id + self.qpos_start_id] - target)/time))
    
    def moveFoot(self, target, id, time):
        curr_pos = self.find_foot_pos(id)
        # diff = target - self.find_foot_pos(id)
        if len(self.movements[id]) == 0:
            self.movements[id].append(Movement(target, time, curr_pos))
        else:
            self.movements[id].append(Movement(target, time))

    def setHeight(self, h):
        self.height = h
        print("Height set")



def stand(movement_handler):
    for id in range(int(NUM_ACTUATORS/3)):
        movement_handler.addMovementT(deg2rad(90),id*3+2,0.5)
        movement_handler.addMovementS(deg2rad(55),id*3+2,(pi/13)*3)

        movement_handler.addMovementT(deg2rad(-43),id*3+1,0.5)
        movement_handler.addMovementS(deg2rad(10),id*3+1,(pi/8)*3)