from math import sin, asin, cos, acos, tan, atan, pi
from math import copysign
from math import sqrt

from numpy import deg2rad, rad2deg
from numpy import array as nparr
from typing import NamedTuple
from collections import deque

NUM_ACTUATORS = 6

UPPER_LEG = 1
UPPER_LEG_2 = UPPER_LEG*UPPER_LEG
LOWER_LEG = 1
LOWER_LEG_2 = LOWER_LEG*LOWER_LEG

class Movement(NamedTuple):
    target: nparr
    vel: nparr
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
        return nparr([cos(curr_pitch),sin(curr_yaw),sin(curr_pitch)])*UPPER_LEG + nparr([cos(curr_pitch+curr_knee),sin(curr_yaw),sin(curr_pitch+curr_knee)])*LOWER_LEG
        
    def updateMoves(self, dt):
         for id in range(NUM_ACTUATORS):
            if len(self.movements[id]) == 0:
                continue

            curr_pos = self.find_foot_pos(id)
            curr_target = curr_pos + self.movements[id][0].vel * dt
            print(curr_target)

            [yaw,pitch,knee] = solveIK(curr_target[0], curr_target[1], curr_target[2])
            
            self.ctrl[id*3] = yaw
            self.ctrl[id*3 + 1] = pitch
            self.ctrl[id*3 + 2] = knee
            # self.ctrl[id] = self.ctrl[id] + copysign(self.movements[id][0].speed, self.movements[id][0].target - self.qpos[id + self.qpos_start_id]) * dt
            if max(abs(curr_target - self.movements[id][0].target)) < 0.01:
                self.movements[id].popleft()

    def addMovementS(self, target, id, speed):
        self.movements[id].append(Movement(target, speed))
    
    def addMovementT(self, target, id, time):
        self.movements[id].append(Movement(target, abs(self.qpos[id + self.qpos_start_id] - target)/time))
    
    def moveFoot(self, target, id, time):
        # [yaw,pitch,knee] = solveIK(x, y, z-self.height)
        curr_pos = self.find_foot_pos(id)
        diff = target - self.find_foot_pos(id)
        # dist = sqrt(diff.dot(diff))
        self.movements[id].append(Movement(target, (diff/time)))
        # print("Move foot %d: %f %f %f" % (foot_id, rad2deg(yaw),rad2deg(pitch), rad2deg(knee)))
        
        # leg1 = nparr([cos(pitch),sin(yaw),sin(pitch)])
        # leg1 *= UPPER_LEG
        # print(leg1)
        # leg2 = nparr([cos(pitch+knee),sin(yaw),sin(pitch+knee)])
        # leg2 *= LOWER_LEG
        # print(leg2)
        # foot = leg1+leg2
        # print(foot)

        # self.addMovementT(yaw,foot_id*3,time)
        # self.addMovementT(pitch,foot_id*3 + 1,time)
        # self.addMovementT(knee,foot_id*3 + 2,time)

    def setHeight(self, h):
        self.height = h
        print("Height set")



def stand(movement_handler):
    for id in range(int(NUM_ACTUATORS/3)):
        movement_handler.addMovementT(deg2rad(90),id*3+2,0.5)
        movement_handler.addMovementS(deg2rad(55),id*3+2,(pi/13)*3)

        movement_handler.addMovementT(deg2rad(-43),id*3+1,0.5)
        movement_handler.addMovementS(deg2rad(10),id*3+1,(pi/8)*3)