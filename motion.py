from math import sin, asin, cos, acos, tan, atan, pi
from math import sqrt

from numpy import deg2rad, rad2deg
import numpy as np
from collections import deque

import quaternion
from quaternion import from_vector_part, as_vector_part

from dataclasses import dataclass, field
from enum import Enum

from roboMath import rotate_vec

NUM_ACTUATORS = 6

UPPER_LEG = 1
UPPER_LEG_2 = UPPER_LEG*UPPER_LEG
LOWER_LEG = 1
LOWER_LEG_2 = LOWER_LEG*LOWER_LEG

# HIP_VECTORS = [a([0.6928, 0.4, 0]), a([0.6928, -0.4, 0]),
#             a([0, 0.8, 0]), a([0, -0.8, 0]),
#             a([-0.6928, 0.4, 0]), a([-0.6928, -0.4, 0])]

OFFSETS = [{"position" : np.array([0.6928, 0.4, 0.0]), "angle" : deg2rad(30)},
           {"position" : np.array([0.6928, -0.4, 0.0]), "angle" : deg2rad(-30)},
           {"position" : np.array([0.0, 0.8, 0.0]), "angle" : deg2rad(90)},
           {"position" : np.array([0.0, -0.8, 0.0]), "angle" : deg2rad(-90)},
           {"position" : np.array([-0.6928, 0.4, 0.0]), "angle" : deg2rad(150)},
           {"position" : np.array([-0.6928, -0.4, 0.0]), "angle" : deg2rad(-150)}]

class MoveType(Enum):
    LINEAR = 1
    SPHERICAL = 2

@dataclass
class Movement:
    target : np.array = field(default_factory=np.array)
    duration : float = 0
    
    # type : MoveType
    # start : np.array = np.array([0,0])
    # t : float = 0.0
    # next = None

# TODO Try make IK this work wothout sqrt
def solve_ik(x,y,z) -> list[float]:
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
        self.movements = [None,None,None,None,None,None]
        self.yaw = 0.0

    def find_foot_pos(self, id):
        curr_yaw = self.ctrl[id*3]
        curr_pitch = self.ctrl[id*3 + 1]
        curr_knee = self.ctrl[id*3 + 2]

        knee_rel = np.array([cos(curr_pitch)*cos(curr_yaw), cos(curr_pitch)*sin(curr_yaw), sin(curr_pitch)])*UPPER_LEG
        foot_rel = np.array([cos(curr_pitch+curr_knee)*cos(curr_yaw), cos(curr_pitch+curr_knee)*sin(curr_yaw), sin(curr_pitch+curr_knee)])*LOWER_LEG
        return  knee_rel + foot_rel
        
    def update_moves(self):
         for id in range(NUM_ACTUATORS):
            if self.movements[id] == None:
                continue

            # Solve IK
            # [yaw,pitch,knee] = solve_ik(curr_target, curr_target[1], curr_target[2])
            [yaw,pitch,knee] = solve_ik(self.movements[id].target[0], self.movements[id].target[1], self.movements[id].target[2])
            
            # Apply actuator commands
            self.ctrl[id*3] = yaw
            self.ctrl[id*3 + 1] = pitch
            self.ctrl[id*3 + 2] = knee


    def set_targets(self, targets):
        for id in range(6):
            self.movements[id] = Movement(rotate_vec(targets[id] - OFFSETS[id]["position"],np.array([0,0,1]), -OFFSETS[id]["angle"]))
        # self.yaw = yaw


    # def move_foot(self, target, id, time, type):
    #     curr_pos = self.find_foot_pos(id)
    #     target -= OFFSETS[id]["position"]
    #     target = rotate_vec(target,np.array([0,0,1]), -OFFSETS[id]["angle"])
    #     if len(self.movements[id]) == 0:
    #         self.movements[id].append(Movement(target, time, type, curr_pos))
    #     else:
    #         self.movements[id].append(Movement(target, time, type))



def stand(movement_handler):
    for id in range(int(NUM_ACTUATORS/3)):
        movement_handler.addMovementT(deg2rad(90),id*3+2,0.5)
        movement_handler.addMovementS(deg2rad(55),id*3+2,(pi/13)*3)

        movement_handler.addMovementT(deg2rad(-43),id*3+1,0.5)
        movement_handler.addMovementS(deg2rad(10),id*3+1,(pi/8)*3)