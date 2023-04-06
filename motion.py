from math import sin, asin, cos, acos, tan, atan, pi
from math import sqrt

from numpy import deg2rad, rad2deg
import numpy as np
from collections import deque

import quaternion
from quaternion import from_vector_part, as_vector_part

from dataclasses import dataclass
from enum import Enum

NUM_ACTUATORS = 6

UPPER_LEG = 1
UPPER_LEG_2 = UPPER_LEG*UPPER_LEG
LOWER_LEG = 1
LOWER_LEG_2 = LOWER_LEG*LOWER_LEG

offsets = [{"position" : np.array([0.6062, 0.35, 0.0]), "angle" : deg2rad(30)},
           {"position" : np.array([0.6062, -0.35, 0.0]), "angle" : deg2rad(-30)},
           {"position" : np.array([0.0, 0.7, 0.0]), "angle" : deg2rad(90)},
           {"position" : np.array([0.0, -0.7, 0.0]), "angle" : deg2rad(-90)},
           {"position" : np.array([-0.6062, 0.35, 0.0]), "angle" : deg2rad(150)},
           {"position" : np.array([-0.6062, -0.35, 0.0]), "angle" : deg2rad(-150)}]

class MoveType(Enum):
    LINEAR = 1
    SPHERICAL = 2

@dataclass
class Movement:
    target : np.array
    duration : float
    type : MoveType
    start : np.array = np.array([0,0])
    t : float = 0.0
    next = None

def lerp(a,b,t):
    return a + (b-a)*t

def slerp(a,b,t):
    ang = acos(a.dot(b)/sqrt(a.dot(a)*b.dot(b)))
    return (sin((1-t)*ang)/sin(ang))*a + (sin(t*ang)/sin(ang))*b

def rotate_vec(vector, axis, angle):
    angle /= 2
    s = sin(angle)
    q = np.quaternion(cos(angle), s*axis[0], s*axis[1], s*axis[2])
    return as_vector_part(q*from_vector_part(vector)*q.conjugate())

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
        self.movements = []
        self.height = 0.0
        for i in range(NUM_ACTUATORS) : self.movements.append(deque())

    def find_foot_pos(self, id):
        curr_yaw = self.ctrl[id*3]
        curr_pitch = self.ctrl[id*3 + 1]
        curr_knee = self.ctrl[id*3 + 2]

        knee_rel = np.array([cos(curr_pitch)*cos(curr_yaw), cos(curr_pitch)*sin(curr_yaw), sin(curr_pitch)])*UPPER_LEG
        foot_rel = np.array([cos(curr_pitch+curr_knee)*cos(curr_yaw), cos(curr_pitch+curr_knee)*sin(curr_yaw), sin(curr_pitch+curr_knee)])*LOWER_LEG
        return  knee_rel + foot_rel
        
    def update_moves(self, dt):
         for id in range(NUM_ACTUATORS):
            if len(self.movements[id]) == 0:
                continue
                
            # Interpolate foot position
            match self.movements[id][0].type:
                case MoveType.LINEAR:
                    curr_target = lerp(self.movements[id][0].start, self.movements[id][0].target, self.movements[id][0].t)
                case MoveType.SPHERICAL:
                    c = self.movements[id][0].start + (self.movements[id][0].target - self.movements[id][0].start)/2
                    curr_target = c + slerp(self.movements[id][0].start - c, self.movements[id][0].target - c, self.movements[id][0].t)

            # Solve IK
            [yaw,pitch,knee] = solve_ik(curr_target[0], curr_target[1], curr_target[2])
            
            # Apply actuator commands
            self.ctrl[id*3] = yaw
            self.ctrl[id*3 + 1] = pitch
            self.ctrl[id*3 + 2] = knee
            self.movements[id][0].t += dt/self.movements[id][0].duration

            # Check finish condition
            if max(abs(curr_target - self.movements[id][0].target)) < 0.01:
                self.movements[id].popleft()
                if len(self.movements[id]) != 0:
                    self.movements[id][0].start = self.find_foot_pos(id)
                continue


    def move_foot(self, target, id, time, type):
        curr_pos = self.find_foot_pos(id)
        target -= offsets[id]["position"]
        target = rotate_vec(target,np.array([0,0,1]), -offsets[id]["angle"])
        if len(self.movements[id]) == 0:
            self.movements[id].append(Movement(target, time, type, curr_pos))
        else:
            self.movements[id].append(Movement(target, time, type))


    def set_height(self, h):
        self.height = h
        print("Height set")



def stand(movement_handler):
    for id in range(int(NUM_ACTUATORS/3)):
        movement_handler.addMovementT(deg2rad(90),id*3+2,0.5)
        movement_handler.addMovementS(deg2rad(55),id*3+2,(pi/13)*3)

        movement_handler.addMovementT(deg2rad(-43),id*3+1,0.5)
        movement_handler.addMovementS(deg2rad(10),id*3+1,(pi/8)*3)