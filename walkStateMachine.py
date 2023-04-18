from statemachine import StateMachine, State
import numpy as np
from numpy import array as a
from numpy import deg2rad, rad2deg
from math import acos, sqrt

REST_Z = 0.5
REST_POS = [a([0.866, 0.500, REST_Z])*2, a([0.866, -0.500, REST_Z])*2,
            a([0.0, 1.000, REST_Z])*2, a([0.0, -1.00, REST_Z])*2,
            a([-0.866, 0.500, REST_Z])*2, a([-0.866, -0.500, REST_Z])*2]
STRIDE_LENGTH = 0.3
PLACE_TOLERANCE = 0.01

SPEED_MAX = 2
HEIGHT_MAX = 1


def find_angle(v):
    if v[1] > 0:        
        return acos(np.clip((v@a([1,0,0]))/sqrt(v@v), -1.0, 1.0))
    else:
        return -acos(np.clip((v@a([1,0,0]))/sqrt(v@v), -1.0, 1.0))


def normalize(v):
    try:
        return v/sqrt(v@v)
    except:
        return v/abs(v)


class WalkCycleMachine(StateMachine):
    "A walk cycle machine"
    rest = State(initial=True, enter="deactivate_all")
    stepping = State(enter="find_active")
     
    walk = rest.to(stepping, cond="has_speed") | stepping.to(rest, cond="step_finished") | rest.to.itself(internal=True) | stepping.to.itself(internal=True) 

    def __init__(self):
        self.active = a([False, False, False, False, False, False])
        self.speed = 0.5
        self.height = REST_Z*2
        self.walk_direction = a([0,0,0])
        self.foot_pos = list(REST_POS)
        self.targets = list(REST_POS)
        
        super(WalkCycleMachine, self).__init__()

    # Enter actions
    # -------------------------------------------------------------------------------------------
    def deactivate_all(self):
        self.active[0] = False
        self.active[1] = False
        self.active[2] = False
        self.active[3] = False
        self.active[4] = False
        self.active[5] = False

    def find_active(self):
        angle = find_angle(self.walk_direction)
        self.angle = angle
        if self.walk_direction is not None:
            id = -1
            if 0.0 <= angle and angle < deg2rad(60):
                id = 0
            elif deg2rad(60) <= angle and angle < deg2rad(120):
                id = 2
            elif deg2rad(120) <= angle and angle < deg2rad(180):
                id = 4
            elif deg2rad(-180) <= angle and angle < deg2rad(-120):
                id = 5
            elif deg2rad(-120) <= angle and angle < deg2rad(-60):
                id = 3
            elif deg2rad(-60.0) <= angle and angle < 0.0:
                id = 1
            if id == 0 or id == 3 or id == 4:
                self.active[0] = True                
                self.active[1] = False
                self.active[2] = False
                self.active[3] = True
                self.active[4] = True
                self.active[5] = False
            elif id == 1 or id == 2 or id == 5:
                self.active[0] = False
                self.active[1] = True
                self.active[2] = True
                self.active[3] = False
                self.active[4] = False
                self.active[5] = True
            else:
                print("id not found")
            if self.is_long(id):
                self.active = np.invert(self.active)
    # -------------------------------------------------------------------------------------------
    
    # Conditions
    # -------------------------------------------------------------------------------------------
    def step_finished(self):
        for i in range(6):
            if not (abs(self.foot_pos[i] - self.targets[i]) < PLACE_TOLERANCE).all():
                return False
        
        return True
    
    def has_speed(self):
        return not (self.walk_direction == a([0,0,0])).all()
    # -------------------------------------------------------------------------------------------

    # Logic
    # -------------------------------------------------------------------------------------------
    def update(self, dt):
        self._update_targets()
        self.walk()

        if self.current_state == self.stepping:
            for i in range(6):
                self.foot_pos[i] = self.foot_pos[i] + (normalize(self.targets[i] - self.foot_pos[i])*a([1,1,4])*self.speed*dt)


    def _update_targets(self):
        for i in range(6):
            if self.active[i]:
                diff = self.foot_pos[i] - self.targets[i]
                dist = sqrt(diff @ diff)
                self.targets[i] = REST_POS[i] + (self.walk_direction * STRIDE_LENGTH)
                self.targets[i][2] = self.height - min(dist, 0.1)
            else:
                self.targets[i] = REST_POS[i] - (self.walk_direction * STRIDE_LENGTH)
                self.targets[i][2] = self.height

    # -------------------------------------------------------------------------------------------

    def is_long(self, id):
        if self.foot_pos[id]@self.foot_pos[id] > REST_POS[id]@REST_POS[id]:
            return True
        return False

    def set_speed(self, value):
        self.speed = max(min(value, SPEED_MAX), 0)

    def set_height(self, value):
        self.height = max(min(value, HEIGHT_MAX), 0)

if __name__ == '__main__':
    sm = WalkCycleMachine()
    sm._graph().write_png("machine.png")