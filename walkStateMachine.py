from statemachine import StateMachine, State
import numpy as np
from numpy import array as a
from numpy import deg2rad, rad2deg
from math import sin,cos,tan, acos, sqrt
from roboMath import clerp, rotate_vec

REST_Z = 0.6
REST_POS = [a([0.866, 0.500, 0.0])*2, a([0.866, -0.500, 0.0])*2,
            a([0.0, 1.000, 0.0])*2, a([0.0, -1.00, 0.0])*2,
            a([-0.866, 0.500, 0.0])*2, a([-0.866, -0.500, 0.0])*2]
STRIDE_LENGTH = 0.3
PLACE_TOLERANCE = 0.01
UP = a([0,0,1])
SPEED_MAX = 2
HEIGHT_MAX = 1.15
YAW_MAX = deg2rad(20)
PITCH_MAX = deg2rad(30)
BODY_RADIUS = 0.7


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
    stepping = State(enter="find_is_supporting")

    walk = rest.to(stepping, cond="should_adjust") | stepping.to(rest, cond="step_finished") | rest.to.itself(internal=True) | stepping.to.itself(internal=True)

    def __init__(self, perception):
        self.is_supporting = np.full(6, False) # List defining if a foot is is_supporting or swinging
        self.speed = 0.5
        self.yaw_rate = deg2rad(25)
        self.height = REST_Z
        self.height_offsets = np.zeros(6)
        self.pitch = 0.0
        self.target_yaw_local = np.zeros(6)
        self.current_yaw_local = np.zeros(6)
        self.centering_yaw = np.full(6, False) # True when robot in process of centering yaw
        self.walk_direction = a([0,0,0])
        self.foot_pos_pre_yaw = list(REST_POS)
        self.foot_pos_post_yaw = list(REST_POS)
        self.targets = list(REST_POS)
        self.perception = perception

        super(WalkCycleMachine, self).__init__()

    # Enter actions
    # -------------------------------------------------------------------------------------------
    def deactivate_all(self):
        self.is_supporting[0] = False
        self.is_supporting[1] = False
        self.is_supporting[2] = False
        self.is_supporting[3] = False
        self.is_supporting[4] = False
        self.is_supporting[5] = False

    def find_is_supporting(self):
        has_direction = not (self.walk_direction == 0).all()
        if not has_direction and not self.centering_yaw.any():
            self.deactivate_all()
            return
        
        if not has_direction:
            angle = 0
        else:
            angle = find_angle(self.walk_direction)
        
        self.angle = angle
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
            self.is_supporting[0] = True
            self.is_supporting[1] = False
            self.is_supporting[2] = False
            self.is_supporting[3] = True
            self.is_supporting[4] = True
            self.is_supporting[5] = False
        elif id == 1 or id == 2 or id == 5:
            self.is_supporting[0] = False
            self.is_supporting[1] = True
            self.is_supporting[2] = True
            self.is_supporting[3] = False
            self.is_supporting[4] = False
            self.is_supporting[5] = True
        else:
            print("id not found")
        
        # Check inversion required
        if has_direction:
            if self.is_long(id):
                self.is_supporting = np.invert(self.is_supporting)
        else:
            if not (self.is_supporting == self.centering_yaw).all():
                self.is_supporting = np.invert(self.is_supporting)
    # -------------------------------------------------------------------------------------------

    # Conditions
    # -------------------------------------------------------------------------------------------
    def step_finished(self):
        for i in range(6):
            if not (abs(self.foot_pos_pre_yaw[i] - self.targets[i]) < PLACE_TOLERANCE).all():
                return False
            if self.is_supporting[i]:
                if self.centering_yaw[i] and not abs(self.current_yaw_local[i]) < 0.001:
                    return False
                else:
                    self.centering_yaw[i] = False
        return True

    def should_adjust(self):
        for i in range(6):
            if not abs(self.foot_pos_pre_yaw[i][2] - self.targets[i][2]) < PLACE_TOLERANCE:
                return True
            if self.centering_yaw[i]:
                return True
        return not (self.walk_direction == 0).all()
    # -------------------------------------------------------------------------------------------

    # Logic
    # -------------------------------------------------------------------------------------------
    def update(self, dt):
        self._update_targets()

        # Cycle state machine
        self.walk()

        # Update foot position for walking
        if self.current_state == self.stepping:
            for i in range(6):
                if not (self.targets[i] - self.foot_pos_pre_yaw[i] == 0).all():
                    self.foot_pos_pre_yaw[i] = self.foot_pos_pre_yaw[i] + (normalize(self.targets[i] - self.foot_pos_pre_yaw[i])*a([1,1,3])*self.speed*dt)

        # Update foot position for local rotation
        for i in range(6):
            self.current_yaw_local[i] = clerp(self.current_yaw_local[i], self.target_yaw_local[i], self.yaw_rate*dt)
            self.foot_pos_post_yaw[i] = rotate_vec(self.foot_pos_pre_yaw[i], UP, self.current_yaw_local[i])


    def _update_targets(self):
        for i in range(6):
            if self.is_supporting[i]:
                diff = self.foot_pos_pre_yaw[i] - self.targets[i]
                dist = sqrt(diff @ diff)
                self.targets[i] = REST_POS[i] + (self.walk_direction * STRIDE_LENGTH)
                # self.targets[i][2] += self.height - self.perception.get_height_at_point(self.targets[i])

                # If not walking means rotationg in place, thus set foot height based on rotation
                if (self.walk_direction == 0).all() and self.centering_yaw[i]:
                    self.targets[i][2] += self.height_offsets[i] - min(abs(self.current_yaw_local[i])*3, 0.7) + self.height
                else:
                    self.targets[i][2] += self.height_offsets[i] - min(dist, 0.7) +  self.height - self.perception.get_height_at_point(self.targets[i])
                    
                if self.centering_yaw[i]:
                    self.target_yaw_local[i] = 0.0
            else:
                # Rotate walk direction to account for pitch angle and add to targets
                self.targets[i] = REST_POS[i] - (self.walk_direction * STRIDE_LENGTH)
                # self.targets[i][2] += (self.height - self.perception.get_height_at_point(self.targets[i]))
                self.targets[i][2] += self.height_offsets[i] + self.height

    # -------------------------------------------------------------------------------------------

    def is_long(self, id):
        if self.foot_pos_post_yaw[id]@self.foot_pos_post_yaw[id] > REST_POS[id]@REST_POS[id]:
            return True
        return False

    def set_speed(self, value):
        self.speed = max(min(value, SPEED_MAX), 0)

    # TODO Standerdise coordinate frames
    def set_walk_direction(self, value):
        self.walk_direction = rotate_vec(value, a([0,1,0]), self.pitch/2)

    def adjust_height(self, value):
        self.height += value
        self.height = min(max(self.height,0), HEIGHT_MAX)
    
    def adjust_pitch(self, value):
        self.pitch += value
        self.pitch = min(max(self.pitch,-PITCH_MAX), PITCH_MAX)
        offset = sin(self.pitch)*BODY_RADIUS
        self.height_offsets[0:2] = -offset
        self.height_offsets[4:6] = offset

    def adjust_local_yaw(self, value):
        for i in range(6):
                # if not self.is_supporting[i]:
                if (-YAW_MAX <= self.target_yaw_local + value).all() and (self.target_yaw_local + value <= YAW_MAX).all():
                    self.target_yaw_local[i] += value
                    self.target_yaw_local[i] = min(max(self.target_yaw_local[i],-YAW_MAX),YAW_MAX)

if __name__ == '__main__':
    sm = WalkCycleMachine()
    sm._graph().write_svg("machine.svg")