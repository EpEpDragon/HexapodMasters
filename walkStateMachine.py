from statemachine import StateMachine, State
import numpy as np
from numpy import array as a
from numpy import deg2rad, rad2deg
from math import sin,cos,tan, acos, sqrt
from roboMath import clerp, rotate_vec, rotate
import time

REST_Z = 0.6
REST_POS = [a([0.866, 0.500, 0.0])*2, a([0.866, -0.500, 0.0])*2,
            a([0.0, 1.000, 0.0])*2, a([0.0, -1.00, 0.0])*2,
            a([-0.866, 0.500, 0.0])*2, a([-0.866, -0.500, 0.0])*2]

STRIDE_LENGTH = 0.3
PLACE_TOLERANCE = 0.15
UP = a([0,0,1])
SPEED_MAX = 2
HEIGHT_MAX = 1.15
YAW_MAX = deg2rad(20)
PITCH_MAX = deg2rad(30)
BODY_RADIUS = 0.7

ANCHOR_CORRECTION_RADIUS = 8
ANCHOR_CORRECTION_THRESHOLD = 0.4



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
    rest = State(initial=True)
    stepping = State()

    walk = rest.to(stepping, cond="should_adjust") | stepping.to(rest, cond="step_finished") | rest.to.itself(internal=True) | stepping.to.itself(internal=True)

    def __init__(self, perception):
        self.is_swinging = np.full(6, False) # List defining if a foot is is_swinging or swinging
        self.speed = 0.5
        self.yaw_rate = deg2rad(25)
        self.height = REST_Z
        self.floor_height = 0.0
        self.height_offsets = np.zeros(6)
        self.pitch = 0.0
        self.target_yaw_local = np.zeros(6)
        self.current_yaw_local = np.zeros(6)
        self.centering_yaw = np.full(6, False) # True when robot in process of centering yaw
        self.walk_direction = a([0,0,0])
        
        self.foot_pos_pre_yaw = np.array(REST_POS)
        self.foot_pos_post_yaw = np.array(REST_POS)
        self.is_move_valid = True
        self.invert_gait = False
        
        self.targets_init= np.array(REST_POS)
        self.targets = np.array(REST_POS)
        self.targets_map = np.zeros((6,2))
        self.targets_prev = np.array(REST_POS)
        
        self.perception = perception
        self.step_height = 0.3

        self.in_translation = self.is_swinging = np.full(6, False) # Translation phase for square step


        super(WalkCycleMachine, self).__init__()

    # Enter actions
    # -------------------------------------------------------------------------------------------
    def on_enter_rest(self):
        self.deactivate_all()

    def on_enter_stepping(self):
        self.find_is_swinging()
        self.select_targets()

    def select_targets(self):
        move_vector_avg = 0
        for i in (self.is_swinging == False).nonzero()[0]:
            move_vector = (REST_POS[i] - (self.walk_direction * STRIDE_LENGTH)) - self.targets_init[i]
            move_vector_avg += move_vector
            self.targets[i] = self.targets_prev[i] + move_vector
            self.targets[i][2] = self.height_offsets[i] + self.height + self.floor_height - self.perception.get_height_at_point(self.foot_pos_post_yaw[i])
        supporting_stride_avg = np.sqrt(move_vector_avg@move_vector_avg)/3
        print("Supp stride:", supporting_stride_avg)

        for i in (self.is_swinging==True).nonzero()[0]:
            self.targets_init[i] = REST_POS[i] + (self.walk_direction * (STRIDE_LENGTH))
            targets_init_far = REST_POS[i] + (self.walk_direction * (STRIDE_LENGTH + supporting_stride_avg))
            optimised_target = self.perception.find_anchor(targets_init_far, ANCHOR_CORRECTION_RADIUS, ANCHOR_CORRECTION_THRESHOLD)
            if optimised_target[0] != -1:
                self.targets_map[i] = optimised_target
                self.is_move_valid = True
            else:
                self.is_move_valid = False


    def deactivate_all(self):
        self.is_swinging[0] = False
        self.is_swinging[1] = False
        self.is_swinging[2] = False
        self.is_swinging[3] = False
        self.is_swinging[4] = False
        self.is_swinging[5] = False

    def find_is_swinging(self):
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
            self.is_swinging[0] = True
            self.is_swinging[1] = False
            self.is_swinging[2] = False
            self.is_swinging[3] = True
            self.is_swinging[4] = True
            self.is_swinging[5] = False
        elif id == 1 or id == 2 or id == 5:
            self.is_swinging[0] = False
            self.is_swinging[1] = True
            self.is_swinging[2] = True
            self.is_swinging[3] = False
            self.is_swinging[4] = False
            self.is_swinging[5] = True
        else:
            print("id not found")
        
        # Check inversion required
        if has_direction:
            if self.invert_gait:
                print("Invert")
                self.is_swinging = np.invert(self.is_swinging)
            else:
                print("Dont Invert")
            self.invert_gait = not self.invert_gait
        else:
            if not (self.is_swinging == self.centering_yaw).all():
                self.is_swinging = np.invert(self.is_swinging)
        
    # -------------------------------------------------------------------------------------------

    # Conditions
    # -------------------------------------------------------------------------------------------
    def step_finished(self):
        for i in range(6):
            if (abs(self.foot_pos_pre_yaw[i] - self.targets[i]) > PLACE_TOLERANCE).any():
                return False
            if self.is_swinging[i]:
                if self.centering_yaw[i] and not abs(self.current_yaw_local[i]) < 0.001:
                    return False
                else:
                    self.centering_yaw[i] = False
        print(time.time(), "step fin:", abs(self.foot_pos_pre_yaw[i] - self.targets[i]), (abs(self.foot_pos_pre_yaw[i] - self.targets[i]) > PLACE_TOLERANCE).all())
        return True

    def should_adjust(self):
        for i in range(6):
            if (abs(self.foot_pos_pre_yaw[i] - self.targets[i]) > PLACE_TOLERANCE).any():
                print(time.time(), "adjust:", abs(self.foot_pos_pre_yaw[i] - self.targets[i]), )
                return True
            if self.centering_yaw[i]:
                return True
        return not (self.walk_direction == 0).all()
    # -------------------------------------------------------------------------------------------

    # Per cycle logic
    # -------------------------------------------------------------------------------------------
    def update(self, dt):
        # Cycle state machine
        self.walk()
        self._update_targets()
        self._update_floor_height()

        if self.is_move_valid:
            # Update foot position for walking
            if self.current_state == self.stepping:
                for i in range(6):
                    if not (self.targets[i] - self.foot_pos_pre_yaw[i] == 0).all():
                        # self.foot_pos_pre_yaw[i] = self.foot_pos_pre_yaw[i] + (normalize(self.targets[i] - self.foot_pos_pre_yaw[i])*a([1,1,3])*self.speed*dt)
                        if self.is_swinging[i]:
                            # pass
                            self.foot_pos_pre_yaw[i] = self.foot_pos_pre_yaw[i] + normalize(self._calculate_flow(1,10,i))*self.speed*dt*a([1,1,3])
                        else:
                            self.foot_pos_pre_yaw[i] = self.foot_pos_pre_yaw[i] + (normalize(self.targets[i] - self.foot_pos_pre_yaw[i])*self.speed*dt)


            # Update foot position for local rotation
            for i in range(6):
                self.current_yaw_local[i] = clerp(self.current_yaw_local[i], self.target_yaw_local[i], self.yaw_rate*dt)
                self.foot_pos_post_yaw[i] = rotate_vec(self.foot_pos_pre_yaw[i], UP, self.current_yaw_local[i])
    
    def _calculate_flow(self, Ch, q, i):
            diff = self.targets[i] - self.foot_pos_pre_yaw[i]
            dist = sqrt(diff @ diff)
            # Mult 100 for scaling (cm to mm)
            x = max(sqrt(diff[:2] @ diff[:2]) * 100, 0.00001)
            y = diff[2] * 100
            Fa = -abs(Ch/x) - abs(0.515*(y-q) / (1+abs(y-q)) - 0.513)
            
            Fb = y/x - Fa*x
            Ftheta = np.arctan(2*Fa*x + Fb)

            Ex = np.cos(Ftheta)
            Ey = np.sin(Ftheta)
            
            return np.append((diff/dist)[:2]*Ex, Ey)

    def _update_floor_height(self):
        """Set floor height to the average of next anchor points"""
        if (self.is_swinging==True).any():
            max_heights = np.zeros(6)
            for i in range(6):
                max_heights[i] = self.perception.get_height_at_point(self.targets[i])
            max_heights.sort()
            max_heights = max_heights[3:]
            self.floor_height = np.sum(max_heights)/3
    
    def _update_targets(self):
        """Update the foot targets"""
        self.is_move_valid = True
        for i in (self.is_swinging==True).nonzero()[0]:
            diff = self.foot_pos_pre_yaw[i][0:2] - self.targets[i][0:2]
            dist = sqrt(diff @ diff)

            self.targets[i] = self.perception._hmap_to_local(self.targets_map[i])
            # If new target valid
            if self.targets[i][0] != -1:
                # Foot arcs
                # If not walking means rotationg in place, thus set foot height based on rotation
                if (self.walk_direction == 0).all() and self.centering_yaw[i]:
                    self.targets[i][2] = self.height_offsets[i] + self.height + self.floor_height - min(abs(self.current_yaw_local[i])*3, 1.5) - self.perception.get_height_at_point(self.targets[i])
                else:
                    step = min(abs(3.5*dist), 1.5)
                    self.targets[i][2] = self.height_offsets[i] +  self.height + self.floor_height - self.perception.get_height_at_point(self.targets[i])
                if self.centering_yaw[i]:
                    self.target_yaw_local[i] = 0.0
                self.targets_prev[i] = self.targets[i]
            else:
                # If invalid set target to current position (Stop leg)
                self.is_move_valid = False
                print(time.time(), i, "Invalid Anchor")
    # -------------------------------------------------------------------------------------------

    def set_speed(self, value):
        self.speed = max(min(value, SPEED_MAX), 0)

    # TODO Standerdise coordinate frames
    def set_walk_direction(self, value):
        self.walk_direction = rotate_vec(value, a([0,1,0]), self.pitch/2)
        self.select_targets()

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
                # if not self.is_swinging[i]:
                if (-YAW_MAX <= self.target_yaw_local + value).all() and (self.target_yaw_local + value <= YAW_MAX).all():
                    self.target_yaw_local[i] += value
                    self.target_yaw_local[i] = min(max(self.target_yaw_local[i],-YAW_MAX),YAW_MAX)

if __name__ == '__main__':
    sm = WalkCycleMachine()
    sm._graph().write_svg("machine.svg")