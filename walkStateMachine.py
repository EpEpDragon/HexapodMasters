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
PLACE_TOLERANCE = 0.05
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
        self.height_offsets = np.zeros(6)
        self.pitch = 0.0
        self.target_yaw_local = np.zeros(6)
        self.current_yaw_local = np.zeros(6)
        self.centering_yaw = np.full(6, False) # True when robot in process of centering yaw
        self.walk_direction = a([0,0,0])
        
        self.foot_pos_pre_yaw = np.array(REST_POS)
        self.foot_pos_post_yaw = np.array(REST_POS)
        
        # targets before optimisation
        self.targets_init = np.array(REST_POS)
        # optimised targets in map space
        self.targets_map = np.zeros((6,2),dtype=int)
        # targets in local space
        self.targets = np.array(REST_POS)
        
        self.perception = perception
        self.step_height = 0.3

        self.in_translation = self.is_swinging = np.full(6, False) # Translation phase for square step


        super(WalkCycleMachine, self).__init__()

    # Enter actions
    # -------------------------------------------------------------------------------------------
    def on_enter_rest(self):
        self._deactivate_all()

    def on_enter_stepping(self):
        self._find_is_swinging()
        self._set_target_anchors()
        

    def _deactivate_all(self):
        self.is_swinging[0] = False
        self.is_swinging[1] = False
        self.is_swinging[2] = False
        self.is_swinging[3] = False
        self.is_swinging[4] = False
        self.is_swinging[5] = False
    
    def _set_target_anchors(self):
        for i in range(6):
            if self.is_swinging[i]:
                # 2x stride lenght because target anchors for next step cycle
                self.targets_init[i] = (REST_POS[i] + self.walk_direction*STRIDE_LENGTH*2)
                new_target = self.perception.find_anchor(self.targets_init[i], ANCHOR_CORRECTION_RADIUS, ANCHOR_CORRECTION_THRESHOLD )
                
                # Check if a valid anchor is found. Fallback to unoptimised target
                # if new_target[0] != -1:
                    # self.targets_map[i] = new_target

                self.targets_map[i] = new_target
                print(time.time(), "set target_map")

                # else:
                    # print(i, "No valid anchor")
                    # self.targets_map[i], _ = self.perception._local_to_hmap(self.targets_init[i])

    def _find_is_swinging(self):
        has_direction = not (self.walk_direction == 0).all()
        if not has_direction and not self.centering_yaw.any():
            self._deactivate_all()
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
            if self._is_long(id):
                self.is_swinging = np.invert(self.is_swinging)
        else:
            if not (self.is_swinging == self.centering_yaw).all():
                self.is_swinging = np.invert(self.is_swinging)
    # -------------------------------------------------------------------------------------------

    # Conditions
    # -------------------------------------------------------------------------------------------
    def step_finished(self):
        for i in range(6):
            if not (abs(self.foot_pos_pre_yaw[i] - self.targets[i]) < PLACE_TOLERANCE).all():
                return False
            if self.is_swinging[i]:
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

    # Continuous Operations
    # -------------------------------------------------------------------------------------------
    def update(self, dt):
        self._update_targets()

        # Cycle state machine
        self.walk()

        # Update foot position for walking
        if self.current_state == self.stepping:
            for i in range(6):
                if not (self.targets[i] - self.foot_pos_pre_yaw[i] == 0).all():
                    self.foot_pos_pre_yaw[i] = self.foot_pos_pre_yaw[i] + (normalize(self.targets[i] - self.foot_pos_pre_yaw[i])*np.array([1,1,2])*self.speed*dt)

        # Update foot position for local rotation
        for i in range(6):
            self.current_yaw_local[i] = clerp(self.current_yaw_local[i], self.target_yaw_local[i], self.yaw_rate*dt)
            self.foot_pos_post_yaw[i] = rotate_vec(self.foot_pos_pre_yaw[i], UP, self.current_yaw_local[i])


    def _update_targets(self):
        for i in range(6):
            # self.targets[i] = self.perception._hmap_to_local(self.targets_map[i])
            if self.is_swinging[i]:
                self.targets[i] = self.perception._hmap_to_local(self.targets_map[i])
                # self.targets[i] = np.append(self.targets_map[i],0)
                # self.targets[i] = self.targets_init[i]
                # print(i, self.targets[i])
                # print(i, self.foot_pos_pre_yaw[i])
                diff = self.foot_pos_pre_yaw[i][0:2] - self.targets[i][0:2]
                dist = sqrt(diff @ diff)
                # inv = np.invert(self.is_swinging)
                
                # Adjust swinging target based on remaining supporting leg stride such that it stays in the same position relative to the terrain
                # not_swing_delta = self.targets[inv] - self.foot_pos_post_yaw[inv]
                # dot = not_swing_delta[0]@not_swing_delta[0]
                # dot += not_swing_delta[1]@not_swing_delta[1]
                # dot += not_swing_delta[2]@not_swing_delta[2]

                # self.targets_init[i] = REST_POS[i] + (self.walk_direction * (STRIDE_LENGTH + np.sum(np.sqrt(dot))/3))

                # new_target = self.perception.find_anchor(self.targets_init[i], ANCHOR_CORRECTION_RADIUS, ANCHOR_CORRECTION_THRESHOLD )
                

                # self.targets[i] = self.targets_init[i]                

                # Foot arcs
                # If not walking means rotationg in place, thus set foot height based on rotation
                if (self.walk_direction == 0).all() and self.centering_yaw[i]:
                    self.targets[i][2] += self.height_offsets[i] + self.height - min(abs(self.current_yaw_local[i])*3, 0.7) - self.perception.get_height_at_point(self.foot_pos_post_yaw[i])
                else:
                    # self.targets[i][2] += self.height_offsets[i] +  effector_offset - min(dist, 0.7)
                    # self.targets[i][2] += self.height_offsets[i] +  self.height - np.clip(10*(-dist*dist*0.25+0.6*dist*0.5),0,None) - self.perception.get_height_at_point(self.foot_pos_post_yaw[i])
                    
                    if dist < 0.05:
                        step = 0
                    else:
                        step = 0.6
                    # step = min(dist,0.6)
                    self.targets[i][2] += self.height_offsets[i] +  self.height - step - self.perception.get_height_at_point(self.targets[i])

                if self.centering_yaw[i]:
                        self.target_yaw_local[i] = 0.0
            else:
                # Rotate walk direction to account for pitch angle and add to targets
                self.targets_init[i] = (REST_POS[i] - (self.walk_direction * STRIDE_LENGTH))
                self.targets[i] = self.targets_init[i]
                
                self.targets[i][2] += self.height_offsets[i] + self.height - self.perception.get_height_at_point(self.foot_pos_post_yaw[i])
    # -------------------------------------------------------------------------------------------

    def _is_long(self, id):
        if self.foot_pos_post_yaw[id]@self.foot_pos_post_yaw[id] > REST_POS[id]@REST_POS[id]:
            return True
        return False

    def set_speed(self, value):
        self.speed = max(min(value, SPEED_MAX), 0)

    # TODO Standerdise coordinate frames
    def set_walk_direction(self, value):
        self.walk_direction = rotate_vec(value, a([0,1,0]), self.pitch/2)
        self._set_target_anchors()
        print(time.time(), "dir set")

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