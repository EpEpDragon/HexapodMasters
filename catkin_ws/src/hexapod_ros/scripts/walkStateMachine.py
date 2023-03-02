import rospy
from hexapod_ros.msg import EffectorTargets

from statemachine import StateMachine, State
import numpy as np
from numpy import array as a
from numpy import deg2rad, rad2deg
from math import sin,cos,tan, acos, sqrt
import math
# from roboMath import clerp, rotate_vec, rotate

REST_Z = 140 # mm
REST_POS = [ 
  a([207.846, -120.000, 0]),
  a([0.000, -240.000, 0]),
  a([-207.846, -120.000, 0]),
  a([-207.846, 120.000, 0]),
  a([-0.000, 240.000, 0]),
  a([207.846, 120.000, 0]),
]

# REST_POS = [a([0.866, 0.500, 0.0])*2, a([0.866, -0.500, 0.0])*2,
#             a([0.0, 1.000, 0.0])*2, a([0.0, -1.00, 0.0])*2,
#             a([-0.866, 0.500, 0.0])*2, a([-0.866, -0.500, 0.0])*2]

# HIP_VECTORS = [a([0.6062, 0.35, 0]), a([0.6062, -0.35, 0]),
#             a([0, 0.7, 0]), a([0, -0.7, 0]),
#             a([-0.6062, 0.35, 0]), a([-0.6062, -0.35, 0])]


STRIDE_LENGTH = 40 # mm
PLACE_TOLERANCE = 5 # mm
UP = a([0,0,1])
SPEED_MAX = 2
HEIGHT_MAX = 200 # mm
YAW_MAX = deg2rad(20)
PITCH_MAX = deg2rad(30)
BODY_RADIUS = 240


def find_angle(v):
    if v[1] > 0:
        return acos(np.clip((v.dot(a([1,0,0])))/sqrt(v.dot(v)), -1.0, 1.0))
    else:
        return -acos(np.clip((v.dot(a([1,0,0])))/sqrt(v.dot(v)), -1.0, 1.0))

def normalize(v):
    try:
        return v/sqrt(v.dot(v))
    except:
        return v/abs(v)


class WalkCycleMachine(StateMachine):
    "A walk cycle machine"
    rest = State("rest", initial=True, enter="deactivate_all")
    stepping = State("stepping", enter="find_is_swinging")

    walk = rest.to(stepping, cond="should_adjust") | stepping.to(rest, cond="step_finished") | rest.to.itself() | stepping.to.itself()

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
        self.foot_pos_pre_yaw = list(REST_POS)
        self.foot_pos_post_yaw = list(REST_POS)
        self.targets = list(REST_POS)
        self.perception = perception
        self.step_height = 0.3

        # Foot position feedback
        self.current_feet_positions = list(REST_POS)


        super(WalkCycleMachine, self).__init__()

    # Receive current feet positions
    def effector_pos_readback(self, msg):
        i = 0
        for vector in msg.targets:
            self.current_feet_positions[i](np.array(vector.data[0], vector.data[1], vector.data[2]))
            i += 1
            print("read %i" % i)
    rospy.Subscriber("effector_current_positions", EffectorTargets, effector_pos_readback)

    # Enter actions
    # -------------------------------------------------------------------------------------------
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
        # Select leg in walking sextant
        if deg2rad(0) >= angle and angle > deg2rad(-60):
            id = 0
        elif deg2rad(-60) >= angle and angle > deg2rad(-120):
            id = 1
        elif deg2rad(-120) >= angle and angle > deg2rad(-180):
            id = 2
        elif deg2rad(120) <= angle and angle < deg2rad(180):
            id = 3
        elif deg2rad(60) <= angle and angle < deg2rad(120):
            id = 4
        elif deg2rad(0) <= angle and angle < 60:
            id = 5
        
        # Select active legs based on leg in walking sextant
        if id != -1:
            self.is_swinging[id] = True
            self.is_swinging[id-1] = False
            self.is_swinging[id-2] = True
            self.is_swinging[id-3] = False
            self.is_swinging[id-4] = True            
            self.is_swinging[id-5] = False
        else:
            print("Active leg not found!")
        
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
            if not (abs(self.current_feet_positions[i] - self.targets[i]) < PLACE_TOLERANCE).all():
                return False
            if self.is_swinging[i]:
                if self.centering_yaw[i] and not abs(self.current_yaw_local[i]) < 0.001:
                    return False
                else:
                    self.centering_yaw[i] = False
        return True

    def should_adjust(self):
        for i in range(6):
            if not abs(self.current_feet_positions[i][2] - self.targets[i][2]) < PLACE_TOLERANCE:
                return True
            if self.centering_yaw[i]:
                return True
        return not (self.walk_direction == 0).all()
    # -------------------------------------------------------------------------------------------

    # Logic
    # -------------------------------------------------------------------------------------------
    def tick(self):
        self.walk()
        self._update_targets()
    
    def update_parameters(self, direction, speed):
        self._set_walk_direction(direction)
        self._set_speed(speed)
        
        # self._update_targets()
        # print(self.speed)
        # print(self.walk_direction)

        # Cycle state machine
        # self.walk()

        # dt = rospy.Time.now() - t_prev
        # # Update foot position for walking
        # if self.current_state == self.stepping:
        #     for i in range(6):
        #         if not (self.targets[i] - self.foot_pos_pre_yaw[i] == 0).all():
        #             self.foot_pos_pre_yaw[i] = self.foot_pos_pre_yaw[i] + (normalize(self.targets[i] - self.foot_pos_pre_yaw[i])*a([1,1,3])*self.speed*dt)

        # # Update foot position for local rotation
        # for i in range(6):
        #     self.current_yaw_local[i] = clerp(self.current_yaw_local[i], self.target_yaw_local[i], self.yaw_rate*dt)
        #     self.foot_pos_post_yaw[i] = rotate_vec(self.foot_pos_pre_yaw[i], UP, self.current_yaw_local[i])
        # print(self.targets[i] - self.foot_pos_pre_yaw[i])


    def _update_targets(self):
        """Update feet targets based on direction, speed and heightmap"""
        for i in range(6):
            # Vertical offsett based on heightmap

            effector_offset = -self.height #- self.perception.get_height_at_point(self.targets[i])
            # self.targets[i] = REST_POS[i] + (self.walk_direction * STRIDE_LENGTH)
            print(self.walk_direction)
            # self.targets[i][2] = self.height_offsets[i] + effector_offset
            if self.is_swinging[i]:
                # print("Swing")                
                # diff = self.foot_pos_pre_yaw[i] - self.targets[i]
                # dist = sqrt(diff.dot(diff))
                self.targets[i] = REST_POS[i] + (self.walk_direction * STRIDE_LENGTH)
                self.targets[i][2] = self.height_offsets[i] + effector_offset + 40
                
                # If not walking means rotationg in place, thus set foot height based on rotation
                # if (self.walk_direction == 0).all() and self.centering_yaw[i]:
                #     self.targets[i][2] += self.height_offsets[i] + effector_offset - min(abs(self.current_yaw_local[i])*3, 0.7)
                # else:
                #     self.targets[i][2] += self.height_offsets[i] +  effector_offset - min(dist, 0.7)

                if self.centering_yaw[i]:
                    self.target_yaw_local[i] = 0.0
            else:
                # print("Not swing")
                # Rotate walk direction to account for pitch angle and add to targets
                self.targets[i] = REST_POS[i] - (self.walk_direction * STRIDE_LENGTH)
                self.targets[i][2] += self.height_offsets[i] + effector_offset
            # print("leg %i target: %f %f %f" % (i,self.targets[i][0], self.targets[i][1], self.targets[i][2]))
        # print("-----------------")



    def _square_step(self, diff, i, dt):
        # if self.is_swinging[i]:
            if (abs(diff[0:2]) < PLACE_TOLERANCE).all():
                self.foot_pos_pre_yaw[i][2] = self.foot_pos_pre_yaw[i][2] + math.copysign(1,diff[2])*self.speed*dt
                return True
            if abs(diff[2] - self.step_height) > PLACE_TOLERANCE:
                self.foot_pos_pre_yaw[i][2] = self.foot_pos_pre_yaw[i][2] + math.copysign(1,diff[2] - self.step_height)*self.speed*dt
                return False
            self.foot_pos_pre_yaw[i][0:2] = self.foot_pos_pre_yaw[i][0:2] + normalize(diff[0:2])*self.speed*dt
            return True
        
    # -------------------------------------------------------------------------------------------

    def _is_long(self, id):
        """Is the leg extended?"""
        if self.current_feet_positions[id].dot(self.current_feet_positions[id]) > REST_POS[id].dot(REST_POS[id]):
            return True
        return False

    def _set_speed(self, value):
        self.speed = max(min(value, SPEED_MAX), 0)

    # TODO Standerdise coordinate frames
    def _set_walk_direction(self, value):
        self.walk_direction = value #rotate_vec(value, a([0,1,0]), self.pitch/2)

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
    sm = WalkCycleMachine(None)
    sm._graph().write_svg("machine.svg")