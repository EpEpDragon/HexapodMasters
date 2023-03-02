#!/usr/bin/env python

import rospy
import numpy as np
from hexapod_ros.msg import HexapodCommands, EffectorTargets
from walkStateMachine import WalkCycleMachine

def run():
    rospy.init_node('hexapod_movement')
    # Controls the walking gait and sets FINAL (in the current step) foot positions.
    walk_machine = WalkCycleMachine(None)
    direction = np.zeros(3)
    speed = 0.0
    
    def update(command_msg):
        direction = np.append(np.array(command_msg.walk_dir),0)
        walk_machine.update(direction, speed)
        # Append 0 to make direction 3D (x,y,0)
        # direction[2] = 0
        

    # Update walk machine when new commands arrive
    rospy.Subscriber('hexapod_command_data', HexapodCommands, update)

    effector_targets_pub = rospy.Publisher('effector_targets', EffectorTargets, queue_size=10)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        # walk_machine.update(direction, speed)
        effector_targets_pub.publish(walk_machine.targets)
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
