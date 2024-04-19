#!/usr/bin/env python

import rospy
import numpy as np
from hexapod_ros.msg import HexapodCommands
from walkStateMachine import WalkCycleMachine

def run():
    # Controls the walking gait and sets FINAL (in the current step) foot positions.
    walk_machine = WalkCycleMachine
    direction = np.zeros(3)
    speed = 0.0
    
    def update(command_msg):
        # Append 0 to make direction 3D (x,y,0)
        direction = np.array(command_msg.walk_dir.append(0))
        

    # Update walk machine when new commands arrive
    rospy.Subscriber('hexapod_command_data', HexapodCommands, update)

    rate = rospy.Rate(30)
    dt = 0.0
    while not rospy.is_shutdown():
        walk_machine.update(direction, speed, rospy.Time.now - t_start)
        t_start = rospy.Time.now()
        rate.sleep()
        dt = rospy.Time.now - t_start

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
