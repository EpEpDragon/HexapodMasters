#!/usr/bin/env python

import rospy
import numpy as np
from hexapod_ros.msg import HexapodCommands
from walkStateMachine import WalkCycleMachine


def run():
    # Controls the walking gait and sets FINAL (in the current step) foot positions.
    walk_machine = WalkCycleMachine
    
    def update(command_msg):
        # Append 0 to make direction 3D (x,y,0)
        direction = np.array(command_msg.walk_dir.append(0))
        walk_machine.update(direction, command_msg.speed)

    # Update walk machine when new commands arrive
    rospy.Subscriber('hexapod_command_data', HexapodCommands, update)

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
