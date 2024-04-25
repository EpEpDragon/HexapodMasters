#!/usr/bin/env python

from ast import walk
import rospy
import numpy as np
from hexapod_ros.msg import HexapodCommands, EffectorTargets
from walkStateMachine import WalkCycleMachine

def run():
    rospy.init_node('hexapod_movement')
    # Controls the walking gait and sets FINAL foot positions.
    walk_machine = WalkCycleMachine(None)
    direction = np.zeros(3)
    speed = 0.0
    

    # Update walk machine when new commands arrive
    def update(command_msg):
        # Append 0 to make direction 3D (x,y,0)
        direction = np.append(np.array(command_msg.walk_dir),0)
        walk_machine.update_parameters(direction, command_msg.speed)
    rospy.Subscriber('hexapod_command_data', HexapodCommands, update)

    effector_targets_pub = rospy.Publisher('effector_targets', EffectorTargets, queue_size=10)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        walk_machine.tick()

        targets_msg = EffectorTargets()
        for i in range(6):
            targets_msg.targets[i].data[0] = walk_machine.targets[i][0]
            targets_msg.targets[i].data[1] = walk_machine.targets[i][1]
            targets_msg.targets[i].data[2] = walk_machine.targets[i][2]
        effector_targets_pub.publish(targets_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
