#!/usr/bin/env python

import rospy

from hexapod_ros.msg import HexapodCommands

def run():
    rospy.init_node('hexapod_command_listner')
    rospy.Subscriber("hexapod_command_data", HexapodCommands, commands_callback)
    rospy.spin()

def commands_callback(data):
    pass
    # print("Receive " + str(data))

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass