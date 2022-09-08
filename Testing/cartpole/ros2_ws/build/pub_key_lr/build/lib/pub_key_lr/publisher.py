import sys, os

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

import rclpy
# from rclpy.node import Node

from std_msgs.msg import Int8


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = os.read(sys.stdin.fileno(), 3).decode()
    if len(key) == 3:
        key = ord(key[2])
    else:
        key = ord(key)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    print("Start")
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('key_lr')
    pub = node.create_publisher(Int8, 'cmd_lr', 10)
    cmd_lr = Int8()
    cmd_lr.data = 0
    try:
        while True:
            key = getKey(settings)
            # print(key)
            if key == 67:
                print('right')
                cmd_lr.data = 1
            elif key == 68:
                print('left')
                cmd_lr.data = -1
            elif key == 27:
                print('quit')
                quit()
            else:
                cmd_lr.data = 0
            pub.publish(cmd_lr)

    except Exception as e:
        print(e)

    finally:
        cmd_lr.data = 0
        pub.publish(cmd_lr)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
