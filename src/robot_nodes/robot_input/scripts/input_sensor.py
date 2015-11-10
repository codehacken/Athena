#!/usr/bin/env python
"""
This is the ROS Node which acts the robots inout sensor. The input
is specifically from a keyboard given through stdin.
AUTHOR: Ashwinkumar Ganesan.
"""

import rospy
from std_msgs.msg import String, Int32

class KeyboardInput:
    def __init__(self):
        # Commands.
        self._commands = {'image': 1}
        self._exit_cm = "exit"

        # Create a publisher for the keyboard.
        self._message_pub = rospy.Publisher("/robot/messages", String, queue_size=10)

        # Create a publisher for the commands.
        # We sent commands from the keyboard to synchronize the frame and the associated message.
        self._command_pub = rospy.Publisher("/robot/commands", Int32, queue_size=10)

    # send input
    def send_input(self):
        while(1):
            message = raw_input("Message: ")
            if (message == self._exit_cm):
                break

            # Send message to CPU.
            self._message_pub.publish(message)

            # Send a command message too.
            self._command_pub.publish(self._commands['image'])

def listener():
    rospy.init_node('robot_input', anonymous=True)
    ki = KeyboardInput()
    ki.send_input()

if __name__ == '__main__':
    listener()

