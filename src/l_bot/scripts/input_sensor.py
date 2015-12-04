#!/usr/bin/env python
"""
This is the ROS Node which acts the robots inout sensor. The input
is specifically from a keyboard given through stdin.
AUTHOR: Ashwinkumar Ganesan.
"""

import rospy
from std_msgs.msg import String, Int32
from l_bot.msg import Control

class KeyboardInput:
    def __init__(self):
        # Commands.
        self._commands = {'image': 1, 'do_test': 3, 'do_train': 4}
        self._exit_cm = "exit"
        self._test_cm = "test"
        self._train_cm = "train"

        # Create a publisher for the keyboard.
        self._message_pub = rospy.Publisher("/robot/messages", String, queue_size=10)

        # Create a publisher for the commands.
        # We sent commands from the keyboard to synchronize the frame and the associated message.
        self._command_pub = rospy.Publisher("/robot/commands", Int32, queue_size=10)

        self._command_pub1 = rospy.Publisher("/robot/commands1", Control, queue_size=10)

    # send input
    def send_input(self):
        while(1):
            print("I am here1")
            message = raw_input("Message: ")
            if (message == self._exit_cm):
                break

            if (message == self._test_cm):
                self._command_pub.publish(self._commands['do_test'])
            elif (message == self._train_cm):
                self._command_pub.publish(self._commands['do_train'])
            else:
                """
                # Send message to CPU.
                #self._message_pub.publish(message)

                # Send a command message too.
                #self._command_pub.publish(self._commands['image'])
                """
                print("I am here 2")
                msg = Control()

                msg.source = 1
                msg.target = 2
                msg.command = "hello"
                msg.type = 3
                msg.message = "my world"
                msg.action = "do something"

                self._command_pub1.publish(msg)



def listener():
    rospy.init_node('robot_input', anonymous=True)
    ki = KeyboardInput()
    ki.send_input()

if __name__ == '__main__':
    listener()

