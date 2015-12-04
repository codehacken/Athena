#!/usr/bin/env python
"""
This is the ROS Node which acts the robots inout sensor. The input
is specifically from a keyboard given through stdin.
AUTHOR: Ashwinkumar Ganesan.
"""
__author__ = 'Ashwinkumar Ganesan'

from lib.node import RobotNode
from lib.transport import Message

class InputNode(RobotNode):
    _cpu_id = 2
    _output_node_id = 3

    def __init__(self, ID, node_name, topic_name):
        super(InputNode, self).__init__(ID, node_name, topic_name)

    def send_add_example(self, msg_str):
        msg = Message(self._t_layer._id, self._cpu_id, 'learn', 6, msg_str)
        self.send_message(msg)

    # send input
    def send_input(self):
        while(1):
            user_input = raw_input("Message: ")
            # If the user input is an exit command.
            if (user_input == "exit"):
                self.send_exit()

            # If the user wants to start the test phase or training phase.
            if (user_input == "test"):
                self.send_test()
            elif (user_input == "train"):
                self.send_train()
            else:
                self.send_add_example(user_input)

if __name__ == '__main__':
    robot_input = InputNode(1, 'robot_input', '/robot/messages')
    robot_input.send_input()

