#!/usr/bin/env python
"""
This is the ROS Node which acts the robots inout sensor. The input
is specifically from a keyboard given through stdin.
AUTHOR: Ashwinkumar Ganesan.
"""
__author__ = 'Ashwinkumar Ganesan'

from lib.node import RobotNode
from lib.transport import Message
from lib.node import mode as act_pass

import sys

class InputNode(RobotNode):
    _cpu_id = 2
    _output_node_id = 3

    def __init__(self, ID, node_name, topic_name, mode_type='passive'):
        super(InputNode, self).__init__(ID, node_name, topic_name)
        self._msg_state = act_pass[mode_type]

    def send_add_example(self, msg_str):
        msg = Message(self._t_layer._id, self._cpu_id, 'learn', 6, msg_str)
        self.send_message(msg)

    def send_test_case(self, msg_str):
        msg = Message(self._t_layer._id, self._cpu_id, 'test', 1, msg_str)
        self.send_message(msg)

    # This is the message that is sent for an active learning example.
    def send_learn_example(self, msg_str):
        msg = Message(self._t_layer._id, self._cpu_id, 'learn',
                      self._msg_state, msg_str)
        self.send_message(msg)

    def recv_learn_example(self, message):
        self._msg_state = message.type

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
                # Check for train or test mode.
                if self._train_mode == True:
                    if self._msg_state == act_pass['passive']:
                        self.send_add_example(user_input)
                    else:
                        self.send_learn_example(user_input)
                # Test Mode.
                else:
                    self.send_test_case(user_input)

if __name__ == '__main__':
    mode = sys.argv[1]
    robot_input = InputNode(1, 'robot_input', '/robot/messages', mode)
    #robot_input.start()
    robot_input.send_input()

