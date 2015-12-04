#!/usr/bin/env python
"""
This is the CPU Node.
AUTHOR: Ashwinkumar Ganesan.
"""

__author__ = 'Ashwinkumar Ganesan'

# ROS.
from process import add_example, test_example
from lib.framework import JointModel
from lib.node import RobotNode

class CpuNode(RobotNode):
    _input_node_id = 1
    _output_node_id = 3

    # To initialize a CPU without a precomputed joint model.
    def __init__(self, ID, node_name, topic_name, ic, joint_model=None):
        super(CpuNode, self).__init__(ID, node_name, topic_name)
        # ic is the image converter.
        self._ic = ic

        if joint_model == None:
            self._joint_model = JointModel()

    def send_print_message_to_op(self, msg_str):
        self.send_print_message(CpuNode._output_node_id, msg_str)

    """
    Receive a message from the user describing the object that
    is placed in front on the robot.
    """
    def recv_add_example(self, message):
        # This is the passive learning implementation.
        # Associate the sentence with the image.
        if self._train_mode == True:
            add_example(self._ic.get_image(), message.message, self._joint_model,
                        self.send_print_message_to_op)
        else:
            msg_str = "Robot is in test mode, switch to training mode to add more examples."
            self.send_print_message_to_op(msg_str)

    def recv_test_case(self, message):
        # Perform a test on the image.
        # The input string from the user is not used.
        if self._train_mode == False:
            test_example(self._ic.get_image(), message.message, self._joint_model,
                         self.send_print_message_to_op)
        else:
            msg_str = "Robot is in training mode, switch to test mode to validate."
            self.send_print_message_to_op(msg_str)
