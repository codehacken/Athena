#!/usr/bin/env python
"""
This is the CPU Node.
AUTHOR: Ashwinkumar Ganesan.
"""

__author__ = 'Ashwinkumar Ganesan'

# ROS.
from process import add_example, test_example, learn_example
from lib.framework import JointModel
from lib.node import RobotNode
from lib.transport import Message
from lib.alframework import ALUniRobotDrivenModel

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
        else:
            self._joint_model = joint_model

        # Depending on the initialization, the joint model may be empty
        # or not. Initialize the ALFramework here.
        self.al_framework = ALUniRobotDrivenModel(self._joint_model)
        self._examples_added = 1

    def send_print_message_to_op(self, msg_str):
        self.send_print_message(CpuNode._output_node_id, msg_str)

    # This is the message that is sent for an active learning example.
    def send_learn_example(self, msg_id, msg_str):
        msg = Message(self._t_layer._id, -1, 'learn',
                      msg_id, msg_str)
        self.send_message(msg)

    """
    Receive a message from the user describing the object that
    is placed in front on the robot.
    """
    def recv_add_example(self, message):
        # This is the passive learning implementation.
        # Associate the sentence with the image.
        if self._train_mode == True:
            add_example(self._ic.get_image(), message.message, self._joint_model,
                        self.send_print_message_to_op, self._examples_added)
            self._examples_added += 1
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

    def recv_learn_example(self, message):
        # This is the passive learning implementation.
        # Associate the sentence with the image.
        if self._train_mode == True:
            learn_example(self._ic.get_image(), message.message, self.al_framework,
                          self.send_print_message_to_op, self.send_learn_example)
        else:
            msg_str = "Robot is in test mode, switch to training mode to add more examples."
            self.send_print_message_to_op(msg_str)
