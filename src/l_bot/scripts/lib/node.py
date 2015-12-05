#!/usr/bin/env python
"""
This is the basic structure of a ROS Node. Inherit its functions.
AUTHOR: Ashwinkumar Ganesan.
"""
__author__ = 'Ashwinkumar Ganesan'

import rospy
from lib.transport import TLayer, Message
import sys
from abc import ABCMeta, abstractmethod

# Constants.
mode = {'active': 0, 'passive': 7}

"""
_testing_folder_name = "data/people_testing_data"
_training_folder_name = "data/people_training_data"
"""

"""
This is the basic Node.
Inherit the class for creating new sensor nodes.
"""
class RobotNode(object):
    __metaclass__ = ABCMeta

    def __init__(self, ID, node_name, topic_name):
        # Create a transport layer to pass messages.
        self.name = node_name
        self._t_layer = TLayer(ID, topic_name, self._process_message)

        ############################################################
        # Map the commands RECEIVED, types with their actions.
        self._recv_command_map = {'train': {0: 'recv_train'},
                                  'test':{0: 'recv_test',
                                          1: 'recv_test_case'},
                                  'exit':{0: 'recv_exit'},
                                  'learn':{0: 'recv_learn_example',
                                           1: 'recv_learn_example',
                                           2: 'recv_learn_example',
                                           3: 'recv_learn_example',
                                           4: 'recv_learn_example',
                                           5: 'recv_learn_example',
                                           6: 'recv_learn_example',
                                           7: 'recv_add_example'},
                                  'print':{0: 'recv_print_message'}
        }
        ############################################################

        # Check if the node is in training mode or test mode.
        self._train_mode = True

        # Initiate the Node.
        rospy.init_node(self.name, anonymous=True)

        """
        # File writing module.
        self._rollover_cnt = 0
        self._rollover_limit = 20

        self.test_filename = "person" + str(self._rollover_cnt) + "_statements.txt"
        self.training_filename = ""

        # File Handles.
        self._test_handle = None
        self._train_handle = None
        """

    # Execute commands according to map defined.
    def _process_message(self, message):
        # Extract the command and the command-type.
        if message.command not in self._recv_command_map:
            print("Incorrect Command: " + message.command)

        if message.type not in self._recv_command_map[message.command]:
            print("Incorrect Command Type: " + message.type)

        getattr(self, self._recv_command_map[message.command][message.type])(message)

    """
    Function to send a message to other nodes.
    def send_message(self, message):
        self._t_layer.send_message(message)
    """
    def send_message(self, message):
        self._t_layer.send_message(message)

    # Start the ROS Node.
    def start(self):
        rospy.spin()

    # Functions to send messages for various commands.
    def send_train(self):
        # Change Node to training mode before sending a message.
        self._train_mode = True
        print("Node Training Mode: On")

        # -1 because this is a broadcast message.
        msg = Message(self._t_layer._id, -1, 'train', 0, "")
        self.send_message(msg)

    def send_test(self):
        # Change Node to testing mode before sending a message.
        self._train_mode = False
        print("Node Testing Mode: On")

        # -1 because this is a broadcast message.
        msg = Message(self._t_layer._id, -1, 'test', 0, "")
        self.send_message(msg)

    """
    The message to be printed is sent to a specific node or
    all nodes using broadcast. (USE: node_id)
    """
    def send_print_message(self, node_id, msg_str):
        # -1 because this is a broadcast message.
        msg = Message(self._t_layer._id, node_id, 'print', 0, msg_str)
        self.send_message(msg)

    # Command to exit all Nodes.
    # Use carefully.
    def send_exit(self):
        msg = Message(self._t_layer._id, -1, 'exit', 0, "")
        self.send_message(msg)
        print("Node Exiting.....")
        sys.exit()

    ####################################################################
    # SET of send functions whose implementation is left to the specific
    # node. These are just interfaces.

    def send_add_example(self):
        pass

    def send_test_case(self):
        pass

    def send_learn_example(self):
        pass
    ####################################################################
    # Functions to perform operations when commands are received.
    def recv_train(self, message):
        print("Node Training Mode: On")
        self._train_mode = True

    def recv_test(self, message):
        print("Node Testing Mode: On")
        self._train_mode = False

    def recv_exit(self, message):
        print("Node Exiting.....")
        sys.exit()

    def recv_print_message(self, message):
        print(message.message)

    ####################################################################
    # SET of recv functions whose implementation is left to the specific
    # node. These are just interfaces.

    # Receive a message to add a image snapshot and associate a text
    # message with it.
    def recv_add_example(self, message):
        pass

    def recv_test_case(self, message):
        pass

    def recv_learn_example(self, message):
        pass
