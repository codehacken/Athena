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

        ##########################################################
        # Map the commands RECEIVED, types with their actions.
        self._recv_command_map = {'train': {0: 'recv_train'},
                                  'test':{0: 'recv_test'},
                                  'exit':{0: 'recv_exit'},
                                  'learn':{6: 'recv_add_example'}
        }
        ##########################################################

        # Check if the node is in training mode or test mode.
        self._train_mode = True

        # Initiate the Node.
        rospy.init_node(self.name, anonymous=True)

    # Execute commands according to map defined.
    def _process_message(self, message):
        # Extract the command and the command-type.
        if message.cmd not in self._recv_command_map:
            print("Incorrect Command: " + message.cmd)

        if message.type not in self._recv_command_map[message.cmd]:
            print("Incorrect Command Type: " + message.type)

        getattr(self, self._recv_command_map[message.cmd][message.type])()

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

    ####################################################################
    # Functions to perform operations when commands are received.
    def recv_train(self):
        self._train_mode = True

    def recv_test(self):
        self._train_mode = False

    def recv_exit(self):
        sys.exit()

    ####################################################################
    # SET of recv functions whose implementation is left to the specific
    # node. These are just interfaces.

    # Receive a message to add a image snapshot and associate a text
    # message with it.
    def recv_add_example(self):
        pass
