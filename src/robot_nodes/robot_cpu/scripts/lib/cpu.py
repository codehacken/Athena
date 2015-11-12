#!/usr/bin/env python
"""
This is the CPU Node.
AUTHOR: Ashwinkumar Ganesan.
"""

# ROS.
import rospy
from std_msgs.msg import Int32, String
from process import process_model
from lib.framework import JointModel

class CPU:
    def __init__(self, ic):
        # List of commands the CPU takes and its state values.
        self._commands = {2:self._process_model}

        # ic is the image converter.
        self._ic = ic

        # This is the subscriber to the commands.
        self._image_sub = rospy.Subscriber("/robot/commands", Int32, self._do_process)

        # Get Keyboard inputs.
        self._input_sub = rospy.Subscriber("/robot/messages", String, self._store_message)
        self._message = ""
        self._joint_model = JointModel()

    # Helper functions.
    def _do_process(self, command):
        try:
            self._commands[command.data]()
        except KeyError:
            pass

    def _store_message(self, message):
        self._message = message.data

    def _process_model(self):
        # Given the function and input of image and keyboard.
        process_model(self._ic.get_image(), self._message, self._joint_model)

