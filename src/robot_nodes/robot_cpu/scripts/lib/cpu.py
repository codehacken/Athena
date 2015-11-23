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
    # To initialize a CPU without a precomputed joint model.
    def __init__(self, ic, joint_model=None):
        # List of commands the CPU takes and its state values.
        self._commands = {2:self._process_model}

        # ic is the image converter.
        self._ic = ic

        # This is the subscriber to the commands.
        self._image_sub = rospy.Subscriber("/robot/commands", Int32, self._do_process)

        # Get Keyboard inputs.
        self._input_sub = rospy.Subscriber("/robot/messages", String, self._store_message)
        self._message = ""

        if joint_model == None:
            self._joint_model = JointModel()

        self._output_pub = rospy.Publisher("/robot/talk", String, queue_size=10)

    # Helper functions.
    def _do_process(self, command):
        try:
            self._commands[command.data]()
        except KeyError:
            pass

    def _store_message(self, message):
        self._message = message.data

    def _publish_message(self, message):
        self._output_pub.publish(message)

    def _process_model(self):
        # Given the function and input of image and keyboard.
        process_model(self._ic.get_image(), self._message, self._joint_model, self._publish_message)
