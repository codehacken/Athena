#!/usr/bin/env python
"""
This is the script that implements the robot's CPU which contains multiple parts.
1. The ImageConverter take inputs from a Kinect Sensor and convert them.
2. Take input from the keyboard.
AUTHOR: Ashwinkumar Ganesan.
"""

# ROS.
import rospy
from lib.kinect import ImageConverter
from lib.cpu import CPU
from lib.process import initialize_model

import sys

def listener(joint_model):
    ic = ImageConverter()
    cpu = CPU(ic, joint_model)
    rospy.init_node('robot_cpu', anonymous=True)
    rospy.spin()

if __name__ == '__main__':

    """
    Write all the initialization code here.
    This includes the model, the examples.
    Once the initializations are complete, then robot goes online.
    """
    # The default condition is to load the join model.
    if len(sys.argv) == 0:
        # This is a precomputed joint model.
        print("I am here 1")
        joint_model = initialize_model()
    elif len(sys.argv) == 1:
        print("I am here 2")
        if(sys.argv[1] == "reset"):
            joint_model = None
    else:
        print("I am here 3")
        print("Too many arguments.")

    listener(joint_model)

