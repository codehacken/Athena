#!/usr/bin/env python
"""
This is the script that implements the robot's CPU which contains multiple parts.
1. The ImageConverter take inputs from a Kinect Sensor and convert them.
2. Take input from the keyboard.
AUTHOR: Ashwinkumar Ganesan.
"""
__author__ = 'Ashwinkumar Ganesan'


# ROS.
import rospy
from lib.kinect import ImageConverter
from lib.cpu import CpuNode
from lib.process import initialize_model

def listener(joint_model):
    ic = ImageConverter()
    cpu = CpuNode(2, 'robot_cpu', '/robot/messages', ic, joint_model)
    #rospy.init_node('robot_cpu', anonymous=True)
    #rospy.spin()
    cpu.start()

if __name__ == '__main__':

    """
    Write all the initialization code here.
    This includes the model, the examples.
    Once the initializations are complete, then robot goes online.
    """
    # This is a precomputed joint model.
    joint_model = initialize_model()
    listener(joint_model)
