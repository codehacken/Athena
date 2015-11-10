#!/usr/bin/env python
"""
This is the script that implements the robot's CPU which contains multiple parts.
1. The ImageConverter take inputs from a Kinect Sensor and convert them.
2. Take input from the keyboard.
AUTHOR: Ashwinkumar Ganesan.
"""

# ROS.
import rospy
from lib.image import ImageConverter
from lib.cpu import CPU

def listener():
    ic = ImageConverter()
    cpu = CPU(ic)
    rospy.init_node('robot_cpu', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    """
    Write all the initialization code here.
    This includes the model, the examples.
    Once the initializations are complete, then robot goes online.
    """
    listener()
