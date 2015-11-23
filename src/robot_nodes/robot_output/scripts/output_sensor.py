#!/usr/bin/env python
"""
This is the ROS Node which acts the Robots output.
AUTHOR: Ashwinkumar Ganesan.
"""

import rospy
from std_msgs.msg import String

class RobotTalk:
    def __init__(self):
        # This is the node for the robot output.
        # The current output is shown on the console.
        self._output_sub = rospy.Subscriber("/robot/talk", String, self._show_output)

    # Print Output
    def _show_output(self, message):
        print(message)

def listener():
    # Initiate the listener.
    rt = RobotTalk()
    rospy.init_node('robot_output', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    listener()
