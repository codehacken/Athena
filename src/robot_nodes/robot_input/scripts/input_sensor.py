#!/usr/bin/env python
# AUTHOR: Ashwinkumar Ganesan.
# This is the ROS Node which acts the robots inout sensor. The input
# is specifically from a keyboard given through stdin.

import rospy
from std_msgs.msg import String

class ImageConverter:
    def __init__(self):
        self.image_pub = rospy.Publisher("/image", Image, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect2/hd/image_color_rect", Image, self.callback)

    def callback(self, image):
        self.image_pub.publish(image)


def listener():
    ic = ImageConverter()
    rospy.init_node('kinect_sensor', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    listener()

