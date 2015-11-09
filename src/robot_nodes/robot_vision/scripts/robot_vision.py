#!/usr/bin/env python
# AUTHOR: Ashwinkumar Ganesan.
# This is the script that implements the robot's vision using,
# a Kinect2 sensor.

import roslib
import sys
import cv2

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter:
    def __init__(self):
        # This is the subscriber to the image and subscriber to the input on when to process,
        # the image.
        self.image_sub = rospy.Subscriber("/kinect2/hd/image_color_rect", Image, self.callback)
        self.image_process_sub = rospy.Subscriber("/robot/input_flag", String, self.if_process)

        # This is the publisher to publish the image to image package, so that it can be viewed.
        self.image_pub = rospy.Publisher("/image", Image, queue_size=10)

        # This is a publisher to publish the processed features to the CPU (ROS Node)
        self.image_process_pub = rospy.Publisher("/robot/processed_image", Image, queue_size=10)

        # Using CV Bridge to process RGB data from Kinect2.
        self.bridge = CvBridge()
        self.process_flag = False

    def callback(self, image):
        self.image_pub.publish(image)

        if self.process_flag == True:
            features = self.process()
            self.image_process_sub.publish(features)

            # Reset the flag, so that no more frames are processed.
            self.process_flag = False

    def if_process(self):
        self.process_flag = True

    # The process function implements OpenCV processing and generates the output.
    def process(self):
        features = ""
        return features

def listener():
    ic = ImageConverter()
    rospy.init_node('kinect_sensor', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    listener()

