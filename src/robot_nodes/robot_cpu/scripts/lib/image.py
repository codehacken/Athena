"""
The ImageConverter take inputs from a Kinect Sensor and convert them.
AUTHOR: Ashwinkumar Ganesan.
"""

# Standard Libraries.
import sys

# ROS.
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Int32

# Libs for CV.
import cv2
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter:
    def __init__(self):
        # List of commands the Converter takes and its state values.
        self._commands = {1:self._take_snapshot}
        self._send_cmds = {'do_process':2}
        self._state = {'image': False}

        # This is the subscriber to the image.
        self._image_sub = rospy.Subscriber("/kinect2/hd/image_color_rect", Image, self._send_image)

        # This is the publisher to publish the image to image package, so that it can be viewed.
        self._image_pub = rospy.Publisher("/image", Image, queue_size=10)

        # The Image converter also has a subscriber for commands.
        self._image_sub = rospy.Subscriber("/robot/commands", Int32, self._exec_command)

        # Using CV Bridge to process RGB data from Kinect2.
        self._bridge = CvBridge()

        # Image
        self._image = None

        # Command Pub.
        self._cmd_pub = rospy.Publisher("/robot/commands", Int32, queue_size=10)

    # Define a list of helper functions.
    # The State value is a string.
    def _set_state(self, state_value):
        self._state[state_value] = True

    def _reset_state(self, state_value):
        self._state[state_value] = False

    def _exec_command(self, command):
        try:
            self._commands[command.data]()
        except KeyError:
            pass

    def get_image(self):
        return self._image

    # Define a list of command functions.
    def _take_snapshot(self):
        self._set_state('image')

    # Define the list of callback functions for subscribers.
    def _send_image(self, image):
        # self._image_pub.publish(image)
        # Check commands given for snapshots.
        if(self._state['image'] == True):
            try:
                 cv_image = self._bridge.imgmsg_to_cv2(image, "bgr8")
            except CvBridgeError, e:
                 print e

            # Process the image.
            self._image = cv_image

            # Send command that the snapshot has been taken.
            self._cmd_pub.publish(self._send_cmds['do_process'])

            # Reset the state.
            self._reset_state('image')

            print("Snapshot Given!")
