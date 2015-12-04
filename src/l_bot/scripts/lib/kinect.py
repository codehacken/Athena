"""
The ImageConverter take inputs from a Kinect Sensor and convert them.
AUTHOR: Ashwinkumar Ganesan.
"""

__author__ = 'Ashwinkumar Ganesan'

# ROS.
import rospy
from sensor_msgs.msg import Image

# Libs for CV.
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter:
    _queue_size = 10

    def __init__(self):
        # This is the subscriber to the image.
        self._image_sub = rospy.Subscriber("/kinect2/hd/image_color_rect", Image, self._send_image)
        #self._image_sub = rospy.Subscriber("/kinect2/hd/image_depth_rect", Image, self._send_image)

        # This is the publisher to publish the image to image package, so that it can be viewed.
        self._image_pub = rospy.Publisher("/image", Image, queue_size=ImageConverter._queue_size)

        # Using CV Bridge to process RGB data from Kinect2.
        self._bridge = CvBridge()

        # Image
        self._image = None
        self._raw_image = None # Contains the most updated raw image


    """
    Convert the raw image from Kinect2 to a cv_image and send it back.
    """
    def get_image(self):
        if self._raw_image == None:
            print("Image not received from Kinect")
            return None
        try:
            cv_image = self._bridge.imgmsg_to_cv2(self._raw_image, "bgr8")
            self._image = cv_image
        except CvBridgeError, e:
            print e

        return self._image

    # Define the list of callback functions for subscribers.
    def _send_image(self, image):
        self._raw_image = image
        self._image_pub.publish(self._raw_image)
