#!/usr/bin/env python
PKG = 'openag_brain'
NAME = 'test_image_analyzer'

import unittest
import tempfile
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from openag_cv.detect_size import process_image

def get_temp_file_name():
    return tempfile.gettempdir() + '/' + next(tempfile._get_candidate_names()) + ".jpg"

class TestUSBCam(unittest.TestCase):
    """
    Test the imaeg analyzer node. It should read an image from usb_cam and
    process it through the src/openag_cv modules.
    """
    image_format_mapping = {
        "rgb8": "RGB",
        "rgba8": "RGBA"
    }
    bridge = CvBridge()

    def setUp(self):
        self.namespace = "usb_cam"
        self.image_received = False

    def callback(self, item):
        rospy.loginfo("===================Image Received===================")
        self.image_received = True
        rospy.loginfo("Image Received {} {}".format(item.width, item.height))
        image_format = self.image_format_mapping.get(item.encoding, None)
        if image_format is None:
            raise ValueError()
        cv2_img = self.bridge.imgmsg_to_cv2(item, "bgr8")
        #np_arr = np.fromstring(item.data, np.uint8)
        #image_np = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        file_name = get_temp_file_name()
        rospy.loginfo("Saving image to {}".format(file_name))
        cv2.imwrite(file_name, cv2_img)
        # rospy.loginfo("Image saved to {}".format(file_name))
        rospy.loginfo("width: {} height: {}".format(item.width, item.height))
        self.assertTrue(item.width == 640)
        self.assertTrue(item.height == 480)
        self.image_processed = True

    def test_usb_cam(self):
        self.image_received = False
        rospy.init_node(NAME, anonymous=True)
        #camera_source = "test_image"
        image_rostopic = "{}/image_raw".format(self.namespace)
        self.sub_measured = rospy.Subscriber(image_rostopic, Image, self.callback)
        rospy.loginfo("Looking for image at " + image_rostopic)
        for _ in range(3, 0, -1):
           rospy.loginfo("Waiting {} seconds to receive image.".format(_))
           rospy.sleep(1)  # Wait for subscriber to be ready to receive messages
        # for msg in self.msgs:
        #     rospy.loginfo("Published: {:f}".format(msg))
        #     self.pub_desired.publish(msg)
        # rospy.sleep(15)  # Wait until all messages are received.
        #rospy.spin()
        self.assertTrue(self.image_received)
        self.assertTrue(self.image_processed)


class TestProcessImage(unittest.TestCase):
    """
    Test the imaeg analyzer node. It should read an image from usb_cam and
    process it through the src/openag_cv modules.
    """
    image_format_mapping = {
        "rgb8": "RGB",
        "rgba8": "RGBA"
    }
    bridge = CvBridge()

    def setUp(self):
        self.namespace = "usb_cam"
        self.image_received = False

    def callback(self, item):
        rospy.loginfo("===================Image Received for Process Image===================")
        self.image_received = True
        rospy.loginfo("Image Received {} {}".format(item.width, item.height))
        image_format = self.image_format_mapping.get(item.encoding, None)
        if image_format is None:
            raise ValueError()
        cv2_img = self.bridge.imgmsg_to_cv2(item, "bgr8")

        processed_image = process_image(cv2_img)
        file_name = get_temp_file_name()
        rospy.loginfo("Saving processed image to {}".format(file_name))
        cv2.imwrite(file_name, processed_image)
        rospy.loginfo("width: {} height: {}".format(item.width, item.height))
        self.assertTrue(item.width == 640)
        self.assertTrue(item.height == 480)
        self.image_processed = True

    def test_image_analyzer(self):
        self.image_received = False
        rospy.init_node(NAME, anonymous=True)
        #camera_source = "test_image"
        image_rostopic = "{}/image_raw".format(self.namespace)
        self.sub_measured = rospy.Subscriber(image_rostopic, Image, self.callback)
        rospy.loginfo("Looking for image at " + image_rostopic)
        for _ in range(3, 0, -1):
           rospy.loginfo("Waiting {} seconds to receive image.".format(_))
           rospy.sleep(1)  # Wait for subscriber to be ready to receive messages
        # for msg in self.msgs:
        #     rospy.loginfo("Published: {:f}".format(msg))
        #     self.pub_desired.publish(msg)
        # rospy.sleep(15)  # Wait until all messages are received.
        #rospy.spin()
        self.assertTrue(self.image_received)
        self.assertTrue(self.image_processed)

if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, NAME, TestProcessImage)
