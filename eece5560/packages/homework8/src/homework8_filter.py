#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageFilterer:

    def __init__(self):

        self._bridge = CvBridge()
        self._pub_crop = rospy.Publisher('image_cropped', Image, queue_size=10)
        self._pub_y = rospy.Publisher('image_yellow', Image, queue_size=10)
        self._pub_w = rospy.Publisher('image_white', Image, queue_size=10)
        rospy.Subscriber('image', Image, self.callback)

    def callback(self, msg: Image):

        cv_img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Get image height
        h, w, _ = cv_img.shape

        # Crop image, publish it
        img_cropped = cv_img[int(h / 2): h, 0: w]
        self._pub_crop.publish(self._bridge.cv2_to_imgmsg(img_cropped, 'bgr8'))

        # Perform necessary pre-processing
        img_hsv = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2HSV)
        kernel_erode_y = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        kernel_erode_w = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        kernel_dilate_y = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        kernel_dilate_w = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8))
        img_hsv_y = cv2.erode(img_hsv, kernel_erode_y)
        img_hsv_y = cv2.dilate(img_hsv_y, kernel_dilate_y)
        img_hsv_y = cv2.erode(img_hsv_y, kernel_erode_y)
        img_hsv_y = cv2.dilate(img_hsv_y, kernel_dilate_y)
        img_hsv_y = cv2.erode(img_hsv_y, kernel_erode_y)
        img_hsv_w = cv2.erode(img_hsv, kernel_erode_w)
        img_hsv_w = cv2.dilate(img_hsv_w, kernel_dilate_w)
        img_hsv_w = cv2.erode(img_hsv_w, kernel_erode_w)
        img_hsv_w = cv2.dilate(img_hsv_w, kernel_dilate_w)
        mask_yellow = cv2.inRange(img_hsv_y, (20, 100, 100), (30, 255, 255))
        mask_white = cv2.inRange(img_hsv_w, (0, 0, 230), (180, 25, 255))

        # Obtain white and yellow images, publish them
        img_yellow = cv2.bitwise_and(img_cropped, img_cropped, mask=mask_yellow)
        img_white = cv2.bitwise_and(img_cropped, img_cropped, mask=mask_white)
        self._pub_y.publish(self._bridge.cv2_to_imgmsg(img_yellow, 'bgr8'))
        self._pub_w.publish(self._bridge.cv2_to_imgmsg(img_white, 'bgr8'))


if __name__ == '__main__':

    rospy.init_node('homework8_filter', anonymous=True)
    img_conv = ImageFilterer()
    rospy.spin()
