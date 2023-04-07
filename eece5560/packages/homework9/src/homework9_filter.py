#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class ImageFilterer:

    def __init__(self):

        self._bridge = CvBridge()

        self._detected_edges = None
        self._cropped_img = None
        self._white_lines = None
        self._yellow_lines = None

        rospy.Subscriber('/image_cropped', Image, self._filter_cropped)
        rospy.Subscriber('/image_yellow', Image, self._filter_yellow)
        rospy.Subscriber('/image_white', Image, self._filter_white)

        self._pub_yellow = rospy.Publisher(
            'image_lines_yellow', Image, queue_size=10
        )
        self._pub_white = rospy.Publisher(
            'image_lines_white', Image, queue_size=10
        )
        self._pub_all = rospy.Publisher(
            'image_lines_all', Image, queue_size=10
        )

    @staticmethod
    def _output_lines(original_img, lines):

        output = np.copy(original_img)
        if lines is not None:
            for i in range(len(lines)):
                ln = lines[i][0]
                cv2.line(output, (ln[0], ln[1]), (ln[2], ln[3]), (255, 0, 0),
                         2, cv2.LINE_AA)
                cv2.circle(output, (ln[0], ln[1]), 2, (0, 255, 0))
                cv2.circle(output, (ln[2], ln[3]), 2, (0, 0, 255))
        return output

    def _filter_cropped(self, msg: Image):

        cv_img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        self._detected_edges = cv2.Canny(cv_img, 100, 200)
        self._cropped_img = cv_img
        return

    def _filter_yellow(self, msg: Image):

        if self._detected_edges is None:
            return

        cv_img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        combined = cv2.bitwise_and(self._detected_edges, cv2.cvtColor(
            cv_img, cv2.COLOR_BGR2GRAY
        ))
        lines = cv2.HoughLinesP(combined, 1, np.pi/180, 10)
        output = self._output_lines(self._cropped_img, lines)
        self._yellow_lines = output
        self._pub_yellow.publish(self._bridge.cv2_to_imgmsg(output, 'bgr8'))
        self._combined()

    def _filter_white(self, msg: Image):

        if self._detected_edges is None:
            return

        cv_img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        combined = cv2.bitwise_and(self._detected_edges, cv2.cvtColor(
            cv_img, cv2.COLOR_BGR2GRAY
        ))
        lines = cv2.HoughLinesP(combined, 1, np.pi/180, 10)
        output = self._output_lines(self._cropped_img, lines)
        self._white_lines = output
        self._pub_white.publish(self._bridge.cv2_to_imgmsg(output, 'bgr8'))
        self._combined()

    def _combined(self):

        if self._yellow_lines is None or self._white_lines is None:
            return

        combined = cv2.bitwise_or(self._yellow_lines, self._white_lines)
        self._pub_all.publish(self._bridge.cv2_to_imgmsg(combined, 'bgr8'))


if __name__ == '__main__':

    rospy.init_node('homework9_filter', anonymous=True)
    img_conv = ImageFilterer()
    rospy.spin()
