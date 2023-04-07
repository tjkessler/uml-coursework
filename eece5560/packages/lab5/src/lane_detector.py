#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from duckietown_msgs.msg import Segment, SegmentList
from cv_bridge import CvBridge
import numpy as np

_IMG_SIZE = (160, 120)
_OFFSET = 40


class LaneDetector:

    def __init__(self):

        self._bridge = CvBridge()
        self._pub = rospy.Publisher('~gpn_seg_list', SegmentList, queue_size=1)
        self._pub_img = rospy.Publisher('~seg_img', Image, queue_size=1)
        self._pub_y = rospy.Publisher('~img_y', Image, queue_size=1)
        self._pub_w = rospy.Publisher('~img_w', Image, queue_size=1)
        rospy.Subscriber('~input_img', CompressedImage, self.callback,
                         queue_size=1, buff_size=2**24)

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

    def callback(self, img: CompressedImage):

        rospy.logwarn('Travis Kessler\'s lab5 code')

        # ros -> cv2, crop
        cv_img = self._bridge.compressed_imgmsg_to_cv2(img, 'bgr8')
        cropped_img = cv2.resize(cv_img, _IMG_SIZE,
                                 interpolation=cv2.INTER_NEAREST)
        cropped_img = cropped_img[_OFFSET:, :]

        # perform necessary pre-processing
        img_hsv = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
        kernel_erode_y = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        kernel_erode_w = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        kernel_dilate_y = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        kernel_dilate_w = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8))
        img_hsv_y = cv2.erode(img_hsv, kernel_erode_y)
        img_hsv_y = cv2.dilate(img_hsv_y, kernel_dilate_y)
        img_hsv_w = cv2.erode(img_hsv, kernel_erode_w)
        img_hsv_w = cv2.dilate(img_hsv_w, kernel_dilate_w)
        mask_yellow = cv2.inRange(img_hsv_y, (15, 100, 100), (55, 255, 255))
        mask_white = cv2.inRange(img_hsv_w, (0, 0, 200), (180, 25, 255))

        # obtain white and yellow images
        img_yellow = cv2.bitwise_and(cropped_img, cropped_img,
                                     mask=mask_yellow)
        img_white = cv2.bitwise_and(cropped_img, cropped_img,
                                    mask=mask_white)
        self._pub_y.publish(self._bridge.cv2_to_imgmsg(img_yellow, 'bgr8'))
        self._pub_w.publish(self._bridge.cv2_to_imgmsg(img_white, 'bgr8'))

        # detect edges
        edges = cv2.Canny(cropped_img, 100, 200)

        # filter yellow lines
        comb_y = cv2.bitwise_and(
            edges, cv2.cvtColor(img_yellow, cv2.COLOR_BGR2GRAY)
        )
        lines_y = cv2.HoughLinesP(comb_y, 1, np.pi/180, 15)
        output_y = self._output_lines(cropped_img, lines_y)

        # filter white lines
        comb_w = cv2.bitwise_and(
            edges, cv2.cvtColor(img_white, cv2.COLOR_BGR2GRAY)
        )
        lines_w = cv2.HoughLinesP(comb_w, 1, np.pi/180, 15)
        output_w = self._output_lines(cropped_img, lines_w)

        # combine filtered/lined images for viewing
        combined = cv2.bitwise_or(output_y, output_w)
        self._pub_img.publish(self._bridge.cv2_to_imgmsg(combined, 'bgr8'))

        if lines_y is None or lines_w is None\
           or len(lines_y) == 0 or len(lines_w) == 0:
            return

        # normalize line values
        arr_cutoff = np.array([0, _OFFSET, 0, _OFFSET])
        arr_ratio = np.array([
            1.0 / _IMG_SIZE[1],
            1.0 / _IMG_SIZE[0],
            1.0 / _IMG_SIZE[1],
            1.0 / _IMG_SIZE[0]
        ])
        norm_lines_y = []
        for i in range(len(lines_y)):
            ln = lines_y[i][0]
            norm_lines_y.append((ln + arr_cutoff) * arr_ratio)
        norm_lines_w = []
        for i in range(len(lines_w)):
            ln = lines_w[i][0]
            norm_lines_y.append((ln + arr_cutoff) * arr_ratio)

        # create msg
        seglist = SegmentList()
        seglist.header.stamp = img.header.stamp
        for line in norm_lines_y:
            new_seg = Segment()
            new_seg.color = Segment.YELLOW
            new_seg.pixels_normalized[0].x = line[0]
            new_seg.pixels_normalized[0].y = line[1]
            new_seg.pixels_normalized[1].x = line[2]
            new_seg.pixels_normalized[1].y = line[3]
            new_seg.normal.x = 0
            new_seg.normal.y = 0
            seglist.segments.append(new_seg)
        for line in norm_lines_w:
            new_seg = Segment()
            new_seg.color = Segment.WHITE
            new_seg.pixels_normalized[0].x = line[0]
            new_seg.pixels_normalized[0].y = line[1]
            new_seg.pixels_normalized[1].x = line[2]
            new_seg.pixels_normalized[1].y = line[3]
            new_seg.normal.x = 0
            new_seg.normal.y = 0
            seglist.segments.append(new_seg)

        # publish
        self._pub.publish(seglist)


if __name__ == '__main__':

    rospy.init_node('lab5_detector', anonymous=True)
    detector = LaneDetector()
    rospy.spin()
