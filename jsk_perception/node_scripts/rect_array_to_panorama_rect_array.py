#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import PanoramaInfo, PanoramaRect, PanoramaRectArray
from jsk_recognition_msgs.msg import RectArray

import sys

def transformPanoramaPoint(x,
                           y,
                           image_height,
                           image_width,
                           theta_min,
                           theta_max,
                           phi_min,
                           phi_max):
    phi = (phi_max + phi_min) / 2.0 + \
            1.0 * (phi_max - phi_min) * (x - (image_width/2.0)) / image_width
    theta = theta_min + 1.0 * (theta_max - theta_min) * y / image_height
    return (theta, phi)


class RectArrayToPanoramaRectArray(object):

    def __init__(self):

        try:
            msg_panorama_image = rospy.wait_for_message(
                '~input_panorama_image', Image, 10)
            msg_panorama_info = rospy.wait_for_message(
                '~input_panorama_info', PanoramaInfo, 10)
        except (rospy.ROSException, rospy.ROSInterruptException) as e:
            rospy.logerr('{}'.format(e))
            sys.exit(1)
        self._frame_id_panorama = msg_panorama_info.header.frame_id
        self._theta_min = msg_panorama_info.theta_min
        self._theta_max = msg_panorama_info.theta_max
        self._phi_min = msg_panorama_info.phi_min
        self._phi_max = msg_panorama_info.phi_max
        self._image_height = msg_panorama_image.height
        self._image_width = msg_panorama_image.width

        # Publisher
        self._pub = rospy.Publisher(
            '~output', PanoramaRectArray, queue_size=10)
        # Subscribers
        self._sub = rospy.Subscriber(
            '~input', RectArray, self._cb)

        rospy.loginfo('initialization has finished.')

    def _cb(self, msg_rects):

        rospy.loginfo('callback called')

        msg_array = PanoramaRectArray()
        msg_array.header.frame_id = self._frame_id_panorama
        msg_array.header.stamp = rospy.Time.now()

        for rect in msg_rects.rects:
            (theta_a, phi_a) = transformPanoramaPoint(
                rect.x,
                rect.y,
                self._image_height,
                self._image_width,
                self._theta_min,
                self._theta_max,
                self._phi_min,
                self._phi_max
            )
            (theta_b, phi_b) = transformPanoramaPoint(
                rect.x + rect.width,
                rect.y + rect.height,
                self._image_height,
                self._image_width,
                self._theta_min,
                self._theta_max,
                self._phi_min,
                self._phi_max
            )

            msg = PanoramaRect()
            msg.theta = theta_a
            msg.phi = phi_a
            msg.height = theta_b - theta_a
            msg.width = phi_b - phi_a

            msg_array.panorama_rects.append(msg)

        self._pub.publish(msg_array)


def main():

    rospy.init_node('rect_array_to_panorama_rect_array')
    node = RectArrayToPanoramaRectArray()
    rospy.spin()


if __name__ == '__main__':
    main()
