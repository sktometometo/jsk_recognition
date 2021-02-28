#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import message_filters

import tf2_ros
import tf2_geometry_msgs

import math

from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from jsk_recognition_msgs.msg import PanoramaRectArray, ClassificationResult
from geometry_msgs.msg import PointStamped

def calcPoint( theta, phi, r ):
    return ( r * math.sin(theta) * math.cos(phi),
             r * math.sin(theta) * math.cos(phi),
             r * cos(phi) )

class PanoramaRectArrayToBoundingBoxArray(object):

    def __init__(self):

        self._frame_fixed = rospy.get_param( '~frame_fixed', 'fixed_frame' )
        self._dimensions = rospy.get_param( '~dimensions', '{}' )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        mf_sub_class = message_filters.Subscriber( '~input_class', ClassificationResult )
        mf_sub_panorama_rects = message_filters.Subsciber( '~input_panorama_rects', PanoramaRectArray )
        self._pub = rospy.Publisher( '~output', BoundingBoxArray, queue_size=1 )

        tf = message_filters.TimeSynchronizer([mf_sub_class, mf_sub_panorama_rects], 10)
        tf.registerCallback(self._cb)

    def _cb(self,
            msg_class,
            msg_panorama_rects):

        try:
            transform_fixed_to_panorama = self._tf_buffer.lookup_transform_full(
                    msg_panorama_rects.header.frame_id,
                    self._frame_fixed.header.frame_id,
                    rospy.Time()
                    )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('{}'.format(e))
            return

        msg_pub = BoundingBoxArray()
        msg_pub.header.frame_id = self._frame_fixed
        msg_pub.header.frame_id = rospy.Time.now()

        for label, panorama_rect in zip(msg_class.labels, msg_panorama_rects.panorama_rects):

            if label not in self._dimensions:
                rospy.logwarn('height for id {} is not specified.'.format(label))
                continue

            theta_center = panorama_rect.theta + panorama_rect.height / 2.0
            phi_center = panorama_rect.phi + panorama_rect.height / 2.0
            distance = 2.0 * self._dimensions[label][2] * math.tan( panorama_rect.height / 2.0 )

            p = PointStamped()
            (p.point.x, p.point.y, p.point.z) = calcPoint(theta_center,phi_center,distance)

            bbox = BoundingBox()
            bbox.header = msg_pub.header
            bbox.pose.position = tf2_geometry_msgs.do_transform_point(p,transform_fixed_to_panorama)
            bbox.pose.orientation.w = 1.0
            bbox.dimensions.x = self._dimensions[label][0]
            bbox.dimensions.y = self._dimensions[label][1]
            bbox.dimensions.z = self._dimensions[label][2]
            bbox.label = label

            msg_pub.boxes.append(bbox)

        self._pub.publish(msg_pub)

def main():

    rospy.init_node('panorama_object_to_bounding_box_array')
    node = PanoramaRectArrayToBoundingBoxArray()
    rospy.spin()

if __name__=='__main__':
    main()

