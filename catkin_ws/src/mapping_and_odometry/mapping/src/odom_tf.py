#!/usr/bin/env python
import numpy as np
import cv2
import roslib
import rospy
import tf
import struct
import math
import time
import rospkg

from nav_msgs.msg import Odometry


class Wheel_tf():
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.origin_matrix = None
        odom_sub = rospy.Subscriber(
            "wheel_odom", Odometry, self.cb_odom, queue_size=1)

    def cb_odom(self, msg):
        if self.origin_matrix is None:
            self.origin_matrix = tf.transformations.quaternion_matrix(
                [
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                ]
            )
            self.origin_matrix[0, 3] = msg.pose.pose.position.x
            self.origin_matrix[1, 3] = msg.pose.pose.position.y
            self.origin_matrix[2, 3] = msg.pose.pose.position.z
            self.origin_matrix = tf.transformations.inverse_matrix(
                self.origin_matrix)
        else:
            new_matrix = tf.transformations.quaternion_matrix(
                [
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                ]
            )
            new_matrix[0, 3] = msg.pose.pose.position.x
            new_matrix[1, 3] = msg.pose.pose.position.y
            new_matrix[2, 3] = msg.pose.pose.position.z

            new_matrix = np.dot(self.origin_matrix, new_matrix)

            quat = tf.transformations.quaternion_from_matrix(new_matrix)

            self.br.sendTransform(
                (
                    new_matrix[0, 3], new_matrix[1, 3], new_matrix[2, 3]
                ),
                (
                    quat[0], quat[1], quat[2], quat[3],
                ),
                msg.header.stamp, "base_link", "odom"
            )


if __name__ == '__main__':
    rospy.init_node('odom_tf')
    wheel_tf = Wheel_tf()
    rospy.spin()
