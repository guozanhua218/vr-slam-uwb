#! /usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
import tf


class WheelTF(object):
    def __init__(self):
        self.broadcaster = tf.TransformBroadcaster()
        self.fram_name = rospy.get_param("~frame_name", "/X1/base_link")
        self.paraent_name = rospy.get_param("~parent_name", "map")
        self.sub_wheel = rospy.Subscriber(
            "odom", Odometry, self.cb_odom, queue_size=1)

    def cb_odom(self, msg):
        self.broadcaster.sendTransform(
            (msg.pose.pose.position.x,
             msg.pose.pose.position.y, msg.pose.pose.position.z),
            (msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
            rospy.Time.now(),
            self.fram_name,
            self.paraent_name
        )


if __name__ == "__main__":
    rospy.init_node("wheel_tf")
    wheeltf = WheelTF()
    rospy.spin()
