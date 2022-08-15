#! /usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA


def test_marker():
    publisher = rospy.Publisher(
        'marker_arti_thruttle', MarkerArray, queue_size=10)
    rospy.init_node('test_marker')
    rate = rospy.Rate(1)
    count = 0
    # team = rospy.get_param('team','DARPA')
    while not rospy.is_shutdown():
        marker_array = MarkerArray()
        markermsg = Marker()
        markermsg.type = Marker.TEXT_VIEW_FACING
        markermsg.header.frame_id = 'global'
        markermsg.id = count
        count += 1
        markermsg.header.stamp = rospy.Time.now()
        markermsg.pose.position.x = 0
        markermsg.pose.position.y = 0
        markermsg.pose.position.z = 0
        markermsg.pose.orientation.x = 0
        markermsg.pose.orientation.y = 0
        markermsg.pose.orientation.z = 0
        markermsg.pose.orientation.w = 0
        markermsg.color.r = 1
        markermsg.color.g = 1
        markermsg.color.b = 1
        markermsg.color.a = 0.7
        markermsg.text = "NCTU"
        markermsg.action = Marker.ADD
        markermsg.lifetime = rospy.Duration(0)
        marker_array.markers.append(markermsg)
        publisher.publish(marker_array)
        rate.sleep()


if __name__ == '__main__':
    try:
        test_marker()
    except rospy.ROSInterruptException:
        pass
