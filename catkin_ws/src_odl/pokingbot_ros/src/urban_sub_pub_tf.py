#!/usr/bin/env python
import rospy
import tf
import math
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
class subvrandpub(object):
 
    def __init__(self):
        self.sub = rospy.Subscriber("/urban/truth_map_posestamped", PoseStamped, self.callback)
        self.pub_urabn_tf = rospy.Publisher('/urban/truth_map_unity',PoseStamped, queue_size=10)
 
    def callback(self, data):
        quaternion = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1] - 1.5708 #rotate
        yaw = euler[2] - 1.5708

        new_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        urabn_tf = data
        urabn_tf.pose.orientation.x = new_q[0]
        urabn_tf.pose.orientation.y = new_q[1]
        urabn_tf.pose.orientation.z = new_q[2]
        urabn_tf.pose.orientation.w = new_q[3]
        # urabn_tf.pose.position =  data.pose.position
        # urabn_tf.header.frame_id = "map"

        #unity
 
        self.pub_urabn_tf.publish(urabn_tf)

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    SUB = subvrandpub()
    rospy.spin() #run forever