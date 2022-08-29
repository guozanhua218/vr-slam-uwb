#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class VRJSToCmdVel(object):
    def __init__(self):
        self.sub_js = rospy.Subscriber("vr/joystick_xy", Float32MultiArray, self.callback_js)
        self.pub_twist = (
            rospy.Publisher("robot1/cmd_vel", Twist, queue_size=10),
            rospy.Publisher("robot2/cmd_vel", Twist, queue_size=10),
            rospy.Publisher("robot3/cmd_vel", Twist, queue_size=10),
            rospy.Publisher("robot4/cmd_vel", Twist, queue_size=10),
        )

    def callback_js(self, data):
        cmd_vel = Twist()
        cmd_vel.angular.z = -data.data[0]
        cmd_vel.linear.x = data.data[1]
        for i in range(len(data.data) - 4):
            if data.data[i + 2]:
                self.pub_twist[i].publish(cmd_vel)


if __name__ == "__main__":
    rospy.init_node("sub_vr_joy", anonymous=True)
    vr_js_to_cmd_vel = VRJSToCmdVel()
    rospy.spin()
