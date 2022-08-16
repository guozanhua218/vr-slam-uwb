#! /usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Vector3

class subvrandpub(object):
 
    def __init__(self):
        # robot1
        self.sub_xy1 = rospy.Subscriber("/vr/joystick_xy", Float32MultiArray, self.callback_xy1)
        self.pub_twist1 = rospy.Publisher("/robot1/cmd_vel",Twist, queue_size=10)
        # # robot2
        self.sub_xy2 = rospy.Subscriber("/vr2/joystick_xy", Float32MultiArray, self.callback_xy2)
        self.pub_twist2 = rospy.Publisher("/robot2/cmd_vel",Twist, queue_size=10)
        # # robot3
        self.sub_xy3 = rospy.Subscriber("/vr3/joystick_xy", Float32MultiArray, self.callback_xy3)
        self.pub_twist3 = rospy.Publisher("/robot3/cmd_vel",Twist, queue_size=10)
 
    def callback_xy1(self, data):
        cmd_vel = Twist()
        print(data.data[0], data.data[1])
        cmd_vel.angular.z = -data.data[0]
        cmd_vel.linear.x= data.data[1]
        self.pub_twist1.publish(cmd_vel)

    def callback_xy2(self, data):
        cmd_vel = Twist()
        print(data.data[0], data.data[1])
        cmd_vel.angular.z = -data.data[0]
        cmd_vel.linear.x= data.data[1]
        self.pub_twist2.publish(cmd_vel)

    def callback_xy3(self, data):
        cmd_vel = Twist()
        print(data.data[0], data.data[1])
        cmd_vel.angular.z = -data.data[0]
        cmd_vel.linear.x= data.data[1]
        self.pub_twist3.publish(cmd_vel)


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    SUB = subvrandpub()
    rospy.spin() #run forever
