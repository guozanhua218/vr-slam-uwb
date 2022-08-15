#!/usr/bin/env python
import rospy
import tf
import math
from std_msgs.msg import Float32, Float64
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Joy
class subvrandpub(object):
 
    def __init__(self):
        self.which_car = rospy.get_param('~which_car',"robot1")
        # print("self.which_car-------------------",self.which_car)
        self.now_is_this_car = 0
        self.sub1 = rospy.Subscriber("/vr/joystick_xy", Float32MultiArray, self.callback_vr)
        self.sub2 = rospy.Subscriber("/vr/stick/val", Float64, self.callback_stick)
        self.pub_husky_stick = rospy.Publisher('stick_move_joint_position_controller/command',Float64, queue_size=10)
 
    def callback_vr(self, data):
        # print(self.which_car,"self.which_car")
        if (self.which_car=="robot1"):
            if(data.data[2]==1): self.now_is_this_car = True
            if(data.data[3]==1): self.now_is_this_car = False
            if(data.data[4]==1): self.now_is_this_car = False
        
        if (self.which_car=="robot2"):
            if(data.data[2]==1): self.now_is_this_car = False
            if(data.data[3]==1): self.now_is_this_car = True
            if(data.data[4]==1): self.now_is_this_car = False
        
        if (self.which_car=="robot3"):
            if(data.data[2]==1): self.now_is_this_car = False
            if(data.data[3]==1): self.now_is_this_car = False
            if(data.data[4]==1): self.now_is_this_car = True

    def callback_stick(self, data):
        if self.now_is_this_car: 
            self.pub_husky_stick.publish(data)

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    SUB = subvrandpub()
    rospy.spin() #run forever