#!/usr/bin/env python
import rospy
import tf
import math
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy
class subvrandpub(object):
 
    def __init__(self):
        self.which_car = rospy.get_param('~which_car',"robot1")
        # print("self.which_car-------------------",self.which_car)
        self.now_is_this_car = 0
        self.sub0 = rospy.Subscriber("/vr/joystick_xy", Float32MultiArray, self.callback_vr)
        # self.sub1 = rospy.Subscriber("/robot1/joy_teleop/joy", Joy, self.callback)
        self.sub2 = rospy.Subscriber("truth_map_posestamped", PoseStamped, self.callback_pose)
        self.sub3 = rospy.Subscriber("camera/rgb/image_raw/compressed", CompressedImage, self.callback_image)
        
        self.pub_husky_pose = rospy.Publisher('/robot0/truth_map_posestamped',PoseStamped, queue_size=10)
        self.pub_image = rospy.Publisher('/robot0/camera/rgb/image_raw/compressed',CompressedImage, queue_size=10)
        self.pub_twist_husky = rospy.Publisher("husky_velocity_controller/cmd_vel",Twist, queue_size=10)
        self.pub_twist_jackal = rospy.Publisher("x2_velocity_controller/cmd_vel",Twist, queue_size=10)
 
    def callback_vr(self, data):
        cmd_vel = Twist()
        # print(data.data[0], data.data[1])
        cmd_vel.angular.z = -data.data[0]
        cmd_vel.linear.x= data.data[1]
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

        # if self.now_is_this_car: self.pub_twist_husky.publish(cmd_vel)
        # if self.now_is_this_car: self.pub_twist_jackal.publish(cmd_vel)

    # def callback(self, data):
    #     # print(self.which_car,"self.which_car")
    #     if (self.which_car=="robot1"):
    #         if(data.buttons[3]==1): self.now_is_this_car = True
    #         if(data.buttons[1]==1): self.now_is_this_car = False
    #         if(data.buttons[0]==1): self.now_is_this_car = False
        
    #     if (self.which_car=="robot2"):
    #         if(data.buttons[3]==1): self.now_is_this_car = False
    #         if(data.buttons[1]==1): self.now_is_this_car = True
    #         if(data.buttons[0]==1): self.now_is_this_car = False
        
    #     if (self.which_car=="robot3"):
    #         if(data.buttons[3]==1): self.now_is_this_car = False
    #         if(data.buttons[1]==1): self.now_is_this_car = False
    #         if(data.buttons[0]==1): self.now_is_this_car = True
        

    def callback_pose(self, data):
        if self.now_is_this_car: self.pub_husky_pose.publish(data)


    def callback_image(self, data):
        if self.now_is_this_car: self.pub_image.publish(data)


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    SUB = subvrandpub()
    rospy.spin() #run forever