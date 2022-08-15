#!/usr/bin/env python
import rospy
import tf
import math
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
class subvrandpub(object):
 
    def __init__(self):
        self.sub = rospy.Subscriber("/unity/robot1_goal", PoseStamped, self.callback1)
        self.pub_goal1 = rospy.Publisher('/robot1/move_base_simple/goal', PoseStamped, queue_size=10)

        self.sub = rospy.Subscriber("/unity/robot2_goal", PoseStamped, self.callback2)
        self.pub_goal2 = rospy.Publisher('/robot2/move_base_simple/goal', PoseStamped, queue_size=10)

        self.sub = rospy.Subscriber("/unity/robot3_goal", PoseStamped, self.callback3)
        self.pub_goal3 = rospy.Publisher('/robot3/move_base_simple/goal', PoseStamped, queue_size=10)
 
    def callback1(self, data):
 
        goal1 = data
        goal1.header.frame_id = "map"
        #unity goal for ros
        self.pub_goal1.publish(goal1)        
 
    def callback2(self, data):
 
        goal2 = data
        goal2.header.frame_id = "map"
        #unity goal for ros
        self.pub_goal2.publish(goal2)
 
    def callback3(self, data):
 
        goal3 = data
        goal3.header.frame_id = "map"
        #unity goal for ros
        self.pub_goal3.publish(goal3)
 
if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    SUB = subvrandpub()
    rospy.spin() #run forever