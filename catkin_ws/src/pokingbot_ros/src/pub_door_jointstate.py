#! /usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState, GetPhysicsProperties, SetPhysicsProperties, SetPhysicsPropertiesRequest, GetJointProperties


class Door(object):
	def __init__(self):
		self.pub_door_1 = rospy.Publisher('/door_07/joint_states', JointState, queue_size=1)
		self.get_door_1_state = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
		self.pub_door_2 = rospy.Publisher('/door_12/joint_states', JointState, queue_size=1)
		self.get_door_2_state = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
		self.pub_door_3 = rospy.Publisher('/door_12_0/joint_states', JointState, queue_size=1)
		self.get_door_3_state = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        
	def door_1_state(self):
		rospy.wait_for_service("/gazebo/get_joint_properties")
		try:
			joint_name = "hinge_door_0.7::hinge"
			res = self.get_door_1_state(joint_name)
			# print(res)

			pub_msg = JointState()
			pub_msg.position = res.position
			pub_msg.name = ['door']
			self.pub_door_1.publish(pub_msg)
		except (rospy.ServiceException) as e:
			print(e)
			return 0        
	def door_2_state(self):
		rospy.wait_for_service("/gazebo/get_joint_properties")
		try:
			joint_name = "hinge_door_1.2::hinge"
			res = self.get_door_2_state(joint_name)
			# print(res)

			pub_msg = JointState()
			pub_msg.position = res.position
			pub_msg.name = ['door']
			self.pub_door_2.publish(pub_msg)
		except (rospy.ServiceException) as e:
			print(e)
			return 0

	def door_3_state(self):
		rospy.wait_for_service("/gazebo/get_joint_properties")
		try:
			joint_name = "hinge_door_1.2_0::hinge"
			res = self.get_door_3_state(joint_name)
			# print(res)

			pub_msg = JointState()
			pub_msg.position = res.position
			pub_msg.name = ['door']
			self.pub_door_3.publish(pub_msg)
		except (rospy.ServiceException) as e:
			print(e)
			return 0

if __name__ == "__main__":
	rospy.init_node("door_tf")
	dooooor = Door()
	while not rospy.is_shutdown():
		dooooor.door_1_state()
		rospy.sleep(0.01)
		dooooor.door_2_state()
		rospy.sleep(0.01)
		dooooor.door_3_state()
		rospy.sleep(0.01)
